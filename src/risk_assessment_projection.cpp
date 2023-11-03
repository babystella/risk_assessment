#include "ros/ros.h"
#include <std_msgs/Header.h>
#include "yolact_ros_msgs/Box.h"
#include "yolact_ros_msgs/Detection.h"
#include "yolact_ros_msgs/Detections.h"
#include "yolact_ros_msgs/Mask.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <cmath>
#include <string>
#include <vector> 
#include <algorithm>
#include <Eigen/Dense>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>
#include "risk_assessment/risk_location.h" 
#include "risk_assessment/risk_locations.h" 
#include "risk_assessment/risk_score.h" 

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// some global variables
float currentx = 0.0;
float currenty = 0.0;
float global_score = 0.0;
float prior_framescore = 0.0;
int zero_cnt = 0;
ros::Publisher risk_score_pub;
ros::Publisher risk_location_pub;

std::vector<std::string> objects = {"poison", "infectious substance", "non-flammable gas", "inhalation hazard", 
									"corrosive", "radioactive","dangerous when wet", "oxygen", "organic peroxide",
									"explosive", "flammable solid", "spontaneously combustible","flammable gas" };

// struct of fuzzy score calculation
struct fuzzyscore
{
    int risklevel;
	// high, medium, low
    float fuzzy_degree[3];
	float riskResultScore;
};


float hyper_param[3][3][2] = {
    {{0, 0.5}, {1.7, 0.75}, {5, 1.8}},
    {{0, 1.2}, {3, 1}, {5, 1.4}},
    {{0, 2.8}, {2.9, 0.6}, {3.3, 0.2}}};

void gaussianass(int level, float input, float* scorearray)
{
	if(level == -1)
	{
		for (int i = 0; i<3;i++)
		{
			scorearray[i] = 0.0;
		}
	}
	else
	{
		for (int i = 0; i<3;i++)
		{
			scorearray[i] = exp(    (-1.0) * pow(input-hyper_param[level-1][i][0], 2.0)     /     (2* pow(hyper_param[level-1][i][1], 2.0))    ) * (level*0.5+0.5);
			//std::cout<< "mu:" << hyper_param[level-1][i][0] << "," << "sigma:" << hyper_param[level-1][i][1] << "," << "degree of membership:" << scorearray[i] << std::endl;
		}

	}
}

int findLlevel(const std::string& s1)
{
	int Level = -1;
	int idx = 0;
	std::vector<std::string>::iterator iter = find(objects.begin(), objects.end(), s1);
	idx = std::distance(objects.begin(), iter);
	//ROS_INFO("idx == %d", idx);
	if (idx<=3)
	{
		Level = 1;
	}
	else if (3<idx && idx<=8)
	{
		Level = 2;
	}
	else if (8<idx && idx<=12)
	{
		Level = 3;
	}
	else
	{
		ROS_INFO("Object not found!");
	}
	//ROS_INFO("Level == %d", Level);
	return Level;
}


void yolactcallback(const yolact_ros_msgs::Detections Detection)
{
	std::vector<fuzzyscore> scores;
	float framescore = 0.0;
    float scorearray[3];
	float norm_framescore = 0;
	double a = 0.09;
	double b = -4.8;
	// ros::Publisher risk_pub;
	Eigen::Vector4d Point3D_camera;
	risk_assessment::risk_score score;
	risk_assessment::risk_locations location;
	risk_assessment::risk_location location_msg;



	double X_world = 1;
	double Y_world = 1;
	double Z_world = 1;
	std::string detection_name;		
	

	double u = 0.0;
	double v = 0.0;
	double cx = 1;
	double cy = 1;
	double fx = 1;
	double fy = 1;
	double Coordiante_x_camera = 0; // location in the camera frame
	double Coordiante_y_camera = 0;
	double Coordiante_z_camera = 0;


    for (int i =0; i<Detection.detections.size(); ++i)
    {
        fuzzyscore currentscore;
        std::vector<risk_assessment::risk_location> LOCATION;
        int level = findLlevel(Detection.detections[i].class_name.c_str());
		//ROS_INFO("%d", level);
		//ROS_INFO("%.2f", Detection.detections[i].distance);
        gaussianass(level, Detection.detections[i].distance*0.001, scorearray);

        currentscore.risklevel = level;
        currentscore.fuzzy_degree[0] = scorearray[0];
        currentscore.fuzzy_degree[1] = scorearray[1];
        currentscore.fuzzy_degree[2] = scorearray[2];
		// ROS_INFO("%.2f, %.2f,%.2f",currentscore.fuzzy_degree[0], currentscore.fuzzy_degree[1],currentscore.fuzzy_degree[2]);
		currentscore.riskResultScore = currentscore.fuzzy_degree[0] * 50 +  currentscore.fuzzy_degree[1] * 10 + currentscore.fuzzy_degree[2] * 5;
        scores.push_back(currentscore);

		// normalize the result

		u = Detection.detections[i].location_x;
		v = Detection.detections[i].location_y;
		
		Coordiante_x_camera = (u - cx) * Detection.detections[i].distance*0.001 / fx;
		Coordiante_y_camera = (v - cy) * Detection.detections[i].distance*0.001 / fy;
		Coordiante_z_camera = Detection.detections[i].distance*0.001;

		Point3D_camera << Coordiante_x_camera, Coordiante_y_camera, Coordiante_z_camera, 1.0; //cameaa tf
		
		Eigen::Matrix4d T_cam_to_world;

		T_cam_to_world << 0.866, -0.5, 0.0, 1.0,
                     0.5, 0.866, 0.0, 2.0,
                     0.0, 0.0, 1.0, 3.0,
                     0.0, 0.0, 0.0, 1.0;   //check tf
		
		Eigen::Vector4d Point3D_world = T_cam_to_world * Point3D_camera;

		double X_world = Point3D_world(0) * 0.001;
    	double Y_world = Point3D_world(1) * 0.001;
    	double Z_world = Point3D_world(2) * 0.001;
	
		ROS_INFO("detection: %s, X:%.2f, Y:%.2f, Z:%.2f", Detection.detections[i].class_name.c_str(), X_world, Y_world, Z_world);

		// risk.risk_X[i] = X_world;
		// risk.risk_Y[i] = Y_world;
		// risk.risk_Z[i] = Z_world;

	
		// location.risklocations[i].risk_X = X_world;
		// location.risklocations[i].risk_Y = Y_world;
		// location.risklocations[i].risk_Z = Z_world;



		location_msg.risk_X  = X_world;
		location_msg.risk_Y  = Y_world;
		location_msg.risk_Z  = Z_world;
		location_msg.type_of_risk  =  Detection.detections[i].class_name.c_str();
		location_msg.risk_SEP = scorearray[i];


		location.risklocations.push_back(location_msg);
		// location.risklocations[i].risk_SEP = scorearray[i];
		// location.risklocations[i].type_of_risk = Detection.detections[i].class_name.c_str();

		// LOCATION.push_back(location);
		
    }

	location.header.stamp = ros::Time::now();
	risk_location_pub.publish(location);


	for(int i=0; i<scores.size();++i)
	{
		// find the max of scores[i].score, and it's position
		framescore += scores[i].riskResultScore;
	}
	
	norm_framescore = 1/(1 + exp(-(a*framescore + b)));

	if (norm_framescore !=0)// have detections
	{
		prior_framescore = norm_framescore;
		zero_cnt = 0;
	}
	else if (zero_cnt < 20) // no detection under 20
	{
		zero_cnt++;
		norm_framescore= prior_framescore;
	}
	else// no detection over 20
	{
		prior_framescore = 0.0;
		zero_cnt = 0;
	}


	ROS_INFO("risk: %.2f", prior_framescore);
	score.riskScore = prior_framescore;
			// set the message timestamp
	score.header.stamp = ros::Time::now();
			// Publish the message
    risk_score_pub.publish(score);
} 

int main(int argc, char *argv[])
{

	// Init ROS
	ros::init(argc, argv, "risk");
	ROS_INFO("Start risk assessment, waiting for detections result...");
	ros::NodeHandle nh;

	// Only subscribe to detections messages, this can be used for testing.
	ros::Subscriber sub = nh.subscribe<yolact_ros_msgs::Detections>("/yolact_ros/detections", 1, yolactcallback);

	risk_score_pub = nh.advertise<risk_assessment::risk_score>("/risk/score",1);
	risk_location_pub = nh.advertise<risk_assessment::risk_locations>("/risk/locations",1);
	
	// Score GUI
	image_transport::ImageTransport imgt(nh);
	image_transport::Publisher scoreImgpub = imgt.advertise("risk/score_image",5);
	

	//cv::Mat image(100, 300, CV_8UC3, cv::Scalar(0));
	
		// 10 hz update
	ros::Rate loop_rate(10);
	
	
	// Start wating for the publisher
	while(ros::ok())
	{		
			
		
		ros::spinOnce();
		//cv::Mat image(100, 300, CV_8UC3, cv::Scalar(0));
		std::string scoreString = "risk score is: " + std::to_string(int(prior_framescore));
	    cv::Mat image(100, 300, CV_8UC3, cv::Scalar(0));
		cv::putText(image, //target image
            scoreString, //text
            cv::Point(10, image.rows / 2), //MIDDLE position
            cv::FONT_HERSHEY_DUPLEX,
            1.0, //size
            CV_RGB(255, 0, 0), //font color
            2 //thick ness
            );
		sensor_msgs::ImagePtr scoreImgmsg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		scoreImgpub.publish(scoreImgmsg);
		if (prior_framescore>70)
		{
			std::cout<<"High Risk!!!"<<std::endl;
			std::cout<<"risk score is: "<<int(prior_framescore)<<std::endl;
		}

		// 1 hz update
		loop_rate.sleep();


	}

	// End of the program
	// experimentwirte.close();
	// std::cout<< "experimentwirte.csv finish" << std::endl;

	std::cout<< "Risk assessment end" << std::endl;
	return 0;
}
