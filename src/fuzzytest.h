#include "ros/ros.h"
#include "yolact_ros_msgs/Box.h"
#include "yolact_ros_msgs/Detection.h"
#include "yolact_ros_msgs/Detections.h"
#include "yolact_ros_msgs/Mask.h"
#include "std_msgs/String.h"
#include <sstream>
#include <cmath>
#include <string>
#include <vector> 
#include <algorithm>

// L1_high_mu, L1_high_sigma = 0, 0.3
// L1_medium_mu, L1_medium_sigma = 1.5, 0.5
// L1_low_mu, L1_low_sigma = 5, 1.5

// L2_high_mu, L2_high_sigma = 0, 0.625
// L2_medium_mu, L2_medium_sigma = 2.5, 0.625
// L2_low_mu, L2_low_sigma = 5, 0.625

// L3_high_mu, L3_high_sigma = 0, 1.5
// L3_medium_mu, L3_medium_sigma = 3.5, 0.5
// L3_low_mu, L3_low_sigma = 5, 0.3

// L1 objects = {"poison", "infectious-substance", "non-flammable-gas", "inhalation-hazard"};
// L2 objects = {"corrosive", "dangerous", "flammable", "organic-peroxide"};
// L3 objects = {"explosive", "radioactive", "flammable-solid", "spontaneously-combustible", "oxygen"};

std::vector<std::string> objects = {"poison", "infectious-substance", "non-flammable-gas", "inhalation-hazard", 
									"corrosive", "dangerous", "flammable", "organic-peroxide",
									"explosive", "radioactive", "flammable-solid", "spontaneously-combustible", "oxygen"};


struct fuzzyscore
{
    int risklevel;
    float highscore;
    float mediumscore;
    float lowscore;
};

void gaussianass(int level, float input, float* scorearray)
{
    float mu = 0.1;
    float sigma = 0.2;

	for (int i = 0; i<3;i++)
	{
		scorearray[i] = exp(-1.0 * pow(input-mu, 2.0) / 2* pow(sigma, 2.0));
		ROS_INFO("haha: %f", fuzzy[i]);
	}
}

int findLlevel(const std::string& s1)
{
	int Level = -1;
	int idx = 0;
	std::vector<std::string>::iterator iter = find(objects.begin(), objects.end(), s1);
	idx = std::distance(objects.begin(), iter);
	ROS_INFO("idx == %d", idx);
	if (idx<=3)
	{
		Level = 1;
	}
	else if (3<idx && idx<=7)
	{
		Level = 2;
	}
	else if (7<idx && idx<=12)
	{
		Level = 3;
	}
	else
	{
		ROS_INFO("Object not found!");
	}
	ROS_INFO("Level == %d", Level);
	return Level;
}


void yolactcallback(const yolact_ros_msgs::Detections Detection)
{
	std::vector<fuzzyscore> scores;
    float scorearray[3];
    for (int i =0; i<Detections.detections.size(); ++i)
    {
        fuzzyscore currentscore;
        
        int level = findLlevel(Detections.detections[i].class_name.c_str());
        gaussianass(level, Detections.detection[i].distace, scorearray);
        
        currentscore.risklevel = level;
        currentscore.highscore = scorearray[0];
        currentscore.mediumscore = scorearray[1];
        currentscore.lowscore = scorearray[2];
        scores.push_back(currentscore);
    }
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fuzzyass");
	
	ROS_INFO("Start risk assessment, waiting for detections result...");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe<yolact_ros_msgs::Detections>("/yolact_ros/detections", 10, yolactcallback);

	ros::spin();
	
	return 0;
}
