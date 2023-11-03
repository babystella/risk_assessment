#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <string>

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    image_transport::ImageTransport imt(nh);
    image_transport::Publisher pub = imt.advertise("risk/score",1);

    //cv::Mat image = cv::imread("/home/haow/CV_ws/src/fuzzy_ass/src/test.png", CV_LOAD_IMAGE_COLOR);
    cv::Mat image(100, 300, CV_8UC3, cv::Scalar(0));
    float test_framescore = 39.5353;
    std::string pi = "risk score is: " + std::to_string(int(test_framescore));
    cv::putText(image, //target image
            pi, //text
            cv::Point(10, image.rows / 2), //MIDDLE position
            cv::FONT_HERSHEY_DUPLEX,
            1.0, //size
            CV_RGB(255, 0, 0), //font color
            2 //thick ness
            );

    sensor_msgs::ImagePtr msg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ros::Rate looprate(5);
    while (nh.ok())
    {
        pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();

    }
    
    return 0;
}