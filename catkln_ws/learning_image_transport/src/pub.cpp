#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_pub");
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/image/true", 1);

    cv::VideoCapture cap(0);
    cv::Mat frame;

    
    while(nh.ok())
    {
        cap >> frame;

        if(!frame.empty())
        {

            cv_bridge::CvImage img;

            img.encoding = sensor_msgs::image_encodings::BGR8;
            img.image = frame;
            img.header.frame_id = "camera_link";
            img.header.stamp = ros::Time::now();

            pub.publish(img.toImageMsg());

            // cv::imshow("frame", frame);

            // Encode, Decode image example            
            // std::vector<uchar> encode;
            // std::vector<int> encode_param;
            
            // encode_param.push_back(10);
            // encode_param.push_back(20);
            
            // cv::imencode(".jpg", frame, encode, encode_param);
            // cv::Mat decode = cv::imdecode(encode, 1);
            // cv::imshow("decode", decode);

            // // Convert encoded image to ROS std_msgs format
            // std_msgs::UInt8MultiArray msgArray;
            // msgArray.data.clear();
            // msgArray.data.resize(encode.size());
            // std::copy(encode.begin(), encode.end(), msgArray.data.begin());

            // // Publish msg
            // pub.publish(msgArray);

            cv::waitKey(1);
          
        }

        ros::spinOnce();
    }

    return 0;
    
}
