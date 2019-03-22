/*
 * Copyright (C) 2017 Walk-Man
 * Author: Alessandro Settimi, Gianluca Lentini, Danilo Caporale
 * email: ale.settimi@gmail.com, danilo.caporale@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <mutex>
#include <udp_interface/udp_client.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <string>
#include <cstring>
#include <sstream>
#include <iostream>



#define IP "192.168.0.55"
// #define IP "127.0.0.1"
// #define PORT 5001

// #define RESIZE_FRAME_HEIGHT 394 //...562
// #define RESIZE_FRAME_WIDTH 700 //...1000
#define PACK_SIZE 8100 //udp pack size; note that OSX limits < 8100 bytes
#define ENCODE_QUALITY 60

/**
 * @brief gazebo oculus interface
 * 
 **/

namespace robot
{
    class GO_interface
    {
    public:
        GO_interface(int PORT);
	void send_images();

    private:
        ros::NodeHandle nh;
    	ros::Subscriber eye_zed_sub;

        void eye_zed_callback(const sensor_msgs::Image& msg);
        std::string to_string(double x);
        double round(double var); 




        bool left_zed_received = false;


    	sensor_msgs::ImageConstPtr left_zed_image;
    	std::mutex left_zed_mutex;
    	udp_client UDPsender;

    	cv::Mat frame;
    	// cv::Mat send;
    	std::vector < uchar > encoded;
    	int zedWidth = 1280;
    	int zedHeight = 720;
    	int jpegqual = ENCODE_QUALITY; // Compression Parameter
    	std::vector < int > compression_params;
    };
}

using namespace robot;

GO_interface::GO_interface(int PORT):UDPsender(IP,PORT)
{
    
    std::string topic_camera;
    nh.getParam("topic_camera", topic_camera);

    eye_zed_sub = nh.subscribe(topic_camera,1,&GO_interface::eye_zed_callback,this);

    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(jpegqual);

    frame = cv::Mat(zedHeight, zedWidth, CV_8UC3);

    // left = cv::Mat(frame, cv::Rect(0, 0, zedWidth, zedHeight));

    // right = cv::Mat(frame, cv::Rect(zedWidth, 0, zedWidth, zedHeight));
}


void GO_interface::eye_zed_callback(const sensor_msgs::Image& msg)
{
        //ROS_INFO("received left");
        left_zed_mutex.lock();
        left_zed_received = true;
        left_zed_image.reset(new sensor_msgs::Image(msg));
        // flag1 = true;
        left_zed_mutex.unlock();    
}

std::string GO_interface::to_string(double x)
{
  std::ostringstream ss;
  ss << x;
  return ss.str();
}

double GO_interface::round(double var) 
{ 
    double value = (int)(var * 100 + .5); 
    return (double)value / 100; 
} 


void GO_interface::send_images()
{
    //ROS_WARN("Sending");    

    left_zed_mutex.lock();
    if(!(left_zed_received))
    {
        left_zed_mutex.unlock();
        return;
    }
    else
    {
        cv_bridge::toCvCopy(left_zed_image, sensor_msgs::image_encodings::BGR8)->image.copyTo(frame);       
    }


    // cv::resize(frame, send, cv::Size(RESIZE_FRAME_WIDTH * 2, RESIZE_FRAME_HEIGHT));
    cv::imencode(".jpg", frame, encoded, compression_params);
    
     // cv::imshow("topic_camera", frame);
     // cv::waitKey(1);
    
    int total_pack = 1 + (encoded.size() - 1) / PACK_SIZE;
    int ibuf[1] = {total_pack};
    ////send size
    
    if(UDPsender.send((char *)(&ibuf[0]), sizeof(int)) == -1)
    {
	    std::cout<<"sendto() failed"<<std::endl;
	    return;
    }

    //send image
    for (int i = 0; i < total_pack; i++)
    {
    	if (UDPsender.send((char *)(&encoded[i * PACK_SIZE]), PACK_SIZE) == -1)
    	{
    		std::cout<<"sendto() failed"<<std::endl;
    		return;
    	}
    }
    
    // left_zed_received = false;

    left_zed_mutex.unlock();
   

}

int main(int argc, char* argv[])
{
    if(!ros::isInitialized())
	ros::init(argc,argv,"test_video_stream");
    ros::NodeHandle n;
    
    int port;
    n.getParam("port", port);

    GO_interface GOI(port);
    // ros::AsyncSpinner spinner(2);
    ros::Rate loop(30);
    // spinner.start();
    
    while(ros::ok())
    {
	    GOI.send_images();
        ros::spinOnce();
        loop.sleep();
    }

    return 1;
}

