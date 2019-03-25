// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include<winsock2.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library

#include <iostream>
#include <Windows.h>

#include <GL/glew.h>

#include <stddef.h>

/*Depending on the SDL version you are using, you may have to include SDL2/SDL.h or directly SDL.h (2.0.7)*/
#include <SDL.h>
#include <SDL_syswm.h>

#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <boost/lambda/lambda.hpp>
//#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/chrono.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <stdio.h>
#include <windows.h>
#include "ros.h"
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs\Float64.h>
#include <std_msgs\Bool.h>
#include <geometry_msgs\Vector3.h>
#include <myo/myo.hpp>



#include <sl/Camera.hpp>

#include "Shader.hpp"

extern bool flag_image_L_;
extern bool flag_left;
extern bool flag_image_R_;
extern bool flag_right;
extern bool flag1;
extern bool flag2;
extern bool flag3;
extern geometry_msgs::Vector3 myo_emg_R1_msg, myo_emg_R2_msg, myo_emg_R3_msg, myo_emg_L1_msg, myo_emg_L2_msg, myo_emg_L3_msg;

extern ros::NodeHandle nh;

extern ovrSession session;
extern ovrGraphicsLuid luid;;
extern ovrResult result;


extern std::mutex image_mtx_L_, image_mtx_R_;

extern cv::Mat frame_right_, frame_left_;
//extern ThreadData thread_data;

int left_image_task(void);
int right_image_task(void);
int oculus_images_task(void);
int tracking_task(void);
int myo_task(void);


//Config UDP
#define zedHeight 720
#define zedWidth 1280
#define LeftCamfx 687.7
#define FRAME_INTERVAL (1000/60)
#define PACK_SIZE 8100		//udp pack size; note that OSX limits < 8100 bytes
#define ENCODE_QUALITY 60

//UDP param
#define PORT_LEFT 2302		//The port on which to listen for left images
#define PORT_RIGHT 2303		//The port on which to listen for left images
#define BUF_LEN 131080		// Larger than maximum UDP packet size

#define leftHand 0
#define rightHand 1

