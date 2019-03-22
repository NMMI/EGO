#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdexcept>
#include <string>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <vector>
#include <iostream>

#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Bool.h>

#include <kdl/frames.hpp>
#include <eigen3/Eigen/Eigen>



Eigen::Vector3d head_pos_;
Eigen::Quaterniond head_quat_;

bool status_calibration_L_, status_calibration_R_, state_, flag_head_; 


void head_track_callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    head_pos_ << msg->position.x, msg->position.y, msg->position.z;
    head_quat_.w() = msg->orientation.w;
    head_quat_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;
    flag_head_ = true;

}

void cal_status_L_callback(const std_msgs::Bool::ConstPtr& msg)
{
    status_calibration_L_ = msg->data;
}

void cal_status_R_callback(const std_msgs::Bool::ConstPtr& msg)
{
    status_calibration_R_ = msg->data;
}

void callback_Botton_A(const std_msgs::Bool::ConstPtr& msg)
{
    state_ = false;
    std::cout<<"Sei fuori dal robot"<<std::endl;
}

int main(int argc, char** argv)
{

    ros::init(argc,argv,"head_oculus_bridge");
    ros::NodeHandle nh;

    bool flag = false;
    state_ = true;
    flag_head_ = false;

    
    double or_Y_old = 0;
    double or_X_old = 0;

    status_calibration_L_ = false;
    status_calibration_R_ = false;

    Eigen::Quaterniond Q0, Qd;

    Qd.w() = 1.0;
    Qd.vec() << 0.0, 0.0, 0.0;

    geometry_msgs::Vector3 orientation;
    orientation.x = 0.0;
    orientation.y = 0.0;
    orientation.z = 0.0;

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/gaze", 10);


    ros::Subscriber sub_head_win = nh.subscribe("/Head_track",1,&head_track_callback);  
    ros::Subscriber sub_calibration_L = nh.subscribe("/left_arm/calibration_status",10,&cal_status_L_callback);  
    ros::Subscriber sub_calibration_R = nh.subscribe("/right_arm/calibration_status",10,&cal_status_R_callback);
    //BUTTON A
    ros::Subscriber sub_Botton_A_ = nh.subscribe("/Button_A", 1, &callback_Botton_A);

    ros::Rate loop(100);
    
    while(ros::ok())
    {

        if(status_calibration_L_ || status_calibration_R_)
        // {
        //if(flag_head_)
        {
            if(!flag)
            {
                Q0 = head_quat_;
                flag = true;
            }
            else
            {
                Qd = Q0.conjugate() * head_quat_;
            }
        }
            

        // }
    	KDL::Rotation::Quaternion(Qd.x(),Qd.y(),Qd.z(),Qd.w()).GetRPY(orientation.x,orientation.y,orientation.z);
    	
    // 	std::cout<<"received: "<<q_x<<" "<<q_y<<" "<<q_z<<" "<<q_w<<std::endl;
	    orientation.z = orientation.y;
	    orientation.y = -orientation.x;
	
    	if(std::isnan(orientation.x) || std::isnan(orientation.y) || std::isnan(orientation.z)) continue;

        if(fabs(orientation.z) >= 1.3){
            orientation.y = or_Y_old;
            orientation.x = or_X_old;
        }
        else{
            or_Y_old = orientation.y;
            or_X_old = orientation.x;
        }

        if(state_) pub.publish(orientation);


    	ros::spinOnce();
        loop.sleep();
    }

    return 1;
}