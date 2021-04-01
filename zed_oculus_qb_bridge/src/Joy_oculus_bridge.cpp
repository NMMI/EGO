/***
 *  Software License Agreement: BSD 3-Clause License
 *
 * Copyright (c) 2019, NMMI
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ----------------------------------------------------------------------------

/**
 * \file      Joy_oculus_bridge.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------

#include <Joy_oculus_bridge.h>
Joy_oculus_bridge::Joy_oculus_bridge()
{
	std::string topic_track, topic_index, pub_pose_des, pub_hand, pub_status;
	std::vector<double> R_baseEgo2shoulder;

	n_.getParam("name", arm_);
	n_.getParam("topic_track", topic_track);
	n_.getParam("topic_index", topic_index);
	n_.getParam("pub_pose_des", pub_pose_des);
	n_.getParam("pub_hand", pub_hand);
	n_.getParam("pub_status", pub_status);
	n_.getParam("R_baseEgo2shoulder", R_baseEgo2shoulder);

	flag_end_calibration_ = false;
	state_ = 1;
	operate_mode_.data = 0; //0 -> real EGO, 1-> rviz environment 
	index_ = 0.0;
	index_old_ = 0.0;
	length_arm_ego_ = 0.348;
	count_calibration_ = 0;
	joy_quat_.w() = 1.0;
	joy_quat_.vec() << 0, 0, 0;

	quat_old_.w() = 1.0;
	quat_old_.vec() << 0, 0, 0;

	status_.data = false;
	
	
	R_baseEgo2shoulder_ <<  R_baseEgo2shoulder[0], R_baseEgo2shoulder[1], R_baseEgo2shoulder[2],
						   R_baseEgo2shoulder[3], R_baseEgo2shoulder[4], R_baseEgo2shoulder[5],
						   R_baseEgo2shoulder[6], R_baseEgo2shoulder[7], R_baseEgo2shoulder[8];

	rot_min90_ << 1.0, 0.0, 0.0,
		  		 0.0, 0.0, 1.0,
		  		 0.0,-1.0, 0.0;



	//Topic you want to subscribe
	//HAND TRACKING
    sub_track_ = n_.subscribe(topic_track, 10, &Joy_oculus_bridge::callback_track, this);
    //INDEX TRIGGER
    sub_index_ = n_.subscribe(topic_index, 10, &Joy_oculus_bridge::callback_index, this);
    //BUTTON A
    sub_Botton_A_ = n_.subscribe("/Button_A", 1, &Joy_oculus_bridge::callback_Botton_A, this);
    //BUTTON B
    sub_Botton_B_ = n_.subscribe("/Button_B", 1, &Joy_oculus_bridge::callback_Botton_B, this);
    //BUTTON X
    sub_Botton_X_ = n_.subscribe("/Button_X", 1, &Joy_oculus_bridge::callback_Botton_X, this);

    pub_pos_des_ = n_.advertise<geometry_msgs::Pose>(pub_pose_des, 10);
    pub_hand_ = n_.advertise<std_msgs::Float64>(pub_hand, 10);
    pub_status_ = n_.advertise<std_msgs::Bool>(pub_status, 10);

    if(arm_.compare("left_") == 0)
    {
    	//INDEX Thumbstick left
    	sub_Thumbstick_L_y_ = n_.subscribe("/LH_joy_y", 10, &Joy_oculus_bridge::callback_Thumbstick_L_y_, this);
    	sub_Thumbstick_R_x_ = n_.subscribe("/RH_joy_x", 10, &Joy_oculus_bridge::callback_Thumbstick_R_x_, this);

    	pub_vel_des_ = n_.advertise<geometry_msgs::Vector3>("/segway_des_vel", 10);
    	pub_operate_mode_ = n_.advertise<std_msgs::Int16>("/operate_mode", 10);


    	vel_des_.x = 0.0;
    	vel_des_.y = 0.0;
    	vel_des_.z = 0.0;

    }

	max_time_cmd_track_ = ros::Duration(1);
	time_cmd_track_ = ros::Time::now();
	

}

Joy_oculus_bridge::~Joy_oculus_bridge()
{

}

void Joy_oculus_bridge::callback_track(const geometry_msgs::Pose::ConstPtr& msg)
{	
	joy_pos_ << msg->position.x, msg->position.y, msg->position.z;
	joy_quat_.w() = msg->orientation.w;
	joy_quat_.vec() << msg->orientation.x, msg->orientation.y, msg->orientation.z;

	time_cmd_track_ = ros::Time::now();
}

void Joy_oculus_bridge::callback_index(const std_msgs::Float64::ConstPtr& msg)
{
	index_ = msg->data;
}

void Joy_oculus_bridge::callback_Thumbstick_L_y_(const std_msgs::Float64::ConstPtr& msg)
{
	vel_des_.y = msg->data * 2;
	if(state_ == 2) {
		pub_vel_des_.publish(vel_des_);
	}
}

void Joy_oculus_bridge::callback_Thumbstick_R_x_(const std_msgs::Float64::ConstPtr& msg)
{
	vel_des_.x = -msg->data / 2;
	if(state_ == 2) {
		pub_vel_des_.publish(vel_des_);
	}

}

void Joy_oculus_bridge::callback_Botton_A(const std_msgs::Bool::ConstPtr& msg)
{
	if(flag_end_calibration_) 
		{
			state_ = 3;
			operate_mode_.data = 1;
			pub_operate_mode_.publish(operate_mode_);
		}
	//std::cout<<"Sei fuori dal robot"<<std::endl;
}

void Joy_oculus_bridge::callback_Botton_X(const std_msgs::Bool::ConstPtr& msg)
{
	
}

void Joy_oculus_bridge::callback_Botton_B(const std_msgs::Bool::ConstPtr& msg)
{
	
}

void Joy_oculus_bridge::run()
{
	switch(state_)
	{
		case 1:
		{
			//calibration
			if((index_ < 0.5) && (index_old_ >= 0.5))
			{
				if(count_calibration_ == 0)
				{
					ref_pos_ =  joy_pos_;
					count_calibration_ ++;
					std::cout <<"START CALIBRATION"<<std::endl;
				}
				else if(count_calibration_ == 1)
				{	
					Eigen::Vector3d diff;

					diff = ref_pos_ - joy_pos_;
					double ipo = diff.norm();
					double ipo2 = sqrt(pow(diff.x(),2) +pow(diff.y(),2) +pow(diff.z(),2));
					std::cout<<"ipo"<<ipo<<std::endl;
					std::cout<<"ipo2"<<ipo2<<std::endl;
					length_arm_ =  sqrt((pow(diff.norm(),2) / 2.0)) ;

					std::cout<<"lunghezza braccio"<<length_arm_<<std::endl;

					Eigen::Matrix4d T_calibration;
					T_calibration.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
					T_calibration.block<3,1>(0,3) << 0.0, 0.0, length_arm_;
					T_calibration.block<1,3>(3,0) = Eigen::Vector3d::Zero();
		  			T_calibration(3,3) = 1;

		  			

		  			T_baseoculus2current_.block<3,3>(0,0) = rot_min90_;
					T_baseoculus2current_.block<3,1>(0,3) = joy_pos_;
					T_baseoculus2current_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
		  			T_baseoculus2current_(3,3) = 1;
					
					T_baseoculus2shoulder_ = T_baseoculus2current_ * T_calibration;

					
					count_calibration_ = 0;
					state_ = 2;

					flag_end_calibration_ = true;

					status_.data = true;
					pub_status_.publish(status_);

					std::cout <<"END CALIBRATION"<<std::endl;

					std::cout <<"Full Teleoperation Mode"<<std::endl;



				}
			}
			index_old_ = index_;

			break;
		}
		case 2:
		{
			//EGO FIRST PERSON TELEOPERATION
			R_c_ = joy_quat_;
			T_baseoculus2current_.block<3,3>(0,0) = R_c_;
			T_baseoculus2current_.block<3,1>(0,3) = joy_pos_;
		  	T_baseoculus2current_.block<1,3>(3,0) = Eigen::Vector3d::Zero();
		  	T_baseoculus2current_(3,3) = 1;

		  	Eigen::Matrix4d T_baseoculus2shoulder_inv;

		  	T_baseoculus2shoulder_inv.block<3,3>(0,0) = rot_min90_.transpose();
		  	T_baseoculus2shoulder_inv.block<3,1>(0,3) = -rot_min90_.transpose() * T_baseoculus2shoulder_.block<3,1>(0,3);
		  	T_baseoculus2shoulder_inv.block<1,3>(3,0) = Eigen::Vector3d::Zero();
		  	T_baseoculus2shoulder_inv(3,3) = 1;

		  	T_shoulder2current_ = T_baseoculus2shoulder_inv * T_baseoculus2current_;

		  	Eigen::Vector3d pos_ref;

		  	pos_ref = R_baseEgo2shoulder_ * T_shoulder2current_.block<3,1>(0,3);

		  	quat_ = R_baseEgo2shoulder_ * T_shoulder2current_.block<3,3>(0,0) * R_baseEgo2shoulder_.transpose();
		  

		  	quat_.normalize();


		  	// rotation to quaternion issue , "Sign Flip" , check  http://www.dtic.mil/dtic/tr/fulltext/u2/1043624.pdf
		  	double sign_check = quat_.w() * quat_old_.w() + quat_.x() * quat_old_.x() + quat_.y() * quat_old_.y() + quat_.z() * quat_old_.z();
		  	if(sign_check < 0.0)
		  	{
		  		quat_.w() = quat_.w() * (-1); 
		  		quat_.vec() = quat_.vec() * (-1); 
		  	}


		  	quat_old_ = quat_;



		  	des_pose_.position.x = (pos_ref.x() * length_arm_ego_) / length_arm_;
		  	des_pose_.position.y = (pos_ref.y() * length_arm_ego_) / length_arm_;
		  	des_pose_.position.z = (pos_ref.z() * length_arm_ego_) / length_arm_;
		  	
		  	des_pose_.orientation.w = quat_.w();
		  	des_pose_.orientation.x = quat_.x();
		  	des_pose_.orientation.y = quat_.y();
		  	des_pose_.orientation.z = quat_.z();

		  	if(ros::Time::now() > time_cmd_track_) //per evitare differenze negativi tra tempi
			{
				if((ros::Time::now() - time_cmd_track_) <= max_time_cmd_track_ )
				{
					pub_pos_des_.publish(des_pose_);
				}
			
			}

		  	//first, we'll publish the transform over tf
		    geometry_msgs::TransformStamped hand_trans;
		    hand_trans.header.stamp = ros::Time::now();
		    hand_trans.header.frame_id = arm_ + "shoulder_flage";
		    hand_trans.child_frame_id =  arm_ + "hand_ref";

		    hand_trans.transform.translation.x = des_pose_.position.x;
		    hand_trans.transform.translation.y = des_pose_.position.y;
		    hand_trans.transform.translation.z = des_pose_.position.z;
		    hand_trans.transform.rotation = des_pose_.orientation;
		    //send the transform
    		hand_broadcaster_.sendTransform(hand_trans);

		  	std_msgs::Float64 hand_closure;
		  	hand_closure.data = index_;
		  	pub_hand_.publish(hand_closure);

		  	
		    


			break;
		}
		case 3:
		{
			

			break;
		}
		case 4:
		{
			

			break;
		}
	}
}

