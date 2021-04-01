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
 * \file      lqr.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------

#include <lqr.h>

lqr::lqr()
{
	//Topic you want to publish
    pub_comm_ = n_.advertise<qb_interface::cubeRef>("qb_class/cube_ref", 1);	//base motor ref (duty cycle [-100,100])
    pub_rec_ = n_.advertise<geometry_msgs::Pose>("state", 1);				//debug

	//Topic you want to subscribe
    sub_gyro_= n_.subscribe("/gyro_good", 1, &lqr::callback_gyro, this);						//gyro measurements 
    sub_euler_ = n_.subscribe("/RPY", 1, &lqr::callback_imu_euler, this);						//we use only pitch for self balance
    sub_enc = n_.subscribe("/qb_class/cube_measurement", 1, &lqr::callback_meas, this);		//mase encoder motor
    sub_pitch_off_ = n_.subscribe("/offset_phi", 1, &lqr::callback_offset_phi, this);			//offset pitch due to COM movement
	sub_des_vel = n_.subscribe("/segway_des_vel", 10, &lqr::callback_des_vel, this);			//reference linear and angular velocity
	sub_sensor_IR = n_.subscribe("/cl4_gpio/analog_in", 1, &lqr::callback_sensor_IR, this);	//IR sensors value
	

    com_R = 0.0;
    com_L = 0.0;
    

    comm_pub_.p_1.push_back(-com_R);
    comm_pub_.p_2.push_back(com_L);

    ros::Rate loop(10);
    ros::Duration init(3.0);
    ros::Time start = ros::Time::now();
    while(ros::Time::now()-start<init)
    {
        loop.sleep();
		pub_comm_.publish(comm_pub_);
    }

    flag_run1_ = flag_run2_ = flag_run3_ =  false;	

    pos1_des = 0.0;
    pos2_des = 0.0;
    vel1_des = 0.0;
    vel2_des = 0.0;
    command_int = 0.0;

    offset_pitch_ = 0.0;//-0.09;
    sensor_IR_ = Eigen::VectorXd::Zero(8);
    arr_sensor_ = Eigen::VectorXd::Zero(10);
    th_des_ = 0.0;
    dth_des_ = 0.0;
    phi_des_ = 0.0;
    dphi_des_ = 0.0;
	sensor_2_ = 0.0;

	enc1_ = 0;
	enc2_ = 0;
	enc1_old_ = 0;
	enc2_old_ = 0;
	vel1_old_ = 0;
	vel2_old_ = 0;
	R_ = 0.13;
	W_ = 0.496;
	N_ = 1/0.2727;
	std::cout<<"Raggio: "<< R_ << ", Distanza ruote: "<< W_<<", Rapporto di riduzione: "<< N_<<std::endl;
	std::cout<<"Il nodo che pubblica la vel_des e il sensore IR devono andare alla stessa freq del controllo, il filtraggio del rif vieni fatto in questo nodo "<<std::endl;

	x_pre_ = Eigen::Vector2d::Zero();
	x_old_ = Eigen::Vector2d::Zero();
	P_pre_ = Eigen::Matrix2d::Zero();
	P_old_ << 0.0001, 	 0,
				  0, 0.0001;

	dth_des_filt_ = 0.0;
	dth_des_filt_old_ = 0.0;
	trsh_L_ = 20;
	trsh_R_ = 20;

	max_dur_cmd_vel_ = ros::Duration(0.1);
	time_cmd_vel_ = ros::Time::now();
	


}

lqr::~lqr()
{

}


void lqr::joy_android(const geometry_msgs::Twist::ConstPtr& msg)
{
	dth_des_ =  msg->linear.x;
	dphi_des_ =  msg->angular.z;
}

void lqr::callback_des_vel(const geometry_msgs::Vector3::ConstPtr& msg)
{	
	if(!isnan(msg->y))
  	{
  		dth_des_ =  msg->y;
	}
	if(!isnan(msg->x))
  	{
		dphi_des_ =  msg->x;
	}


	if(fabs(dth_des_) > MAX_LIN_VEL) dth_des_ = sgn(dth_des_) * MAX_LIN_VEL;
	if(fabs(dphi_des_) > MAX_ANG_VEL) dphi_des_ = sgn(dphi_des_) * MAX_ANG_VEL;

	time_cmd_vel_ = ros::Time::now();
}

void lqr::callback_gyro(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	gyro_ << msg->x, msg->y, msg->z;
  	flag_run1_ = true;

}

void lqr::callback_imu_euler(const geometry_msgs::Vector3::ConstPtr& msg)
{
  	euler_ << msg->x, msg->y, msg->z;
  	flag_run2_ = true;

}



void lqr::callback_meas(const qb_interface::cubePos::ConstPtr& msg)
{
	if(!flag_run3_)
	{
		if(!isnan(msg->p_1[0]) && !isnan(msg->p_2[0]))
		{
			enc1_ = msg->p_1[0] * 4.0 * PI / 65536;
			enc2_ = msg->p_2[0] * 4.0 * PI / 65536;
			enc1_of_ = enc1_;
			enc2_of_ = enc2_;
			enc1_ = 0;
			enc2_ = 0;
			
			flag_run3_ = true;
		}
		
	}
	if(!isnan(msg->p_1[0]) && !isnan(msg->p_2[0]))
	{
		enc1_ = msg->p_1[0] * 4.0 * PI / 65536  - enc1_of_;
		enc2_ = msg->p_2[0] * 4.0 * PI / 65536  - enc2_of_;
	}

}


void lqr::callback_offset_phi(const std_msgs::Float32::ConstPtr& msg)
{
  	offset_pitch_ = msg->data;

}

void lqr::callback_sensor_IR(const phidgets_interface_kit::AnalogArray::ConstPtr& msg)
{
  	// controlla NAN
  	if(msg->values.size()<8) return;

  	if(!isnan(msg->values[2]))
  	{
      sensor_IR_(2) = fabs(9462.0 / (msg->values[2] - 16.92));     // distanza in cm
  	}

  	arr_sensor_ << arr_sensor_.tail(arr_sensor_.size() - 1), sensor_IR_(2);
  	sensor_2_ = arr_sensor_.mean();
}



void lqr::run()
{
	//range in cui il sensore 20 150 è sicuro che non sbaglia
	double inf_lim = 25.0;
	double sup_lim = 80.0;

	double enc1_g, enc2_g, vel1_enc, vel2_enc, vel1, vel2, pos1, pos2, a, th, dth, phi, dphi;

	//dati rimanenti da registrare con rosbag
	geometry_msgs::Pose rec_pub;
	nav_msgs::Odometry odom;


	double k_int;
	Eigen::MatrixXd k_feed(2,6);
	Eigen::MatrixXd command(2,1), command_feed(2,1);
	Eigen::MatrixXd state(6,1);
	

	k_int = -10.78;

	k_feed << -20.5,   -1816.9, -95.1, -250.8,  60.2236, 50.2114, 
	      -20.5,   -1816.9, -95.1, -250.8, -60.2236, -50.2114;

	a = dt / (0.05 + dt);

	
	if(flag_run1_ && flag_run2_ && flag_run3_)
	{
		if(ros::Time::now() > time_cmd_vel_) //per evitare differenze negativi tra tempi
		{
			

			if((ros::Time::now() - time_cmd_vel_) > max_dur_cmd_vel_ )
			{
				dth_des_ = 0.0;
				dphi_des_ = 0.0;
			}
		}

		//posizione e velocità relativa struttura ruote
		enc1_g = unwrap(enc1_old_, enc1_) ;
		enc2_g = unwrap(enc2_old_, enc2_) ;

		vel1_enc = (enc1_g - enc1_old_)/dt;
		vel2_enc = (enc2_g - enc2_old_)/dt;


		enc1_old_ = enc1_g;
		enc2_old_ = enc2_g;

		// posizine/velocità assoluta ruote pos1 -> right , pos2->left  tenendo conto del rapporto di riduzione N_
		pos1 = euler_(1) - (enc1_g / N_);
		pos2 = euler_(1) + (enc2_g / N_);

		vel1 = gyro_(1) - (vel1_enc / N_);
		vel2 = gyro_(1) + (vel2_enc / N_);

		
		vel1 = (1 - a) * vel1_old_ + (a * vel1);
		vel2 = (1 - a) * vel2_old_ + (a * vel2);

		vel1_old_ = vel1;
		vel2_old_ = vel2;

		th = 0.5 * (pos1 + pos2);
		dth = 0.5 * (vel1 + vel2);
		phi = (R_ / W_) * (pos1 - pos2);
		dphi = (R_ / W_) * (vel1 - vel2);

		// //filtraggio della velocità lineare desiderata in funzione del sensore IR//////////////////////
		// if(dth_des_ > 0.0)
		// {
		// 	if(sensor_2_<= inf_lim)
		// 	{
		// 		dth_des_filt_ = 0;
				
		// 	}
		// 	else
		// 	{
		// 		if(sensor_2_ >=sup_lim)
		// 		{
		// 			dth_des_filt_ = dth_des_;

		// 		}
		// 		else
		// 		{
		// 			//variazione velocità des
		// 			// dth_des_ = 0;

		// 			dth_des_filt_ = dth_des_ * ((sensor_2_- inf_lim) / (sup_lim - inf_lim));
		// 			if(dth_des_filt_ <= 0.02)
		// 			{
		// 				dth_des_filt_ = 0;
		// 			}


		// 		}
		// 	}

		// }
		// if(dth_des_ <= 0.0)
		// {
		// 	dth_des_filt_ = dth_des_;

		// }
		//senza IR
		dth_des_filt_ = dth_des_;

		dth_des_filt_ = (1 - 0.002) * dth_des_filt_old_ + (0.002 * dth_des_filt_);
		dth_des_filt_old_ = dth_des_filt_;

		// std::cout << "velocità filtrata: " << dth_des_filt_ << std::endl;

		////////////////////////////////fine filtraggio///////////////////////////////////////////////////
		

		// std::cout << "velocità filtrata: " << dth_des_filt_ << std::endl;

		th_des_ += dth_des_filt_ * dt;
		phi_des_ += dphi_des_ * dt;

		//controllo LQR
		state << (th_des_ - th), 0.0 - (euler_(1) + offset_pitch_), (dth_des_filt_ - dth), 0 - gyro_(1), (phi_des_ - phi), (dphi_des_ - dphi);
		command_feed = k_feed * state;

//------------------------------------------------------------------------------//
		command(0,0) = command_feed (0,0) ;
		command(1,0) = command_feed (1,0) ;
//------------------------------------------------------------------------------//
		com_R = sgn(command(0,0))*trsh_R_ + (command(0,0));
		com_L = sgn(command(1,0))*trsh_L_ + (command(1,0));

		
		

		if(com_R < -100)
		{
			com_R = -100;
		}
		
		if (com_R > 100)
		{
			com_R = 100;
		}

		if(com_L < -100)
		{
			com_L = -100;
		}
		
		if (com_L > 100)
		{
			com_L = 100;
		}

		//comando ruote
		comm_pub_.p_1[0] = (com_R);
		comm_pub_.p_2[0] = (-com_L);


		pub_comm_.publish(comm_pub_);

		rec_pub.position.x = th * R_;
		rec_pub.position.y = dth * R_;
		rec_pub.position.z = vel1;
		rec_pub.orientation.w = vel2;
		rec_pub.orientation.x = phi_des_;
		rec_pub.orientation.y = phi;
		rec_pub.orientation.z = dphi;

		

		pub_rec_.publish(rec_pub);
	}
}

int lqr::sgn(double d){
    return d<0? -1 : d>0; 
    }

double lqr::unwrap(double previousAngle,double newAngle){
    return previousAngle - angleDiff(newAngle,previousAngle);
}

double lqr::angleDiff(double a,double b){
    double dif = std::fmod(b - a + PI,PI2);
    if (dif < 0)
        dif += PI2;
    return dif - PI;
}

double lqr::kalman_dth(double y1, double y2, double ax, double az, double psi) {
    
	Eigen::Matrix2d A,C, Q, P, R,Inv, I, L;
	Eigen::Vector2d B, x, y;
	double u;

	A << 1, 0,
		dt, 1;
	
	B << dt, 0;

	C << 1, 0,
		 0, 1;


	Q << 0.01, 0,
			0, 0.01;

	R << 1, 0,
			0, 0.001;

	I = Eigen::Matrix2d::Identity();


	y << y1, y2;
	u = (-sin(psi) * ax + cos(psi) * az) * 9.81;
    x_pre_ = A * x_old_ + B *u;
    P_pre_ = A * P_old_ * A.transpose() + Q;

    //Correction
    Inv = C * P_pre_ * C.transpose() + R;
    L = P_pre_ * C.transpose() * Inv.inverse();
    x = x_pre_ + L * (y - C * x_pre_);
    P = (I - L*C) * P_pre_;


    x_old_ = x;
    P_old_ = P;

    return x(0);
}

  void lqr::stop_motor()
  {
  	com_R = 0.0;
    com_L = 0.0;
    
    comm_pub_.p_1.push_back(-com_R);
    comm_pub_.p_2.push_back(com_L);
	pub_comm_.publish(comm_pub_);

  }
