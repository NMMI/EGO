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
 * \file      Sensor.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------

#include <Sensor.h>

Sensor::Sensor()
{
	//Topic you want to publish
    pub_gyro_ = n_.advertise<geometry_msgs::Vector3>("gyro_good", 1);
    pub_acc_ = n_.advertise<geometry_msgs::Vector3>("acc_good", 1);
    pub_q_est_= n_.advertise<geometry_msgs::Quaternion>("q_est", 1);
    pub_euler_ = n_.advertise<geometry_msgs::Vector3>("RPY", 1);
    
	//Topic you want to subscribe
    sub_imu_acc_ = n_.subscribe("/qb_class_imu/acc", 1, &Sensor::callback_imu_acc, this);
    sub_imu_gyro_ = n_.subscribe("/qb_class_imu/gyro", 1, &Sensor::callback_imu_gyro, this);
    step_ = 0;
    n_sample_ = 800;

    data_1_ = Eigen::MatrixXd::Zero(n_sample_,3);
    data_2_ = Eigen::MatrixXd::Zero(n_sample_,3);
    gyro_old_ = Eigen::Vector3d::Zero();
    gyro_1_ = Eigen::Vector3d::Zero();
    gyro_2_ = Eigen::Vector3d::Zero();


    flag_run1_ = flag_run2_ =  false;
    flag_offset_ = true;

    q_old_.w() = 1.0; 
    q_old_.vec() << 0, 0.0, 0.0;  


    /* Kalman filter variables and constants */
	Q_angle = 0.001; // Process noise covariance for the accelerometer - Sw 0.001
	Q_gyro = 0.003; // Process noise covariance for the gyro - Sw 0.003
	R_angle = 0.3; // Measurement noise covariance - Sv 0.03

	angle = 0.0; // It starts at 180 degrees
	bias = 0.0;
	P_00 = P_01 = P_10 = P_11 = 0.0;


}

Sensor::~Sensor()
{

}


//tutte le acc provenienti dalle imu (NON MYO)
void Sensor::callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
	if(!isnan(msg->m[0].x) && !isnan(msg->m[0].y) && !isnan(msg->m[0].z) && !isnan(msg->m[1].x) && !isnan(msg->m[1].y) && !isnan(msg->m[1].z))
	{
		acc_1_ << msg->m[0].x, msg->m[0].y, msg->m[0].z;
		acc_2_ << msg->m[1].x, msg->m[1].y, msg->m[1].z;
		flag_run1_ = true;

	}
  	
  	

}

void Sensor::callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg)
{
	if(!isnan(msg->m[0].x) && !isnan(msg->m[0].y) && !isnan(msg->m[0].z) && !isnan(msg->m[1].x) && !isnan(msg->m[1].y) && !isnan(msg->m[1].z))
	{
		gyro_1_ << msg->m[0].x, msg->m[0].y, msg->m[0].z;
  		gyro_2_ << msg->m[1].x, msg->m[1].y, msg->m[1].z;
  		flag_run2_ = true;

	}
  	

}

void Sensor::offset_gyro(Eigen::Vector3d gyro1, Eigen::Vector3d gyro2)
{
	int j;
	Eigen::Vector3d diff_1, diff_2;

	Eigen::Quaterniond gyro_q;
	
	

	if(flag_run1_ && flag_run2_)
	{
		if(step_ == 0)
		{
			data_1_(step_,0) = gyro1(0);
			data_1_(step_,1) = gyro1(1);
			data_1_(step_,2) = gyro1(2);
			data_2_(step_,0) = gyro2(0);
			data_2_(step_,1) = gyro2(1);
			data_2_(step_,2) = gyro2(2);
			step_++;

		}
		else
		{	
			if(step_ < n_sample_)
			{
				diff_1(0) =  (gyro1(0) - data_1_(0,0));
				diff_1(1) =  (gyro1(1) - data_1_(0,1));
				diff_1(2) =  (gyro1(2) - data_1_(0,2));
				diff_2(0) =  (gyro2(0) - data_2_(0,0));
				diff_2(1) =  (gyro2(1) - data_2_(0,1));
				diff_2(2) =  (gyro2(2) - data_2_(0,2));

				diff_1 = diff_1.cwiseAbs();
				diff_2 = diff_2.cwiseAbs();

				for(j = 0; j < 3; j++)
				{
					if(diff_1(j) < 2)
					{
						data_1_(step_, j) = gyro1(j);
						gyro1_old_(j) = gyro1(j);
					}
					else
					{
						data_1_(step_, j) = gyro1_old_(j);
					}
				}

				for(j = 0; j < 3; j++)
				{
					if(diff_2(j) < 2)
					{
						data_2_(step_, j) = gyro2(j);
						gyro2_old_(j) = gyro2(j);
					}
					else
					{
						data_2_(step_, j) = gyro2_old_(j);
					}
				}
				step_++;
			}
			else 
			{	
				offset_1_(0) = data_1_.col(0).sum() / n_sample_;
				offset_1_(1) = data_1_.col(1).sum() / n_sample_;
				offset_1_(2) = data_1_.col(2).sum() / n_sample_;
				offset_2_(0) = data_2_.col(0).sum() / n_sample_;
				offset_2_(1) = data_2_.col(1).sum() / n_sample_;
				offset_2_(2) = data_2_.col(2).sum() / n_sample_;

				flag_offset_ = false;
			 std::cout<< "off1:"<< offset_1_(0)<<" "<<offset_1_(1)<<" "<<offset_1_(2)<<std::endl;
			 std::cout<< "off2:"<< offset_2_(0)<<" "<<offset_2_(1)<<" "<<offset_2_(2)<<std::endl;

			}
			

		}


	}
}

void Sensor::run()
{
	double Beta = 0.03;
	int i;
	double tr_g =  3.0;
	double tr_a = 0.08;


	geometry_msgs::Vector3 gyro_g_pub, acc_g_pub, euler_pub;
	Eigen::Vector3d diff_g, diff_a;

	//sono i gyro senza offset
	Eigen::Vector3d gyro_1_0, gyro_2_0;

	Eigen::Vector3d acc_g, gyro_g;
	Eigen::Vector3d Acc_earth(0.0, 0.0, 1.0);
	Eigen::VectorXd F_partial(3);
	Eigen::MatrixXd J_partial(3, 4);
	Eigen::VectorXd step_q(4);
	Eigen::Quaterniond q_step;
	Eigen::Quaterniond q_ang_rate;
	Eigen::Quaterniond q_nabla;
	Eigen::Quaterniond q_integr;
	Eigen::Quaterniond q_est;
	Eigen::Quaterniond qDot;
	Eigen::Quaterniond gyro_q;
	Eigen::Vector3d euler;


	if(flag_offset_)
	{
		offset_gyro(gyro_1_, gyro_2_);
	}
	else
	{
		gyro_1_0 = gyro_1_ - offset_1_;
		gyro_2_0 = gyro_2_ - offset_2_;

		diff_g = (gyro_1_0 - gyro_2_0);
		diff_g = diff_g.cwiseAbs();


		for(i = 0; i < 3; i++)
		{
			if(diff_g(i) < tr_g)
			{
				gyro_g(i) = (gyro_1_0(i) + gyro_2_0(i)) / 2;
				
			}
			else
			{
				
				gyro_g(i) = gyro_old_(i);
				
			}

			gyro_old_(i) =  gyro_g(i);
		}
		gyro_1_0_old_ = gyro_1_0;
		gyro_2_0_old_ = gyro_2_0;

		diff_a = (acc_1_ - acc_2_);
		diff_a = diff_a.cwiseAbs();

		for(i = 0; i < 3; i++)
		{
			if(diff_a(i) < tr_a)
			{
				acc_g(i) = acc_1_(i);
				
			}
			else
			{
				acc_g(i) = acc_old_(i);
			}
			acc_old_(i) =  acc_g(i);
		}

		acc_1_old_ = acc_1_;
		acc_2_old_ = acc_2_;


		//Normalizzazione acc
		acc_g = acc_g / acc_g.norm();

		//gradi to rad al sec
		gyro_g = gyro_g * (PI / 180);
		gyro_q.w() = 0;
		gyro_q.vec() = gyro_g;


		double tilt_angle = atan2(acc_g(0), -acc_g(2));
		euler (1) =  - kalman(tilt_angle, gyro_g(1)); // calculate the angle using a Kalman filter
  		euler (0) = 0.0;
  		euler (2) = 0.0;

		gyro_g_pub.x = gyro_g(0);
		gyro_g_pub.y = -gyro_g(1);
		gyro_g_pub.z = gyro_g(2);
	

		pub_gyro_.publish(gyro_g_pub);

		acc_g_pub.x = acc_g(0);
		acc_g_pub.y = acc_g(1);
		acc_g_pub.z = acc_g(2);

		pub_acc_.publish(acc_g_pub);

		euler_pub.x = euler(0);
		euler_pub.y = euler(1);
		euler_pub.z = euler(2);

		pub_euler_.publish(euler_pub);



	}
}




double Sensor::kalman(double newAngle, double newRate) {

    angle += dt * (newRate - bias);
    
    P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
    P_01 += -dt * P_11;
    P_10 += -dt * P_11;
    P_11 += +Q_gyro * dt;
  
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    y = newAngle - angle;
    angle += K_0 * y;
    bias += K_1 * y;
   
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    
    return angle;
}