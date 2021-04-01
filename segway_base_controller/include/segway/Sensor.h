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
 * \file      Sensor.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------


#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Eigen>
// ROS cuustom msg
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
// #include <qb_interface/cubeRef.h>

# define PI 3.14159


class Sensor
{
public:
  Sensor();
  ~Sensor();


  void run();
  // Eigen::Quaterniond madgwick_kin(Eigen::Vector3d acc, Eigen::Vector3d gyro, Eigen::Quaterniond q_old, double State_joint_prev, Eigen::Vector3d Joint_prev, double State_joint_nxt, Eigen::Vector3d Joint_nxt);


  double dt;



private:

  void callback_imu_acc(const qb_interface::inertialSensorArray::ConstPtr& msg);
  void callback_imu_gyro(const qb_interface::inertialSensorArray::ConstPtr& msg);

  void offset_gyro(Eigen::Vector3d gyro1, Eigen::Vector3d gyro2);
  double kalman(double newAngle, double newRate);

  ros::Subscriber sub_imu_acc_, sub_imu_gyro_;
  ros::Publisher pub_gyro_, pub_acc_, pub_q_est_, pub_euler_;

  Eigen::Vector3d acc_1_, gyro_1_;
  Eigen::Vector3d acc_2_, gyro_2_;
  int step_;
  Eigen::Vector3d offset_1_, offset_2_;
  bool flag_run1_, flag_run2_, flag_offset_;
  ros::NodeHandle n_; 

  int n_sample_;
  Eigen::MatrixXd data_1_, data_2_;
  Eigen::Vector3d gyro1_old_, gyro2_old_;

  Eigen::Vector3d gyro_old_;
  Eigen::Vector3d acc_old_;
  Eigen::Quaterniond q_old_;
  Eigen::Vector3d gyro_1_0_old_, gyro_2_0_old_;
  Eigen::Vector3d acc_1_old_, acc_2_old_;

  geometry_msgs::Quaternion q_est_pub_;

/* Kalman filter variables and constants */
double Q_angle; // Process noise covariance for the accelerometer - Sw
double Q_gyro; // Process noise covariance for the gyro - Sw
double R_angle; // Measurement noise covariance - Sv

double angle; // It starts at 180 degrees
double bias;
double P_00 , P_01 , P_10 , P_11 ;
double y, S;
double K_0, K_1;

};//End of class SubscribeAndPublish