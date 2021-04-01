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
 * \file      lqr.h
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Eigen>
#include <math.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>




// ROS cuustom msg

#include <qb_interface/cubeRef.h>
#include <qb_interface/cubeCurrent.h>
#include <qb_interface/cubePos.h>

#include <phidgets_interface_kit/AnalogArray.h>

# define PI 3.1416
# define PI2 6.2832
# define MAX_LIN_VEL 2.0
# define MAX_ANG_VEL 0.5



class lqr
{
public:
  lqr();
  ~lqr();


  void run();
  void stop_motor();
  double dt;



private:

  void callback_gyro(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_acc(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_imu_euler(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_meas(const qb_interface::cubePos::ConstPtr& msg);
  void callback_comm(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_myo(const sensor_msgs::Imu::ConstPtr& msg);
  void callback_des_vel(const geometry_msgs::Vector3::ConstPtr& msg);
  void callback_offset_phi(const std_msgs::Float32::ConstPtr& msg);
  void callback_sensor_IR(const phidgets_interface_kit::AnalogArray::ConstPtr& msg);
  void callback_thrs(const std_msgs::Float32::ConstPtr& msg);
  void joy_android(const geometry_msgs::Twist::ConstPtr& msg);

  
  int sgn(double d);
  double unwrap(double previousAngle,double newAngle);
  double angleDiff(double a,double b);
  double kalman_dth(double y1, double y2, double ax, double az, double psi);

  double pos1_des;
  double pos2_des;
  double vel1_des;
  double vel2_des;

  double offset_pitch_;

  ros::Subscriber sub_gyro_, sub_acc_, sub_euler_, sub_des_vel;
  ros::Publisher pub_comm_, pub_rec_;
  ros::Subscriber sub_enc, sub_pitch_off_, sub_sensor_IR, sub_test_joy_android;
  // ros::Subscriber sub_trsh;


  qb_interface::cubeRef comm_pub_;

  Eigen::Vector3d gyro_, acc_, euler_;
  Eigen::VectorXd sensor_IR_, arr_sensor_;
  double command_int;
  double th_des_, dth_des_, phi_des_, dphi_des_, sensor_2_;
  double enc1_, enc2_, enc1_old_, enc2_old_, enc1_of_, enc2_of_, vel1_old_, vel2_old_, vel_old_, w_old_;
  double R_, W_, N_;
  double trsh_L_, trsh_R_;
  bool flag_run1_, flag_run2_, flag_run3_, flag_run4_;

  ros::NodeHandle n_;
  double com_R, com_L;

  Eigen::Vector2d x_pre_, x_old_;
  Eigen::Matrix2d P_pre_, P_old_;
  ros::Time time_cmd_vel_;
  ros::Duration max_dur_cmd_vel_;

  double dth_des_filt_, dth_des_filt_old_;
};//End of class SubscribeAndPublish
