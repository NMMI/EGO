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
 * \file      main_odometry_publisher.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double p_, v_, phi_, dphi_;


void state_callback(const geometry_msgs::Pose& msg)
{
  if(!isnan(msg.position.x))
  {
    p_ = msg.position.x;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.position.y))
  {
    v_ = msg.position.y;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.orientation.y))
  {
    phi_ = msg.orientation.y;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.orientation.z))
  {
    dphi_ = msg.orientation.z;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  
}



int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher_node");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom_wheel", 10);
  ros::Subscriber sub_state = n.subscribe("/state", 10, state_callback);

  tf::TransformBroadcaster odom_broadcaster;

  double R = 0.13;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;

  p_ = 0.0;
  v_ = 0.0;
  phi_ = 0.0;
  dphi_ = 0.0;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    double delta_x = (v_ * cos(phi_) * dt);
    double delta_y = (v_ * sin(phi_) * dt);
    

    x += delta_x;
    y += delta_y;
    th = phi_;

    vx = (v_ * cos(phi_));
    vy = (v_ * sin(phi_));
    vth = dphi_;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = R;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = R;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();

  }
}