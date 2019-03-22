#include <ros/ros.h>
#include <ros_myo/EmgArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>

# define PI 3.14159

class Joy_oculus_bridge
{
public:
  Joy_oculus_bridge();
  ~Joy_oculus_bridge();

  void run();
  double dt;




private:

	ros::NodeHandle n_;

	//HAND TRACKING
	void callback_track(const geometry_msgs::Pose::ConstPtr& msg);
	//INDEX TRIGGER
	void callback_index(const std_msgs::Float64::ConstPtr& msg);
	//INDEX Thumbstick left
	void callback_Thumbstick_L_y_(const std_msgs::Float64::ConstPtr& msg);
	//INDEX Thumbstick right
	void callback_Thumbstick_R_x_(const std_msgs::Float64::ConstPtr& msg);
	//Botton_A
	void callback_Botton_A(const std_msgs::Bool::ConstPtr& msg);
	//Botton_B
	void callback_Botton_B(const std_msgs::Bool::ConstPtr& msg);
	//Botton_X
	void callback_Botton_X(const std_msgs::Bool::ConstPtr& msg);

	ros::Subscriber sub_track_, sub_index_, sub_Thumbstick_L_y_, sub_Thumbstick_R_x_, sub_Botton_A_, sub_Botton_B_, sub_Botton_X_;
	ros::Publisher pub_pos_des_, pub_hand_, pub_status_, pub_vel_des_, pub_operate_mode_;

	Eigen::Vector3d joy_pos_, ref_pos_;
	Eigen::Quaterniond joy_quat_, quat_, quat_old_;
	//matrici T_0_-> calibration frame, T_c_->current frame from joy, T_->from T_0_ to T_c_, T_b0_ ->from frame robot base arm to T_0_
	Eigen::Matrix4d T_baseoculus2shoulder_, T_baseoculus2current_, T_shoulder2current_, T_;//, T_c_, T_, T_b0_;
	// Eigen::Matrix4d T_0_, T_c_, T_, T_b0_;
	Eigen::Matrix3d R_c_, R_, R_b0_, ref_orient_, R_baseEgo2shoulder_, rot_min90_;
	geometry_msgs::Pose des_pose_;
	double index_, index_old_, length_arm_, length_arm_ego_;
	int state_, count_calibration_;
	tf::TransformBroadcaster hand_broadcaster_;
	std::string arm_;
	std_msgs::Bool status_;
	bool flag_end_calibration_;
	geometry_msgs::Vector3 vel_des_;
	std_msgs::Int16 operate_mode_;

	ros::Time time_cmd_track_;
  	ros::Duration max_time_cmd_track_;



};//End of class SubscribeAndPublish