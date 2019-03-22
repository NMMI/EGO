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