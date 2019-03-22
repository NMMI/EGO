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
#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <pseudo_inversion.h>
#include <skew_symmetric.h>
// #include <eigen_conversions/eigen_kdl.h>



// // ROS cuustom msg

// #include <qb_interface/cubeRef.h>
// #include <qb_interface/cubeCurrent.h>
// #include <qb_interface/cubePos.h>

// #include <phidgets_interface_kit/AnalogArray.h>

# define PI 3.1416
# define PI2 6.2832


class inv_kin
{
public:
  inv_kin();
  ~inv_kin();



  void run();
  // Eigen::Quaterniond madgwick_kin(Eigen::Vector3d acc, Eigen::Vector3d gyro, Eigen::Quaterniond q_old, double State_joint_prev, Eigen::Vector3d Joint_prev, double State_joint_nxt, Eigen::Vector3d Joint_nxt);

  double dt;



private:
  void callback_right_des(const geometry_msgs::Pose::ConstPtr& msg);
  void callback_left_des(const geometry_msgs::Pose::ConstPtr& msg);
  void run_R();
  void run_L();
  int sgn(double d);


  Eigen::MatrixXd W_;
  KDL::Chain right_arm_chain_, left_arm_chain_;

//Right arm
  Eigen::Vector3d T_bt_R_;          //traslazione rigida dal sistema posto sul primo cubo del braccio destro al sistema torso
  Eigen::Matrix3d R_bt_R_;          //rotazione rigida dal sistema posto sul primo cubo del braccio destro al sistema torso
  Eigen::Quaterniond Q_bt_R_;       //rotazione rigida dal sistema posto sul primo cubo del braccio destro al sistema torso
  Eigen::Quaterniond Q_zmin90_;        //rotazione di -90 intorno a z
  Eigen::Vector3d pos_R_, pos_d_R_, e1_R_, e2_R_;
  Eigen::Matrix3d orient_R_;
  Eigen::Quaterniond quat_R_, quat_d_R_;
  Eigen::VectorXd e_R_;
  Eigen::MatrixXd Jac_R_, Jac_pinv_R_, k_R_, Jac_W_R_;
  Eigen::VectorXd qdot_R_, q_eig_R_, qdot_zero_R_;                                                                                                                             
  KDL::JntArray  q_R_;            // Joint positions
  KDL::Jacobian  J_R_;            // Jacobian
  KDL::Frame     x_R_;            // Tip pose                                                                                                                                       
  KDL::Frame     xd_R_;           // Tip desired pose                                                                                                                               
  KDL::Frame     x0_R_;           // Tip initial pose 
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_R_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_R_;
  Eigen::Matrix3d skew_R_;
  KDL::Vector quat_d_R_vec;
  


//Left arm
  Eigen::Quaterniond Q_zplus90_;        //rotazione di -90 intorno a z
  Eigen::Vector3d pos_L_, pos_d_L_, e1_L_, e2_L_;
  Eigen::Matrix3d orient_L_;
  Eigen::Quaterniond quat_L_, quat_d_L_;
  Eigen::VectorXd e_L_;
  Eigen::MatrixXd Jac_L_, Jac_pinv_L_, k_L_;
  Eigen::VectorXd qdot_L_, q_eig_L_;                                                                                                                             
  KDL::JntArray  q_L_;            // Joint positions
  KDL::Jacobian  J_L_;            // Jacobian
  KDL::Frame     x_L_;            // Tip pose                                                                                                                                       
  KDL::Frame     xd_L_;           // Tip desired pose                                                                                                                               
  KDL::Frame     x0_L_;           // Tip initial pose 
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_L_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_L_;
  Eigen::Matrix3d skew_L_;
  KDL::Vector quat_d_L_vec;



  ros::NodeHandle n_;


  ros::Publisher pub_right_q_, pub_left_q_;
  ros::Subscriber sub_right_des_, sub_left_des_;
  
  geometry_msgs::Pose q_right_pub_, q_left_pub_;

};


