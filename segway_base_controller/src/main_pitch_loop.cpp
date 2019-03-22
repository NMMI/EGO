#include <ros/ros.h>
#include <ros/rate.h>
#include <eigen3/Eigen/Eigen>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

# define PI 3.14159


//sistema di riferimento utilizzato nel pitch loop : x up, y left
KDL::JntArray  q_R_, q_L_;            // Joint positions


void meas_R_callback(const geometry_msgs::Pose& msg)
{
	if(!isnan(msg.position.x))
	{
		q_R_(0) = msg.position.x;
		// std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
	}
  if(!isnan(msg.position.y))
  {
    q_R_(1) = msg.position.y;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.position.z))
  {
    q_R_(2) = msg.position.z;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.orientation.x))
  {
    q_R_(3) = -msg.orientation.x;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.orientation.y))
  {
    q_R_(4) = msg.orientation.y;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
	

}

void meas_L_callback(const geometry_msgs::Pose& msg)
{
  if(!isnan(msg.position.x))
  {
    q_L_(0) = msg.position.x;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.position.y))
  {
    q_L_(1) = msg.position.y;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.position.z))
  {
    q_L_(2) = msg.position.z;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.orientation.x))
  {
    q_L_(3) = -msg.orientation.x;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  if(!isnan(msg.orientation.y))
  {
    q_L_(4) = msg.orientation.y;
    // std::cout<<"misura 1:"<<cube_meas1 <<std::endl;
  }
  

}



//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 100;
  
  double d1 = 0.084;
  double d2 = 0.090;
  double d3 = 0.078;
  double d4 = 0.090;
  double d5 = 0.090;

  Eigen::Matrix3d orient_R_, orient_L_;
  Eigen::Vector3d tran_90mm, tmp;
  tran_90mm << 0.0, 0.0, 0.090;

  KDL::Chain right_arm_chain_, left_arm_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_R_, jnt_to_pose_solver_L_;
  KDL::Frame     x_R_, x_L_;            // Tip pose  

  //DH del braccio destro(libreria KDL)
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d1, 0.0)));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, 0.0, PI/2.0)));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d2 + d3, -PI/2.0)));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, 0.0, 0.0)));
  right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d4 + d5, 0.0)));

  //DH del braccio sinistro(libreria KDL)
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d1, 0.0)));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, 0.0, -PI/2.0)));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d2 + d3, PI/2.0)));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, 0.0, 0.0)));
  left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d4 + d5, 0.0)));


  // Construct the kdl solvers in non-realtime. 
  jnt_to_pose_solver_R_.reset(new KDL::ChainFkSolverPos_recursive(right_arm_chain_));
  jnt_to_pose_solver_L_.reset(new KDL::ChainFkSolverPos_recursive(left_arm_chain_));

  q_R_.resize(right_arm_chain_.getNrOfJoints());
  q_L_.resize(left_arm_chain_.getNrOfJoints());
  KDL::SetToZero(q_R_);
  KDL::SetToZero(q_L_);

  //colonna -> componenti del vettore, riga -> link del braccio di riferimento
  Eigen::MatrixXd CM_R_links(right_arm_chain_.getNrOfJoints(), 3);
  Eigen::MatrixXd CM_L_links(left_arm_chain_.getNrOfJoints(), 3);


  // double offset = 0;
  std_msgs::Float32 offset_phi_pub; 
  geometry_msgs::Vector3 CM_Vect;
  Eigen::Vector4d CM1_L, CM2_L, CM3_L, CM4_L, CM5_L, CM1_R, CM2_R, CM3_R, CM4_R, CM5_R, CM_torso, CM;
  
  Eigen::Matrix4d T_R, T_L , R_R_torso_cube, R_L_torso_cube, R_base_torso, R_z_pi;

  // traslazione base (assiale) -> primo cubo braccio destro
  T_R << 1, 0, 0, (0.087 + 0.546 + 0.010),
  		   0, 1, 0,                  -0.108,
  		   0, 0, 1,                  -0.008,
  		   0, 0, 0,                       1;
  // rotazione torso -> primo cubo braccio destro
  R_R_torso_cube << 0.173648, -0.171010,  -0.969846, 0.0,
                         0.0,  0.984808,  -0.173648, 0.0,
                    0.984808, 0.030154,    0.171010, 0.0,
                           0,        0,           0,   1;
  // traslazione base (assiale) -> primo cubo braccio sinistro
  T_L << 1, 0, 0, (0.087 + 0.546 + 0.010),
  		   0, 1, 0,                   0.108,
  		   0, 0, 1,                  -0.008,
  		   0, 0, 0,                       1;

  // rotazione torso -> primo cubo braccio sinistro
  R_L_torso_cube << 0.173648, 0.171010,  0.969846, 0.0,
                         0.0, 0.984808, -0.173648, 0.0,
                   -0.984808, 0.030154,  0.171010, 0.0,
                           0,        0,         0,   1;
  // rotazione base (assiale) -> torso (Rx(pi/2)*Ry(pi/2))
  R_base_torso << 0.0, 0.0, 1.0, 0.0,
                  1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0, 
                  0.0, 0.0, 0.0, 1.0;
  // rotazione intorno a z di pi, necessaria per portarmi dal sistema primo cubo -> DH primo cubo (necessaria solo al sinistro, nel destro già coincide)
  R_z_pi << -1.0,  0.0, 0.0, 0.0,
             0.0, -1.0, 0.0, 0.0,
             0.0,  0.0, 1.0, 0.0,
             0.0,  0.0, 0.0, 1.0;

  //sistema di riferimento base(x up, y left)
  CM_torso << (0.218 + 0.087), 0.0, 0.005, 1.0;


  CM1_R << 0.0,0.0,0.0, 1;
  CM2_R << 0.0,0.0,0.0, 1;
  CM3_R << 0.0,0.0,0.0, 1;
  CM4_R << 0.0,0.0,0.0, 1;
  CM5_R << 0.0,0.0,0.0, 1;
  CM1_L << 0.0,0.0,0.0, 1;
  CM2_L << 0.0,0.0,0.0, 1;
  CM3_L << 0.0,0.0,0.0, 1;
  CM4_L << 0.0,0.0,0.0, 1;
  CM5_L << 0.0,0.0,0.0, 1;



  offset_phi_pub.data = 0.0;

  ros::init(argc, argv, "pitch_loop");
  ros::NodeHandle n_;

  ros::Rate r(rateHZ);
  // ros::Subscriber cube_1, cube_2, cube_3, cube_4, cube_5, cube_11, cube_12, cube_13, cube_14, cube_15;
  ros::Subscriber sub_meas_R, sub_meas_L;
  ros::Publisher  pub_CM = n_.advertise<geometry_msgs::Vector3>("CM", 10);
  ros::Publisher  pub_offset_phi = n_.advertise<std_msgs::Float32>("offset_phi", 10);

  //Topic you want to subscribe
  sub_meas_R = n_.subscribe("/measure_R", 10, meas_R_callback);
  sub_meas_L = n_.subscribe("/measure_L", 10, meas_L_callback);

  std::cout << " CALCOLO empiricamente il CENTRO DI MASSA" <<std::endl;

  while(ros::ok())
  {

    for(int i = 0; i < right_arm_chain_.getNrOfJoints() ; i++)
    {
      jnt_to_pose_solver_R_->JntToCart(q_R_, x_R_, i + 1);


      //per il link 1 e 3 bisogna aggiungere una traslazione di 0.090 lungo z (non è considerata in DH)
      if((i == 1) || (i == 3))
      {
        //prendo orientazione e la trasformo in eigen
        for(int k = 0; k < 3 ; k++)
        {
          for(int z = 0; z < 3 ; z++)
          {
            orient_R_(k, z) = x_R_.M(k, z);

          }
            
        }

        for(int j = 0; j < 3 ; j++)
        {
          tmp(j) = x_R_.p(j);
        }
        tmp = (orient_R_ * tran_90mm) + tmp;

        for(int j = 0; j < 3 ; j++)
        {
          CM_R_links(i,j) = tmp(j);
        }
      }
      else
      {
        for(int j = 0; j < 3 ; j++)
        {
          CM_R_links(i,j) = x_R_.p(j);
        }
      }
    }

    for(int i = 0; i < left_arm_chain_.getNrOfJoints() ; i++)
    {
      jnt_to_pose_solver_L_->JntToCart(q_L_, x_L_, i + 1);

      //per il link 1 e 3 bisogna aggiungere una traslazione di 0.090 lungo z (non è considerata in DH)
      if((i == 1) || (i == 3))
      {
        //prendo orientazione e la trasformo in eigen
        for(int k = 0; k < 3 ; k++)
        {
          for(int z = 0; z < 3 ; z++)
          {
            orient_L_(k, z) = x_L_.M(k, z);

          }
            
        }

        for(int j = 0; j < 3 ; j++)
        {
          tmp(j) = x_L_.p(j);
        }
        tmp = (orient_L_ * tran_90mm) + tmp;

        for(int j = 0; j < 3 ; j++)
        {
          CM_L_links(i,j) = tmp(j);
        }
      }
      else
      {
        for(int j = 0; j < 3 ; j++)
        {
          CM_L_links(i,j) = x_L_.p(j);
        }
      }
     
    }

    CM1_R << CM_R_links(0,0), CM_R_links(0,1), CM_R_links(0,2), 1;
    CM2_R << CM_R_links(1,0), CM_R_links(1,1), CM_R_links(1,2), 1;
    CM3_R << CM_R_links(2,0), CM_R_links(2,1), CM_R_links(2,2), 1;
    CM4_R << CM_R_links(3,0), CM_R_links(3,1), CM_R_links(3,2), 1;
    CM5_R << CM_R_links(4,0), CM_R_links(4,1), CM_R_links(4,2), 1;

    CM1_L << CM_L_links(0,0), CM_L_links(0,1), CM_L_links(0,2), 1;
    CM2_L << CM_L_links(1,0), CM_L_links(1,1), CM_L_links(1,2), 1;
    CM3_L << CM_L_links(2,0), CM_L_links(2,1), CM_L_links(2,2), 1;
    CM4_L << CM_L_links(3,0), CM_L_links(3,1), CM_L_links(3,2), 1;
    CM5_L << CM_L_links(4,0), CM_L_links(4,1), CM_L_links(4,2), 1;

  	
  	CM1_R = T_R * R_base_torso * R_R_torso_cube * CM1_R;
  	CM2_R = T_R * R_base_torso * R_R_torso_cube * CM2_R;
  	CM3_R = T_R * R_base_torso * R_R_torso_cube * CM3_R;
  	CM4_R = T_R * R_base_torso * R_R_torso_cube * CM4_R;
    CM5_R = T_R * R_base_torso * R_R_torso_cube * CM5_R;

  	CM1_L = T_L * R_base_torso * R_L_torso_cube * R_z_pi * CM1_L;
  	CM2_L = T_L * R_base_torso * R_L_torso_cube * R_z_pi * CM2_L;
  	CM3_L = T_L * R_base_torso * R_L_torso_cube * R_z_pi * CM3_L;
  	CM4_L = T_L * R_base_torso * R_L_torso_cube * R_z_pi * CM4_L;
    CM5_L = T_L * R_base_torso * R_L_torso_cube * R_z_pi * CM5_L;



  	CM = ((CM_torso * 12.8) + (CM1_R * 0.6) + (CM2_R * 0.64) + (CM3_R * 0.59) + (CM4_R * 0.35) + (CM5_R * 0.38)  + (CM1_L * 0.6) + (CM2_L * 0.64) + (CM3_L * 0.59) + (CM4_L * 0.35) + (CM5_L * 0.38))/(12.8 + ((0.6 + 0.64 + 0.59 + 0.35 + 0.38)*2));
    offset_phi_pub.data = (vel_des - vel_curr) * kp + (pos_des - pos_curr) * ki;
  	offset_phi_pub.data = atan2(-CM(2), CM(0));

  	CM_Vect.x = CM(0);
  	CM_Vect.y = CM(1);
  	CM_Vect.z = CM(2);

  	pub_CM.publish(CM_Vect);
  	pub_offset_phi.publish(offset_phi_pub);

    ros::spinOnce();
    r.sleep();
        
  }// end while()
return 0;
}