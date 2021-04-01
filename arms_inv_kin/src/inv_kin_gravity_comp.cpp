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
 * \file      inv_kin_gravity_comp.cpp
 *
 * \author       _Centro di Ricerca "E.Piaggio"_
 * \author       _Istituto Italiano di Tecnologia, Soft Robotics for Human Cooperation and Rehabilitation Lab_
**/
// ----------------------------------------------------------------------------

#include <inv_kin_gravity_comp.h>

#define STIFFNESS_ARMS 0.3

inv_kin_gravity_comp::inv_kin_gravity_comp()
{
	
	bool EMG = false; 
	
	double d1 = 0.0;
	double d2 = 0.090;
	double d3 = 0.078;
	double d4 = 0.090;
	double d5 = 0.090;


	identity_.resize(5,5);
	identity_ = Eigen::MatrixXd::Identity(5,5);
	Q_zmin90_.w() = 0.7071;
	Q_zmin90_.vec() << 0.0, 0.0, -0.7071;

	Q_zplus90_.w() = 0.7071;
	Q_zplus90_.vec() << 0.0, 0.0, 0.7071;

	// rotazione rigida  da braccio destro a torso 
	Eigen::Matrix3d Rot_R2t, Rot_y_min90;
	Rot_R2t << 0.173648, 	  0.0,  0.984808,
			  -0.171010, 0.984808, 0.0301537,
			  -0.969846,-0.173648,  0.171010;

	Rot_y_min90 << 0.0, 0.0, -1.0,
	  		       0.0, 1.0,  0.0,
	       		   1.0, 0.0,  0.0;

	Rot_R_ = Rot_R2t * Rot_y_min90;
	Q_Rot_R_ = Rot_R_;


	Eigen::Vector3d grav_R(0.0, 0.0, -9.81);
	grav_R = Rot_R2t * grav_R;
	gravity_R_ = KDL::Vector(grav_R(0), grav_R(1), grav_R(2));


	// rotazione rigida  da braccio sinistro a torso 
	Eigen::Matrix3d Rot_L2t, Rot_z_pi, Rot_t2myo;
	Rot_z_pi << -1.0,  0.0, 0.0,
			   	 0.0, -1.0, 0.0,
			   	 0.0,  0.0, 1.0;

	Rot_L2t << 0.173648, 	  0.0, -0.984808,
			   0.171010, 0.984808, 0.0301537,
			   0.969846,-0.173648,  0.171010;

	Rot_t2myo << 0.0, 0.0, 1.0,
			   0.0, -1.0, 0.0,
			   1.0, 0.0,  0.0;

	Rot_L_ = Rot_z_pi * Rot_L2t * Rot_t2myo;
	Q_Rot_L_ = Rot_L_;

	Eigen::Vector3d grav_L(0.0, 0.0, -9.81);
	grav_L = Rot_z_pi * Rot_L2t * grav_L;
	gravity_L_ = KDL::Vector(grav_L(0), grav_L(1), grav_L(2));


	//costanti derivate dal cubo datasheet
	a_motor_ = 2.0;
	k_motor_ = 0.5;

 	inert_R_1_ = KDL::RigidBodyInertia(0.6, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
 	inert_R_2_ = KDL::RigidBodyInertia(0.643, KDL::Vector(0.0, 0.0, (0.09 - 0.007)));
 	inert_R_3_ = KDL::RigidBodyInertia(0.59, KDL::Vector(0.0, 0.0, 0.0));
 	inert_R_4_ = KDL::RigidBodyInertia(0.35, KDL::Vector(0.0, 0.0, (0.09 - 0.013)));
 	inert_R_5_ = KDL::RigidBodyInertia(0.38, KDL::Vector(0.0, 0.0, 0.0));//mano

 	inert_L_1_ = KDL::RigidBodyInertia(0.6, KDL::Vector(0.0, 0.0, 0.0)); // secondo cubo
 	inert_L_2_ = KDL::RigidBodyInertia(0.643, KDL::Vector(0.0, 0.0, (0.09 - 0.007)));
 	inert_L_3_ = KDL::RigidBodyInertia(0.59, KDL::Vector(0.0, 0.0, 0.0));
 	inert_L_4_ = KDL::RigidBodyInertia(0.35, KDL::Vector(0.0, 0.0, (0.09 - 0.013)));
 	inert_L_5_ = KDL::RigidBodyInertia(0.38, KDL::Vector(0.0, 0.0, 0.0));//mano


	//DH del braccio destro(libreria KDL)
	right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d1, 0.0), inert_R_1_));
	right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, 0.0, PI/2.0), inert_R_2_));
	right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d2 + d3, -PI/2.0), inert_R_3_));
	right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, 0.0, 0.0), inert_R_4_));
	right_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d4 + d5, 0.0), inert_R_5_));


	//DH del braccio sinistro(libreria KDL)
	left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, d1, 0.0), inert_L_1_));
	left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, 0.0, -PI/2.0), inert_L_2_));
	left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d2 + d3, PI/2.0), inert_L_3_));
	left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, -PI/2.0, 0.0, 0.0), inert_L_4_));
	left_arm_chain_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0, PI/2.0, d4 + d5, 0.0), inert_L_5_));

	// chain.addSegment(Segment(Joint(Joint::RotZ),Frame::DH(double a, double alpha, double d, double theta)));
	
	// Construct the kdl solvers in non-realtime.
	jnt_to_pose_solver_R_.reset(new KDL::ChainFkSolverPos_recursive(right_arm_chain_));
	jnt_to_jac_solver_R_.reset(new KDL::ChainJntToJacSolver(right_arm_chain_));
	id_solver_R_.reset(new KDL::ChainDynParam(right_arm_chain_,gravity_R_));


	jnt_to_pose_solver_L_.reset(new KDL::ChainFkSolverPos_recursive(left_arm_chain_));
	jnt_to_jac_solver_L_.reset(new KDL::ChainJntToJacSolver(left_arm_chain_));
	id_solver_L_.reset(new KDL::ChainDynParam(left_arm_chain_,gravity_L_));



	// Resize (pre-allocate) the variables in non-realtime.                                                                                                                         
	q_R_.resize(right_arm_chain_.getNrOfJoints());
	J_R_.resize(right_arm_chain_.getNrOfJoints());
	KDL::SetToZero(q_R_);
	pos_R_.Zero();
	// pos_d_R_ << -0.348, 0.0 ,0.084;
	pos_d_R_ << -0.348, 0.0 ,0.0;

	quat_d_R_.w() = 1.0;
	quat_d_R_.vec() << 0.0, 0.0, 0.0;
	quat_d_R_ = quat_d_R_ * Q_zmin90_;
	e1_R_.Zero();
	e2_R_.Zero();
	orient_R_.Zero();
	e_R_.resize(6);
	Jac_R_.resize(6, right_arm_chain_.getNrOfJoints());
	Jac_pinv_R_.resize(right_arm_chain_.getNrOfJoints(), 6);
	k_R_ = Eigen::Matrix<double, 6, 6>::Identity() * 5.0;
	qdot_R_.resize(right_arm_chain_.getNrOfJoints());
	q_eig_R_.resize(right_arm_chain_.getNrOfJoints());
	q_eig_R_ = Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints());
	defl_R_.resize(right_arm_chain_.getNrOfJoints());
	defl_R_ = Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints());
	G_comp_R_.resize(right_arm_chain_.getNrOfJoints());
	KDL::SetToZero(G_comp_R_);
	qpreset_R_ = STIFFNESS_ARMS;
	q0_R_.resize(right_arm_chain_.getNrOfJoints());



	q_L_.resize(left_arm_chain_.getNrOfJoints());
	J_L_.resize(left_arm_chain_.getNrOfJoints());
	KDL::SetToZero(q_L_);
	pos_L_.Zero();
	pos_d_L_ << -0.348, 0.0 ,0.0;
	quat_d_L_.w() = 1.0;
	quat_d_L_.vec() << 0.0, 0.0, 0.0;
	quat_d_L_ =  quat_d_L_ * Q_zplus90_;
	e1_L_.Zero();
	e2_L_.Zero();
	orient_L_.Zero();
	e_L_.resize(6);
	Jac_L_.resize(6, left_arm_chain_.getNrOfJoints());
	Jac_pinv_L_.resize(left_arm_chain_.getNrOfJoints(), 6);
	k_L_ = Eigen::Matrix<double, 6, 6>::Identity() * 5.0;
	qdot_L_.resize(left_arm_chain_.getNrOfJoints());
	q_eig_L_.resize(left_arm_chain_.getNrOfJoints());
	q_eig_L_ = Eigen::VectorXd::Zero(left_arm_chain_.getNrOfJoints());
	defl_L_.resize(left_arm_chain_.getNrOfJoints());
	defl_L_ = Eigen::VectorXd::Zero(left_arm_chain_.getNrOfJoints());
	G_comp_L_.resize(left_arm_chain_.getNrOfJoints());
	KDL::SetToZero(G_comp_L_);
	qpreset_L_ = STIFFNESS_ARMS;
	q0_L_.resize(left_arm_chain_.getNrOfJoints());



	//Topic you want to publish
    pub_right_q_ = n_.advertise<geometry_msgs::Pose>("frank_q_des_right", 10);
    pub_left_q_ = n_.advertise<geometry_msgs::Pose>("frank_q_des_left", 10);

    //Topic you want to subscribe
    sub_right_des_ = n_.subscribe("/right_arm/command1", 10, &inv_kin_gravity_comp::callback_right_des, this);
    sub_left_des_ = n_.subscribe("/left_arm/command1", 10, &inv_kin_gravity_comp::callback_left_des, this);

    if(EMG)
    {
	    sub_right_stiffness_ = n_.subscribe("/right_arm/teleoperation_controller_mt_effort/stiffness_scale", 10, &inv_kin_gravity_comp::callback_right_stiffness, this);
	    sub_left_stiffness_ = n_.subscribe("/left_arm/teleoperation_controller_mt_effort/stiffness_scale", 10, &inv_kin_gravity_comp::callback_left_stiffness, this);
	    pub_right_stiffness_ = n_.advertise<std_msgs::Float64>("right_arm_stiffness", 10);
	    pub_left_stiffness_ = n_.advertise<std_msgs::Float64>("left_arm_stiffness", 10);
    }


    // --- rate limiter init --- 
    max_rate = 0.01;

	old_pos_L_.resize(left_arm_chain_.getNrOfJoints());
	old_pos_R_.resize(left_arm_chain_.getNrOfJoints());

    old_pos_L_ << 0.0, 0.0, 0.0, 0.0, 0.0;
    old_pos_R_ << 0.0, 0.0, 0.0, 0.0, 0.0;

    max_dur_cmd_des_ = ros::Duration(1);
	time_cmd_left_des_ = ros::Time::now();
	time_cmd_right_des_ = ros::Time::now();


}

inv_kin_gravity_comp::~inv_kin_gravity_comp()
{

}


void inv_kin_gravity_comp::callback_right_des(const geometry_msgs::Pose::ConstPtr& msg)
{
	pos_d_R_(0) = msg->position.x;
	pos_d_R_(1) = msg->position.y;
	pos_d_R_(2) = msg->position.z;

	pos_d_R_ = Rot_R_ * pos_d_R_;

	quat_d_R_.w() = msg->orientation.w;
	quat_d_R_.x() = msg->orientation.x;
	quat_d_R_.y() = msg->orientation.y;
	quat_d_R_.z() = msg->orientation.z;
	
	quat_d_R_ =  Q_Rot_R_* (quat_d_R_ * Q_zmin90_) * Q_Rot_R_.inverse();

	time_cmd_right_des_ = ros::Time::now();
	
}

void inv_kin_gravity_comp::callback_left_des(const geometry_msgs::Pose::ConstPtr& msg)
{
	pos_d_L_.x() = msg->position.x;
	pos_d_L_.y() = msg->position.y;
	pos_d_L_.z() = msg->position.z;

	pos_d_L_ = Rot_L_ * pos_d_L_;


	quat_d_L_.w() = msg->orientation.w;
	quat_d_L_.x() = msg->orientation.x;
	quat_d_L_.y() = msg->orientation.y;
	quat_d_L_.z() = msg->orientation.z;

	quat_d_L_ =  Q_Rot_L_* (quat_d_L_ * Q_zplus90_) * Q_Rot_L_.inverse();

	time_cmd_left_des_ = ros::Time::now();
	
}


void inv_kin_gravity_comp::callback_right_stiffness(const std_msgs::Float64::ConstPtr& msg)
{
	// la STIFFNESS arriva tra [0,1] (per sicurezza la controllo), la moltiplico per 0.3 per riportarla tra [0,0.3]
	if((msg->data > 0.0) && (msg->data < 1.0))
	{
		qpreset_R_ = msg->data * STIFFNESS_ARMS;
	}
	else if(msg->data > 1.0) 
		{
			qpreset_R_ = STIFFNESS_ARMS;
		}
		else if(msg->data < 0.0) qpreset_R_ = 0.0;

	std_msgs::Float64 qpreset_R_float;
	qpreset_R_float.data = qpreset_R_;
	pub_right_stiffness_.publish(qpreset_R_float);
}

void inv_kin_gravity_comp::callback_left_stiffness(const std_msgs::Float64::ConstPtr& msg)
{
	// la STIFFNESS arriva tra [0,1] (per sicurezza la controllo), la moltiplico per 0.3 per riportarla tra [0,0.3]
	if((msg->data > 0.0) && (msg->data < 1.0))
	{
		qpreset_L_ = msg->data * STIFFNESS_ARMS;
	}
	else if(msg->data > 1.0) 
		{
			qpreset_L_ = STIFFNESS_ARMS;
		}
		else if(msg->data < 0.0) qpreset_L_ = 0.0;

	std_msgs::Float64 qpreset_L_float;
	qpreset_L_float.data = qpreset_L_;
	pub_left_stiffness_.publish(qpreset_L_float);

	    // std::cout << " preset" << qpreset_R_ << std::endl;

}

void inv_kin_gravity_comp::run_R()
{	
	
	// // Compute the forward kinematics and Jacobian (at this location).
	jnt_to_pose_solver_R_->JntToCart(q_R_, x_R_);
	jnt_to_jac_solver_R_->JntToJac(q_R_, J_R_);




	//converto posizione, matrice di rot e jac da kdl in Eigen
	for(int i = 0; i < 3 ; i++)
	{
		pos_R_(i) = x_R_.p(i);
	}

	// std::cout<<"position: "<< pos_R_(0)<<" "<< pos_R_(1)<<" "<<pos_R_(2)<< std::endl;

	for(int i = 0; i < 3 ; i++)
	{
		for(int j = 0; j < 3 ; j++)
		{
			orient_R_(i, j) = x_R_.M(i, j);

		}
		
	}

	for(int i = 0; i < 6 ; i++)
	{
		for(int j = 0; j < right_arm_chain_.getNrOfJoints() ; j++)
		{
			Jac_R_(i, j) = J_R_(i, j);

		}
		
	}

	//da matrice di rot a quat
	quat_R_ = orient_R_;
	quat_R_.normalize();


	
	quat_d_R_vec(0) = quat_d_R_.x();
	quat_d_R_vec(1) = quat_d_R_.y();
	quat_d_R_vec(2) = quat_d_R_.z();

	
	skew_symmetric(quat_d_R_vec, skew_R_);
	e1_R_ = pos_d_R_ - pos_R_;
	e2_R_ = (quat_R_.w() * quat_d_R_.vec()) - (quat_d_R_.w() * quat_R_.vec()) - (skew_R_ * quat_R_.vec()); 

	e_R_ << e1_R_, e2_R_;
	
	if(e_R_.norm() > 0.03)
	// if(0)
	{
		pseudo_inverse(Jac_R_ , Jac_pinv_R_,true);
		
		qdot_R_ = Jac_pinv_R_ * (k_R_ * e_R_);

		q_eig_R_ += qdot_R_ * dt;



		if (q_eig_R_(0) > PI || q_eig_R_(0) < -PI)
		{
			q_eig_R_(0) = sgn(q_eig_R_(0)) * PI;
		}
		

		if (q_eig_R_(1) > PI / 18)
		{
			q_eig_R_(1) = PI / 18;
		}
		else
		{
			if (q_eig_R_(1) < -PI )
			{
				q_eig_R_(1) = -PI;
			}	
		}
		
		if (q_eig_R_(2) > PI || q_eig_R_(2) < -PI)
		{
			q_eig_R_(2) = sgn(q_eig_R_(2)) * PI;
		}

		if (q_eig_R_(3) > PI/2 || q_eig_R_(3) < -PI/2)
		{
			q_eig_R_(3) = sgn(q_eig_R_(3)) * PI/2;
		}

		if (q_eig_R_(4) > PI || q_eig_R_(4) < -PI)
		{
			q_eig_R_(4) = sgn(q_eig_R_(4)) * PI;
		}

		for(int i = 0; i < right_arm_chain_.getNrOfJoints() ; i++)
		{
			q_R_(i) = q_eig_R_(i);
		}
	}

	id_solver_R_->JntToGravity(q_R_, G_comp_R_);

	for(int i = 0; i < right_arm_chain_.getNrOfJoints(); i++)
	{
		if(fabs(G_comp_R_(i)) < 0.0000001)
		{
			defl_R_(i) = 0.0; 
		}
		else
		{				
			defl_R_(i) = -atan((k_motor_ + k_motor_ * pow(tan(a_motor_ * qpreset_R_),2) - sqrt(pow(-G_comp_R_(i),2) * pow(tan(a_motor_ * qpreset_R_),2) + 2 * pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_R_),2) + pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_R_),4) + pow(k_motor_,2))) / (-G_comp_R_(i) * pow(tan(a_motor_ * qpreset_R_),2))) / a_motor_;	
		}

	}

	q_right_pub_.position.x = q_R_(0) - defl_R_(0);
	q_right_pub_.position.y = q_R_(1) - defl_R_(1);
	q_right_pub_.position.z = q_R_(2) - defl_R_(2);
	q_right_pub_.orientation.x = -(q_R_(3) - defl_R_(3)); // il meno è dovuto al fatto che il giunto reale ha verso opposto a quello di DH
	q_right_pub_.orientation.y = q_R_(4) - defl_R_(4);


	if(ros::Time::now() > time_cmd_right_des_) //per evitare differenze negativi tra tempi
		{
			if((ros::Time::now() - time_cmd_right_des_) > max_dur_cmd_des_ )
			{
				q_right_pub_.position.x = 0;
				q_right_pub_.position.y = 0;
				q_right_pub_.position.z = 0;
				q_right_pub_.orientation.x = 0;
				q_right_pub_.orientation.y = 0;

				q_eig_R_ = Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints());

				max_rate = 0.001;
			}
			else{
				max_rate = 0.01;

			}
		}

	// !!!!!!!!!!!!!!!!!! RATE LIMITER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  //
	if(fabs((q_right_pub_.position.x -old_pos_R_(0))) > max_rate)
		q_right_pub_.position.x = old_pos_R_(0) +sgn(q_right_pub_.position.x -old_pos_R_(0))*max_rate;
	
	if(fabs((q_right_pub_.position.y -old_pos_R_(1))) > max_rate)
		q_right_pub_.position.y = old_pos_R_(1) +sgn(q_right_pub_.position.y -old_pos_R_(1))*max_rate;
	
	if(fabs((q_right_pub_.position.z -old_pos_R_(2))) > max_rate)
		q_right_pub_.position.z = old_pos_R_(2) +sgn(q_right_pub_.position.z -old_pos_R_(2))*max_rate;

	if(fabs((q_right_pub_.orientation.x -old_pos_R_(3))) > max_rate)
		q_right_pub_.orientation.x = old_pos_R_(3) +sgn(q_right_pub_.orientation.x -old_pos_R_(3))*max_rate;

	if(fabs((q_right_pub_.orientation.y -old_pos_R_(4))) > max_rate)
		q_right_pub_.orientation.y = old_pos_R_(4) +sgn(q_right_pub_.orientation.y -old_pos_R_(4))*max_rate;

	old_pos_R_(0) = q_right_pub_.position.x;
	old_pos_R_(1) = q_right_pub_.position.y;
	old_pos_R_(2) = q_right_pub_.position.z;
	old_pos_R_(3) = q_right_pub_.orientation.x;
	old_pos_R_(4) = q_right_pub_.orientation.y;
	// !!!!!!!!!!!!!!!!!! RATE LIMITER  FIN !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  //

}


void inv_kin_gravity_comp::run_L()
{	
	// Compute the forward kinematics and Jacobian (at this location).
	jnt_to_pose_solver_L_->JntToCart(q_L_, x_L_);
	jnt_to_jac_solver_L_->JntToJac(q_L_, J_L_);

	//converto posizione, matrice di rot e jac da kdl in Eigen
	for(int i = 0; i < 3 ; i++)
	{
		pos_L_(i) = x_L_.p(i);
	}

	for(int i = 0; i < 3 ; i++)
	{
		for(int j = 0; j < 3 ; j++)
		{
			orient_L_(i, j) = x_L_.M(i, j);

		}
		
	}

	for(int i = 0; i < 6 ; i++)
	{
		for(int j = 0; j < left_arm_chain_.getNrOfJoints() ; j++)
		{
			Jac_L_(i, j) = J_L_(i, j);

		}
		
	}

	//da matrice di rot a quat
	quat_L_ = orient_L_;
	quat_L_.normalize();


	
	quat_d_L_vec(0) = quat_d_L_.x();
	quat_d_L_vec(1) = quat_d_L_.y();
	quat_d_L_vec(2) = quat_d_L_.z();

	
	skew_symmetric(quat_d_L_vec, skew_L_);
	e1_L_ = pos_d_L_ - pos_L_;
	// e2_L_ = (quat_L_.w() * quat_d_L_.vec()) - (quat_d_L_.w() * quat_L_.vec()) - (quat_d_L_.vec().cross(quat_L_.vec())); 
	e2_L_ = (quat_L_.w() * quat_d_L_.vec()) - (quat_d_L_.w() * quat_L_.vec()) - (skew_L_ * quat_L_.vec()); 

	e_L_ << e1_L_, e2_L_;
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////
	
if(e_L_.norm() > 0.03)
	{
		pseudo_inverse(Jac_L_ , Jac_pinv_L_,true);
		qdot_L_ = Jac_pinv_L_ * (k_L_ * e_L_);
		q_eig_L_ += qdot_L_ * dt;

		if (q_eig_L_(0) > PI || q_eig_L_(0) < -PI)
		{
			q_eig_L_(0) = sgn(q_eig_L_(0)) * PI;
		}


		if (q_eig_L_(1) < -PI / 18) 
		{
			q_eig_L_(1) = -PI / 18;
		}
		else
		{
			if (q_eig_L_(1) > PI )
			{
				q_eig_L_(1) = PI;
			}	
		}

		if (q_eig_L_(2) > PI || q_eig_L_(2) < -PI)
		{
			q_eig_L_(2) = sgn(q_eig_L_(2)) * PI;
		}

		if (q_eig_L_(3) > PI/2 || q_eig_L_(3) < -PI/2)
		{
			q_eig_L_(3) = sgn(q_eig_L_(3)) * PI/2;
		}

		if (q_eig_L_(4) > PI || q_eig_L_(4) < -PI)
		{
			q_eig_L_(4) = sgn(q_eig_L_(4)) * PI;
		}

			for(int i = 0; i < left_arm_chain_.getNrOfJoints() ; i++)
		{
			q_L_(i) = q_eig_L_(i);
		}

	}

	id_solver_L_->JntToGravity(q_L_, G_comp_L_);

	for(int i = 0; i < left_arm_chain_.getNrOfJoints(); i++)
	{
		if(fabs(G_comp_L_(i)) < 0.0000001)
		{
			defl_L_(i) = 0.0; 
		}
		else
		{				
			defl_L_(i) = -atan((k_motor_ + k_motor_ * pow(tan(a_motor_ * qpreset_L_),2) - sqrt(pow(-G_comp_L_(i),2) * pow(tan(a_motor_ * qpreset_L_),2) + 2 * pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_L_),2) + pow(k_motor_,2) * pow(tan(a_motor_ * qpreset_L_),4) + pow(k_motor_,2))) / (-G_comp_L_(i) * pow(tan(a_motor_ * qpreset_L_),2))) / a_motor_;	
		}

	}

	q_left_pub_.position.x = q_L_(0) - defl_L_(0);
	q_left_pub_.position.y = q_L_(1) - defl_L_(1);
	q_left_pub_.position.z = q_L_(2) - defl_L_(2);
	q_left_pub_.orientation.x = -(q_L_(3) - defl_L_(3)); // il meno è dovuto al fatto che il giunto reale ha verso opposto a quello di DH
	q_left_pub_.orientation.y = q_L_(4) - defl_L_(4);

	if(ros::Time::now() > time_cmd_left_des_) //per evitare differenze negativi tra tempi
		{
			if((ros::Time::now() - time_cmd_left_des_) > max_dur_cmd_des_ )
			{
				q_left_pub_.position.x = 0;
				q_left_pub_.position.y = 0;
				q_left_pub_.position.z = 0;
				q_left_pub_.orientation.x = 0;
				q_left_pub_.orientation.y = 0;

				q_eig_L_ = Eigen::VectorXd::Zero(left_arm_chain_.getNrOfJoints());

				max_rate = 0.001;
			}
			else{
				max_rate = 0.01;

			}
		}

	// !!!!!!!!!!!!!!!!!! RATE LIMITER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  //
	if(fabs((q_left_pub_.position.x -old_pos_L_(0)))>= max_rate)
		q_left_pub_.position.x = old_pos_L_(0) +sgn(q_left_pub_.position.x -old_pos_L_(0))*max_rate;
	
	if(fabs((q_left_pub_.position.y -old_pos_L_(1)))>= max_rate)
		q_left_pub_.position.y = old_pos_L_(1) +sgn(q_left_pub_.position.y -old_pos_L_(1))*max_rate;
	
	if(fabs((q_left_pub_.position.z -old_pos_L_(2))) >= max_rate)
		q_left_pub_.position.z = old_pos_L_(2) +sgn(q_left_pub_.position.z -old_pos_L_(2))*max_rate;

	if(fabs((q_left_pub_.orientation.x -old_pos_L_(3))) >= max_rate)
		q_left_pub_.orientation.x = old_pos_L_(3) +sgn(q_left_pub_.orientation.x -old_pos_L_(3))*max_rate;

	if(fabs((q_left_pub_.orientation.y -old_pos_L_(4))) >= max_rate)
		q_left_pub_.orientation.y = old_pos_L_(4) +sgn(q_left_pub_.orientation.y -old_pos_L_(4))*max_rate;

	old_pos_L_(0) = q_left_pub_.position.x;
	old_pos_L_(1) = q_left_pub_.position.y;
	old_pos_L_(2) = q_left_pub_.position.z;
	old_pos_L_(3) = q_left_pub_.orientation.x;
	old_pos_L_(4) = q_left_pub_.orientation.y;
	// !!!!!!!!!!!!!!!!!! RATE LIMITER  FIN !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  //
} 

void inv_kin_gravity_comp::run()
{
	run_R();
	run_L();

	pub_right_q_.publish(q_right_pub_);
	pub_left_q_.publish(q_left_pub_);

}

int inv_kin_gravity_comp::sgn(double d)
{
    return (d < 0) ? -1 : d>0; 
}