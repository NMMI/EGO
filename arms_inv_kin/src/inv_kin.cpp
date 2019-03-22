#include <inv_kin.h>

inv_kin::inv_kin()
{
	// double d1 = 0.084;
	double d1 = 0.0;

	double d2 = 0.090;
	double d3 = 0.078;
	double d4 = 0.090;
	double d5 = 0.090;

	Q_zmin90_.w() = 0.7071;
	Q_zmin90_.vec() << 0.0, 0.0, -0.7071;

	Q_zplus90_.w() = 0.7071;
	Q_zplus90_.vec() << 0.0, 0.0, 0.7071;



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
	jnt_to_jac_solver_R_.reset(new KDL::ChainJntToJacSolver(right_arm_chain_));

	jnt_to_pose_solver_L_.reset(new KDL::ChainFkSolverPos_recursive(left_arm_chain_));
	jnt_to_jac_solver_L_.reset(new KDL::ChainJntToJacSolver(left_arm_chain_));


	// Resize (pre-allocate) the variables in non-realtime.                                                                                                                         
	q_R_.resize(right_arm_chain_.getNrOfJoints());
	J_R_.resize(right_arm_chain_.getNrOfJoints());
	KDL::SetToZero(q_R_);
	pos_R_.Zero();
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
	W_ = Eigen::Matrix<double, 5, 5>::Identity();
	W_(4,4) = 0.01;
	skew_R_.Zero();


	qdot_R_.resize(right_arm_chain_.getNrOfJoints());
	q_eig_R_.resize(right_arm_chain_.getNrOfJoints());
	q_eig_R_ = Eigen::VectorXd::Zero(right_arm_chain_.getNrOfJoints());



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


	//Topic you want to publish
    pub_right_q_ = n_.advertise<geometry_msgs::Pose>("frank_q_des_right", 10);
    pub_left_q_ = n_.advertise<geometry_msgs::Pose>("frank_q_des_left", 10);

    //Topic you want to subscribe
    sub_right_des_ = n_.subscribe("/right_arm/command1", 10, &inv_kin::callback_right_des, this);
    sub_left_des_ = n_.subscribe("/left_arm/command1", 10, &inv_kin::callback_left_des, this);


}

inv_kin::~inv_kin()
{

}


void inv_kin::callback_right_des(const geometry_msgs::Pose::ConstPtr& msg)
{
	pos_d_R_(0) = msg->position.x;
	pos_d_R_(1) = msg->position.y;
	pos_d_R_(2) = msg->position.z;

	quat_d_R_.w() = msg->orientation.w;
	quat_d_R_.x() = msg->orientation.x;
	quat_d_R_.y() = msg->orientation.y;
	quat_d_R_.z() = msg->orientation.z;

	quat_d_R_ =  quat_d_R_ * Q_zmin90_;


}

void inv_kin::callback_left_des(const geometry_msgs::Pose::ConstPtr& msg)
{
	pos_d_L_.x() = msg->position.x;
	pos_d_L_.y() = msg->position.y;
	pos_d_L_.z() = msg->position.z;

	quat_d_L_.w() = msg->orientation.w;
	quat_d_L_.x() = msg->orientation.x;
	quat_d_L_.y() = msg->orientation.y;
	quat_d_L_.z() = msg->orientation.z;

	quat_d_L_ =  quat_d_L_ * Q_zplus90_;
	
}



void inv_kin::run_R()
{	
	
	// Compute the forward kinematics and Jacobian (at this location).
	jnt_to_pose_solver_R_->JntToCart(q_R_, x_R_);
	jnt_to_jac_solver_R_->JntToJac(q_R_, J_R_);

	//converto posizione, matrice di rot e jac da kdl in Eigen
	for(int i = 0; i < 3 ; i++)
	{
		pos_R_(i) = x_R_.p(i);
	}

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

	quat_R_ = orient_R_;
	quat_R_.normalize();


	
	quat_d_R_vec(0) = quat_d_R_.x();
	quat_d_R_vec(1) = quat_d_R_.y();
	quat_d_R_vec(2) = quat_d_R_.z();

	
	skew_symmetric(quat_d_R_vec, skew_R_);
	e1_R_ = pos_d_R_ - pos_R_;
	e2_R_ = (quat_R_.w() * quat_d_R_.vec()) - (quat_d_R_.w() * quat_R_.vec()) - (skew_R_ * quat_R_.vec()); 

	e_R_ << e1_R_, e2_R_;
	
	std::cout<<"norm_e: "<< e_R_.norm() << std::endl;

	
	if(e_R_.norm() > 0.03)
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

		q_right_pub_.position.x = q_R_(0) * 1.5; //NOTE: gravity comp.;
		q_right_pub_.position.y = q_R_(1);
		q_right_pub_.position.z = q_R_(2);
		q_right_pub_.orientation.x = -q_R_(3);
		q_right_pub_.orientation.y = q_R_(4);
	}
	
///////////////////////////////////////////////////////////////////////////////////////////////////////////

}


void inv_kin::run_L()
{	
	// Compute the forward kinematics and Jacobian (at this location).
	jnt_to_pose_solver_L_->JntToCart(q_L_, x_L_);
	jnt_to_jac_solver_L_->JntToJac(q_L_, J_L_);

	//converto posizione, matrice di rot e jac da kdl in Eigen
	for(int i = 0; i < 3 ; i++)
	{
		pos_L_(i) = x_L_.p(i);
	}

	// std::cout<<"position: "<< pos_R_(0)<<" "<< pos_R_(1)<<" "<<pos_R_(2)<< std::endl;

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

		q_left_pub_.position.x = q_L_(0) * 1.5; //NOTE: gravity comp.
		q_left_pub_.position.y = q_L_(1);
		q_left_pub_.position.z = q_L_(2);
		q_left_pub_.orientation.x = -q_L_(3);
		q_left_pub_.orientation.y = q_L_(4);
	}


}

void inv_kin::run()
{
	run_R();
	run_L();

	pub_right_q_.publish(q_right_pub_);
	pub_left_q_.publish(q_left_pub_);

}

int inv_kin::sgn(double d)
{
    return d<0? -1 : d>0; 
}