// Define
#define ERR_TIMEOUT		5
#define EQ_PRESET		true
#define PERC			true

// ROS Headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

// ROS custom msg
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
#include <qb_interface/quaternion.h>
#include <qb_interface/quaternionArray.h>
#include <qb_interface/temperature.h>
#include <qb_interface/temperatureArray.h>

// General Headers
#include <vector>
#include <map>
#include <iostream>
#include <eigen3/Eigen/Eigen>


// Custom Headers
#include "qb_class.h"
#include "qbImuBoard.h"


class qb_class_imu : public qb_class {

	public:
		// Costructor
		qb_class_imu();

		// Destructor
		~qb_class_imu();

		// SpinOnce function
		void spinOnce();

		// Spin function
		void spin();

	protected:

		// Step Time, 1 / step_time = communication frequency
		double step_time_imu_;

		int hand_step_div_;
	private:

		// Functions


		// Read measurements of all IMUs
		bool readIMU();
		
		// Variables
		std::vector<qbImuBoard*> imuboard_chain_;

		// Ros node handle
		ros::NodeHandle* node_;


	
 		// Publisher variables
		ros::Publisher imuboard_pub_acc_;
		ros::Publisher imuboard_pub_gyro_;
		ros::Publisher imuboard_pub_mag_;
		ros::Publisher imuboard_pub_quat_;
		ros::Publisher imuboard_pub_temp_;


		Eigen::MatrixXd Acc_, Acc_old_;


};