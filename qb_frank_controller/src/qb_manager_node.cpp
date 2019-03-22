#include <ros/ros.h>
#include <ros/rate.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <qb_frank_controller/qbmove_communications.h>

#define STIFFNESS_NECK 0.3
#define STIFFNESS_ARMS 0.3
#define HAND_CLOSURE 18000
#define ACTIVATE true



struct imu_properties{
    float*   imu_values_;
    int      n_imu_;
    uint8_t* ids_;
    uint8_t* imu_table_;
    uint8_t* mag_cal_;
};

std::map<int, std::vector<short int>> cube_pos;
std::map<int, short int> hand_pos;

std::map<int,int> arm_pos_id_map;
int head_id;
int hand_id;

geometry_msgs::Pose q_des_;
double qpreset_;

double tick2rad(short int meas)
{
	return (double(meas)) * (2.0 * M_PI) / 32768.0;
}

short int compute_cube_motor_position(double eq, double stiff, short int& th1, short int& th2)
{
	th1 = (short int)((eq + stiff) * 32768.0 / (2* M_PI));
	th2 = (short int)((eq - stiff) * 32768.0 / (2* M_PI));
}

short int compute_hand_motor_position(double eq, short int& th1)
{
	th1 = (short int)(eq*HAND_CLOSURE);
}

void arm_chain_values_callback(const geometry_msgs::Pose& msg)
{
    q_des_.position.x = msg.position.x;
	q_des_.position.y = msg.position.y;
	q_des_.position.z = msg.position.z;
	q_des_.orientation.x = msg.orientation.x;
	q_des_.orientation.y = msg.orientation.y;
}

void gaze_callback(const geometry_msgs::Vector3& msg)
{
	if(head_id == 21) compute_cube_motor_position(msg.z,STIFFNESS_NECK,cube_pos.at(head_id).at(0),cube_pos.at(head_id).at(1));
 	if(head_id == 22) compute_cube_motor_position(msg.y - 0.3,STIFFNESS_NECK,cube_pos.at(head_id).at(0),cube_pos.at(head_id).at(1));
}

void hand_callback(const std_msgs::Float64& msg)
{

	if((msg.data > 0.0) && (msg.data <= 1.0))
	{
		compute_hand_motor_position(msg.data,hand_pos.at(hand_id));
	}
	else if(msg.data <= 0.0)
	{
		compute_hand_motor_position(0.0,hand_pos.at(hand_id));
	}
	else if(msg.data > 1) compute_hand_motor_position(1.0,hand_pos.at(hand_id));
}

void callback_stiffness(const std_msgs::Float64& msg)
{
	if((msg.data > 0.0) && (msg.data < STIFFNESS_ARMS))
	{
		qpreset_ = msg.data;
	}
	else if(msg.data <= 0.0)
	{
		qpreset_ = 0.0;
	}
	else if(msg.data >= STIFFNESS_ARMS) qpreset_ = STIFFNESS_ARMS;

}

void activate(comm_settings* cs, int id)
{
	commActivate(cs, id, 1);
	usleep(1000);
}

void deactivate(comm_settings* cs, int id)
{
	commActivate(cs, id, 0);
	usleep(1000);
}

//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "qb_manager");
  	ros::NodeHandle n_;

  	double rateHZ = 100;
	short int inputs[2];
	short int measurements[3];
	qpreset_ = STIFFNESS_ARMS;

	std::vector<int> cube_id;
	std::vector<int> imu_id;// = {7,17};
	
	std::string chain;
	std::string port;
	std::string arm_chain_topic;
	std::string hand_topic;
	std::string arm_stiff_topic;

	std::string measure_topic, measure1_topic, measure2_topic;
	std::string imu_topic;

	ros::NodeHandle pn("~");
	pn.getParam("chain",chain);
	pn.getParam("port",port);

	if(chain=="left")
	{
	    cube_id = std::vector<int>({1,2,3,4,5,22});
	    arm_pos_id_map[0] = 1;
	    arm_pos_id_map[1] = 2;
	    arm_pos_id_map[2] = 3;
	    arm_pos_id_map[3] = 4;
	    arm_pos_id_map[4] = 5;
	    hand_id = 6;
	    head_id = 22;

	    arm_chain_topic = "/frank_q_des_left";
	    hand_topic = "/left_hand/hand_remap/hand_position";
	    measure_topic = "measure_L";
	    measure1_topic = "measure_L_1";
	    measure2_topic = "measure_L_2";
	    arm_stiff_topic = "/left_arm_stiffness";
	}
	else
	{
	    cube_id = std::vector<int>({11,12,13,14,15,21});
	    arm_pos_id_map[0] = 11;
	    arm_pos_id_map[1] = 12;
	    arm_pos_id_map[2] = 13;
	    arm_pos_id_map[3] = 14;
	    arm_pos_id_map[4] = 15;
	    hand_id = 16;
	    head_id = 21;

	    arm_chain_topic = "/frank_q_des_right";
	    hand_topic = "/right_hand/hand_remap/hand_position";
	    measure_topic = "measure_R";
	    measure1_topic = "measure_R_1";
	    measure2_topic = "measure_R_2";
	    arm_stiff_topic = "/right_arm_stiffness";
	}
	
	ros::Subscriber sub_arml = n_.subscribe(arm_chain_topic.c_str(),1,&arm_chain_values_callback);
	ros::Subscriber sub_gaze = n_.subscribe("/gaze",1,&gaze_callback);
	ros::Subscriber sub_hand = n_.subscribe(hand_topic.c_str(),1,&hand_callback);
	ros::Subscriber sub_stiffness = n_.subscribe(arm_stiff_topic.c_str(), 1, &callback_stiffness);


	ros::Publisher  pub_measure = n_.advertise<geometry_msgs::Pose>(measure_topic, 10);
	ros::Publisher  pub_measure1 = n_.advertise<geometry_msgs::Pose>(measure1_topic, 10);
	ros::Publisher  pub_measure2 = n_.advertise<geometry_msgs::Pose>(measure2_topic, 10);
	geometry_msgs::Pose msg, msg_1, msg_2;
	
	comm_settings comm_settings_t;

	openRS485(&comm_settings_t, port.c_str());
	usleep(10000);

	std::cout<<std::endl;
	std::cout<<"Opened: "<<port<<std::endl;

	std::cout<<std::endl;
	std::cout<<"QB Manager"<<std::endl;
	std::cout<<" - Cubes ("<<cube_id.size()<<") = ";

	for(auto id:cube_id)
	{
		cube_pos[id] = {0,0};
		activate(&comm_settings_t,id);
		std::cout<<id<<", ";
	}

	std::cout<<std::endl<<" - Hand = ";

	hand_pos[hand_id]=0;
	activate(&comm_settings_t,hand_id);
	std::cout<<hand_id<<std::endl;
	
  	ros::Rate r(rateHZ);

	while(ros::ok())
	{

		for(auto id:cube_id)
		{
			if(id == 1  || id == 11) compute_cube_motor_position(q_des_.position.x,qpreset_,cube_pos.at(id).at(0),cube_pos.at(id).at(1));
			if(id == 2  || id == 12) compute_cube_motor_position(q_des_.position.y,qpreset_,cube_pos.at(id).at(0),cube_pos.at(id).at(1)); 
			if(id == 3  || id == 13) compute_cube_motor_position(q_des_.position.z,qpreset_,cube_pos.at(id).at(0),cube_pos.at(id).at(1)); 
			if(id == 4  || id == 14) compute_cube_motor_position(q_des_.orientation.x,qpreset_,cube_pos.at(id).at(0),cube_pos.at(id).at(1)); 
			if(id == 5  || id == 15) compute_cube_motor_position(q_des_.orientation.y,qpreset_,cube_pos.at(id).at(0),cube_pos.at(id).at(1)); 
		}
	    
		for(auto id:cube_id)
		{
			inputs[0] = cube_pos[id].at(0);
			inputs[1] = cube_pos[id].at(1);

			commSetInputs(&comm_settings_t, id, inputs);
			usleep(100);
		}

		inputs[0] = hand_pos[hand_id];
		inputs[1] = 0;

		commSetInputs(&comm_settings_t, hand_id, inputs);
		usleep(100);

		for(auto id:cube_id)
		{
			if(commGetMeasurements(&comm_settings_t, id, measurements)>0)
			{
				if(id == 1  || id == 11)
				{
					msg.position.x = tick2rad(measurements[2]);
					msg_1.position.x = tick2rad(measurements[0]);
					msg_2.position.x = tick2rad(measurements[1]);
				}
				if(id == 2  || id == 12)
				{
					msg.position.y = tick2rad(measurements[2]);
					msg_1.position.y = tick2rad(measurements[0]);
					msg_2.position.y = tick2rad(measurements[1]);
				}
				if(id == 3  || id == 13)
				{
					msg.position.z = tick2rad(measurements[2]);
					msg_1.position.z = tick2rad(measurements[0]);
					msg_2.position.z = tick2rad(measurements[1]);
				}
	 
				if(id == 4  || id == 14)
				{
					msg.orientation.x = tick2rad(measurements[2]);
					msg_1.orientation.x = tick2rad(measurements[0]);
					msg_2.orientation.x = tick2rad(measurements[1]);
				}
				if(id == 5  || id == 15)
				{
					msg.orientation.y = tick2rad(measurements[2]);
					msg_1.orientation.y = tick2rad(measurements[0]);
					msg_2.orientation.y = tick2rad(measurements[1]);
				}

				if(id == 21 || id == 22)
				{
					msg.orientation.z = tick2rad(measurements[2]);
					msg_1.orientation.z = tick2rad(measurements[0]);
					msg_2.orientation.z = tick2rad(measurements[1]);
				}

				usleep(100);
			}
			
		}
		pub_measure.publish(msg);
		pub_measure1.publish(msg_1);
		pub_measure2.publish(msg_2);

		
		commGetMeasurements(&comm_settings_t, hand_id, measurements);
		usleep(100);

		ros::spinOnce();
		r.sleep();

	}

	for(auto id:cube_id)
		deactivate(&comm_settings_t,id);

	deactivate(&comm_settings_t,hand_id);

	closeRS485(&comm_settings_t);
	usleep(1000);


}