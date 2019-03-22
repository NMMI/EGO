#include "qb_class_imu.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "qb_interface_node");

  qb_class_imu qb_int_imu;

  cout << "[INFO] Start to spin qb_interface_node" << endl;

  // qb_int_imu.spinOnce();
  qb_int_imu.spin();

  return 0;
}