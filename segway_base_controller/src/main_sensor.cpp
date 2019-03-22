#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "Sensor.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 400;

  ros::init(argc, argv, "Sensor_node");
  ros::NodeHandle nh;

  Sensor Obj;
  Obj.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);


  while(ros::ok())
  {
    Obj.run();

    ros::spinOnce();
    r.sleep();
        
  }// end while()
return 0;
}