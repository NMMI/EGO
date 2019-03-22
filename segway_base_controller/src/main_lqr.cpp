#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "lqr.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 400;

  ros::init(argc, argv, "lqr_node");
  ros::NodeHandle nh;

  lqr Obj;
  Obj.dt = 1 / rateHZ;
  ros::Rate r(rateHZ);


  while(ros::ok())
  {
    Obj.run();

    ros::spinOnce();
    r.sleep();
        
  }// end while()
  Obj.stop_motor();
  return 0;
}