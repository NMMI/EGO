#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "inv_kin.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 200;

  ros::init(argc, argv, "inv_kin_node");
  ros::NodeHandle nh;

  inv_kin Obj;
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