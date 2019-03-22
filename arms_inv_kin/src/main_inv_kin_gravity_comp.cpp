#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "inv_kin_gravity_comp.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 200;

  ros::init(argc, argv, "inv_kin_gravity_comp_node");
  ros::NodeHandle nh;

  inv_kin_gravity_comp Obj;
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