
#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "Joy_oculus_bridge.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{
  double rateHZ = 100;

  ros::init(argc, argv, "Joy_oculus_bridge_node");
  
  Joy_oculus_bridge Obj;
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