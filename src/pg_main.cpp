#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pg_functions.h"


 

int main(int argc, char **argv)
{

  ros::init(argc, argv, "contact_detection_node");
  ros::NodeHandle n;

  proto_functions PF;

  ros::Rate loop_rate(200);

  while(ros::ok())
  {
  	PF.protoManager();

  	ros::spinOnce();

  	loop_rate.sleep();
  }  

  return 0;
}
