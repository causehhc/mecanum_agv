#include <ros/ros.h>
#include "HectorMappingRos.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hector_slam");
  HectorMappingRos sm;
  ros::spin();
  return(0);
}
