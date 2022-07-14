#include "nutils.h"
#include <iostream>
//#include <ros/ros.h>

void TopicAdvertisedTip(const char *topic)
{
  // ROS_INFO("%s has been advertised,use 'rostopic "
  //          "echo /%s' to view the data",
  //          topic, topic);
  std::cout << topic << "has been advertised,use rostopic echo /" << topic << " to view the data" << "\n";
}
