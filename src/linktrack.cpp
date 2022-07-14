#include "linktrack/linktrack_ros2.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkTrack>());
  rclcpp::shutdown();

  return 0;
}
