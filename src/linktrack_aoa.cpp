#include "linktrack_aoa/linktrack_aoa_ros2.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinkTrackAOA>());
  rclcpp::shutdown();

  return 0;
}
