#include "tofsense/tofsense_ros2.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TOFSense>());
  rclcpp::shutdown();

  return 0;
}
