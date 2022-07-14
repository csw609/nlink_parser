#include "tofsensem/tofsensem_ros2.h"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TOFSenseM>());
  rclcpp::shutdown();

  return 0;
}
