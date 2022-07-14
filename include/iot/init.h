#ifndef IOT_INIT_H
#define IOT_INIT_H

#include "utils/protocol_extracter/nprotocol_extracter.h"
#include <nlink_parser_interfaces/msg/iot_frame0.hpp>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <unordered_map>

namespace iot
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction, rclcpp::Node *nd);

  private:
    void InitFrame0(NProtocolExtracter *protocol_extraction);
    //std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::IotFrame0>::SharedPtr  publisher_IotFrame0_;
    //ros::NodeHandle nh_;

    rclcpp::Node *node;
    
  };

} // namespace iot
#endif // IOT_INIT_H
