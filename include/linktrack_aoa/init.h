#ifndef LINKTRACKAOAINIT_H
#define LINKTRACKAOAINIT_H

//#include <nlink_parser/LinktrackAoaNodeframe0.h>
//#include <nlink_parser/LinktrackNodeframe0.h>

#include "nlink_parser_interfaces/msg/linktrack_aoa_nodeframe0.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe0.hpp"


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>

#include <unordered_map>

#include "utils/protocol_extracter/nprotocol_extracter.h"

namespace linktrack_aoa
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial,
                  rclcpp::Node *nd);

  private:
    void initDataTransmission();
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void InitAoaNodeFrame0(NProtocolExtracter *protocol_extraction);
    
    //std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe0>::SharedPtr     publisher_LinktrackNodeframe0;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackAoaNodeframe0>::SharedPtr    publisher_LinktrackAoaNodeframe0_;

    //ros::NodeHandle nh_;
    //ros::Subscriber dt_sub_;
    rclcpp::Node *node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
  };
} // namespace linktrack_aoa

#endif // LINKTRACKAOAINIT_H
