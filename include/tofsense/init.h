#ifndef TOFSENSEINIT_H
#define TOFSENSEINIT_H

//#include <nlink_parser/TofsenseCascade.h>
//#include <nlink_parser/TofsenseFrame0.h>
#include <nlink_parser_interfaces/msg/tofsense_cascade.hpp>
#include <nlink_parser_interfaces/msg/tofsense_frame0.hpp>

// #include <ros/ros.h>
#include<rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <map>
#include <unordered_map>

#include "utils/protocol_extracter/nprotocol_extracter.h"

namespace tofsense
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial,
                  rclcpp::Node *nd);

  private:
    void InitFrame0(NProtocolExtracter *protocol_extraction);
    //std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::TofsenseCascade>::SharedPtr  publisher_TofsenseCascade_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::TofsenseFrame0>::SharedPtr  publisher_TofsenseFrame0_;

    std::map<int, nlink_parser_interfaces::msg::TofsenseFrame0> frame0_map_;

    serial::Serial *serial_;

    const int frequency_ = 10;
    bool is_inquire_mode_ = true;

    //ros::NodeHandle nh_;
    rclcpp::Node *node;
    
    //ros::Timer timer_scan_;
    //ros::Timer timer_read_;
    rclcpp::TimerBase::SharedPtr timer_scan_;
    rclcpp::TimerBase::SharedPtr timer_read_;
    uint8_t node_index_ = 0;
  };

} // namespace tofsense
#endif // TOFSENSEINIT_H
