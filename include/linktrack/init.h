#ifndef LINKTRACKINIT_H
#define LINKTRACKINIT_H

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>

#include <unordered_map>

//#include "nlink_unpack/nlink_utils.h"
#include "utils/nlink_unpack/nlink_utils.h"
//#include "protocol_extracter/nprotocol_extracter.h"
#include "utils/protocol_extracter/nprotocol_extracter.h"

#include "nlink_parser_interfaces/msg/linktrack_anchorframe0.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe0.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe1.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe2.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe3.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe4.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe5.hpp"
#include "nlink_parser_interfaces/msg/linktrack_nodeframe6.hpp"
#include "nlink_parser_interfaces/msg/linktrack_tagframe0.hpp"

class NProtocolExtracter;
namespace linktrack
{
  class Init
  {
  public:
    explicit Init(NProtocolExtracter *protocol_extraction,
                  serial::Serial *serial,
                  rclcpp::Node *nd);

  private:
    void initDataTransmission();
    void initAnchorFrame0(NProtocolExtracter *protocol_extraction);
    void initTagFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame0(NProtocolExtracter *protocol_extraction);
    void initNodeFrame1(NProtocolExtracter *protocol_extraction);
    void initNodeFrame2(NProtocolExtracter *protocol_extraction);
    void initNodeFrame3(NProtocolExtracter *protocol_extraction);
    void initNodeFrame4(NProtocolExtracter *protocol_extraction);
    void initNodeFrame5(NProtocolExtracter *protocol_extraction);
    void initNodeFrame6(NProtocolExtracter *protocol_extraction);

    //std::unordered_map<NProtocolBase *, ros::Publisher> publishers_;

    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackAnchorframe0>::SharedPtr  publisher_Anchorframe0_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackTagframe0>::SharedPtr     publisher_LinktrackTagframe0_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe0>::SharedPtr    publisher_LinktrackNodeframe0_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe1>::SharedPtr    publisher_LinktrackNodeframe1_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe2>::SharedPtr    publisher_LinktrackNodeframe2_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe3>::SharedPtr    publisher_LinktrackNodeframe3_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe4>::SharedPtr    publisher_LinktrackNodeframe4_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe5>::SharedPtr    publisher_LinktrackNodeframe5_;
    rclcpp::Publisher<nlink_parser_interfaces::msg::LinktrackNodeframe6>::SharedPtr    publisher_LinktrackNodeframe6_;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_;
    //ros::NodeHandle nh_;
    //ros::Subscriber dt_sub_;
    rclcpp::Node *node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_sub_;
  };
} // namespace linktrack

#endif // LINKTRACKINIT_H
