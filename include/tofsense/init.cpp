
#include "init.h"

#include "utils/nlink_protocol.h"
#include "utils/nlink_unpack/nlink_tofsense_frame0.h"
#include "utils/nlink_unpack/nlink_utils.h"
#include "utils/nutils.h"

#include <chrono>
using namespace std::chrono_literals;

class NTS_ProtocolFrame0 : public NLinkProtocol
{
public:
  NTS_ProtocolFrame0();

protected:
  void UnpackFrameData(const uint8_t *data) override;
};

NTS_ProtocolFrame0::NTS_ProtocolFrame0()
    : NLinkProtocol(true, g_nts_frame0.fixed_part_size,
                    {g_nts_frame0.frame_header, g_nts_frame0.function_mark})
{
}

void NTS_ProtocolFrame0::UnpackFrameData(const uint8_t *data)
{
  g_nts_frame0.UnpackData(data, length());
}

namespace tofsense
{
  nlink_parser_interfaces::msg::TofsenseFrame0 g_msg_frame0;

#pragma pack(push, 1)
  struct
  {
    char header[2]{0x57, 0x10};
    uint8_t reserved0[2]{0xff, 0xff};
    uint8_t id{};
    uint8_t reserved1[2]{0xff, 0xff};
    uint8_t checkSum{};
  } g_command_read;
#pragma pack(pop)

  Init::Init(NProtocolExtracter *protocol_extraction, serial::Serial *serial, rclcpp::Node *nd)
      : serial_(serial)
  {
    node = nd;
    bool inquire_mode = true;
    is_inquire_mode_ = inquire_mode; // To Do : Parametrization
                                     // serial_ ? ros::param::param<bool>("~inquire_mode", true) : false;

    InitFrame0(protocol_extraction);
  }

  void Init::InitFrame0(NProtocolExtracter *protocol_extraction)
  {
    static auto protocol_frame0_ = new NTS_ProtocolFrame0;
    protocol_extraction->AddProtocol(protocol_frame0_);
    protocol_frame0_->SetHandleDataCallback(
        [=]
        {
          if (!publisher_TofsenseCascade_ && !publisher_TofsenseFrame0_)
          {
            // ros::NodeHandle nh_;
            if (is_inquire_mode_)
            {
              auto topic = "nlink_tofsense_cascade";
              publisher_TofsenseCascade_ = node->create_publisher<nlink_parser_interfaces::msg::TofsenseCascade>(topic, 50);
              // nh_.advertise<nlink_parser::TofsenseCascade>(topic, 50);
              TopicAdvertisedTip(topic);
            }
            else
            {
              auto topic = "nlink_tofsense_frame0";
              publisher_TofsenseFrame0_ = node->create_publisher<nlink_parser_interfaces::msg::TofsenseFrame0>(topic, 50);
              // nh_.advertise<nlink_parser::TofsenseFrame0>(topic, 50);
              TopicAdvertisedTip(topic);
            }
          }

          const auto &data = g_nts_frame0.result;

          g_msg_frame0.id = data.id;
          g_msg_frame0.system_time = data.system_time;
          g_msg_frame0.dis = data.dis;
          g_msg_frame0.dis_status = data.dis_status;
          g_msg_frame0.signal_strength = data.signal_strength;
          g_msg_frame0.range_precision = data.range_precision;

          if (is_inquire_mode_)
          {
            frame0_map_[data.id] = g_msg_frame0;
          }
          else
          {
            publisher_TofsenseFrame0_->publish(g_msg_frame0);
          }
        });

    if (is_inquire_mode_)
    {
      timer_scan_ = node->create_wall_timer(
          100ms,
          [=]() // (const ros::TimerEvent &)
          {
            frame0_map_.clear();
            node_index_ = 0;
            // timer_read_.start();
            timer_read_->reset();
          }
          
          );

      // timer_scan_ = nh_.createTimer(
      //     ros::Duration(1.0 / frequency_),
      //     [=](const ros::TimerEvent &)
      //     {
      //       frame0_map_.clear();
      //       node_index_ = 0;
      //       timer_read_.start();
      //     },
      //     false, true);
      timer_read_ = node->create_wall_timer(
          6ms,
          [=]() // (const ros::TimerEvent &)
          {
            if (node_index_ >= 8)
            {
              if (!frame0_map_.empty())
              {
                nlink_parser_interfaces::msg::TofsenseCascade msg_cascade;
                for (const auto &msg : frame0_map_)
                {
                  msg_cascade.nodes.push_back(msg.second);
                }
                publisher_TofsenseCascade_->publish(msg_cascade);
              }
              // timer_read_.stop();
              timer_read_->cancel();
            }
            else
            {
              g_command_read.id = node_index_;
              auto data = reinterpret_cast<uint8_t *>(&g_command_read);
              NLink_UpdateCheckSum(data, sizeof(g_command_read));
              serial_->write(data, sizeof(g_command_read));
              ++node_index_;
            }
          });
      // timer_read_ = nh_.createTimer(
      //     ros::Duration(0.006),
      //     [=](const ros::TimerEvent &)
      //     {
      //       if (node_index_ >= 8)
      //       {
      //         if (!frame0_map_.empty())
      //         {
      //           nlink_parser::TofsenseCascade msg_cascade;
      //           for (const auto &msg : frame0_map_)
      //           {
      //             msg_cascade.nodes.push_back(msg.second);
      //           }
      //           publisher_TofsenseCascade_->publish(msg_cascade);
      //         }
      //         timer_read_.stop();
      //       }
      //       else
      //       {
      //         g_command_read.id = node_index_;
      //         auto data = reinterpret_cast<uint8_t *>(&g_command_read);
      //         NLink_UpdateCheckSum(data, sizeof(g_command_read));
      //         serial_->write(data, sizeof(g_command_read));
      //         ++node_index_;
      //       }
      //     },
      //     false, false);
    }
  }

} // namespace tofsense
