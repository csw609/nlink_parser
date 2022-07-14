
#include "init.h"

#include "utils/nlink_protocol.h"
#include "utils/nlink_unpack/nlink_iot_frame0.h"
#include "utils/nlink_unpack/nlink_utils.h"
#include "utils/nutils.h"

namespace
{
  class ProtocolFrame0 : public NLinkProtocol
  {
  public:
    ProtocolFrame0()
        : NLinkProtocol(true, g_iot_frame0.fixed_part_size,
                        {g_iot_frame0.frame_header, g_iot_frame0.function_mark})
    {
    }

  protected:
    void UnpackFrameData(const uint8_t *data) override
    {
      g_iot_frame0.UnpackData(data, length());
    }
  };

} // namespace

namespace iot
{
  nlink_parser_interfaces::msg::IotFrame0 g_msg_iotframe0;

  Init::Init(NProtocolExtracter *protocol_extraction, rclcpp::Node *nd)
  {
    node = nd;
    InitFrame0(protocol_extraction);
  }

  void Init::InitFrame0(NProtocolExtracter *protocol_extraction)
  {
    static auto protocol = new ProtocolFrame0;
    protocol_extraction->AddProtocol(protocol);
    protocol->SetHandleDataCallback(
        [=]
        {
          if (!publisher_IotFrame0_)
          {
            //ros::NodeHandle nh_;
            auto topic = "nlink_iot_frame0";
            publisher_IotFrame0_ = node->create_publisher<nlink_parser_interfaces::msg::IotFrame0>(topic,50);
                //nh_.advertise<nlink_parser::IotFrame0>(topic, 50);
            TopicAdvertisedTip(topic);
          }

          const auto &data = g_iot_frame0;
          g_msg_iotframe0.uid = data.uid;
          g_msg_iotframe0.nodes.resize(IOT_FRAME0_NODE_COUNT);
          for (int i = 0; i < IOT_FRAME0_NODE_COUNT; ++i)
          {
            g_msg_iotframe0.nodes[i].uid = data.nodes[i].uid;
            g_msg_iotframe0.nodes[i].cnt = data.nodes[i].cnt;
            g_msg_iotframe0.nodes[i].dis = data.nodes[i].dis;
            g_msg_iotframe0.nodes[i].aoa_angle_horizontal =
                data.nodes[i].aoa_angle_horizontal;
            g_msg_iotframe0.nodes[i].aoa_angle_vertical =
                data.nodes[i].aoa_angle_vertical;
          }

          publisher_IotFrame0_->publish(g_msg_iotframe0);
        });
  }

} // namespace iot
