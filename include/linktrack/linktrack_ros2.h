#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "init.h"
#include "utils/init_serial.h"
#include "utils/protocol_extracter/nprotocol_extracter.h"

#include <iomanip>
#include <iostream>

using namespace std::chrono_literals;

class LinkTrack : public rclcpp::Node
{
public:
    LinkTrack() : Node("linktrack"), count_(0)
    {
        serial::Serial serial;
        NProtocolExtracter protocol_extraction;

        initSerial(&serial);
        linktrack::Init init(&protocol_extraction, &serial, this);

        while (rclcpp::ok())
        {
            auto available_bytes = serial.available();
            std::string str_received;
            if (available_bytes)
            {
                serial.read(str_received, available_bytes);
                // printHexData(str_received);
                protocol_extraction.AddNewData(str_received);
            }
        }
    }

private:
    size_t count_;
};