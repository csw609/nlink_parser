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

class TOFSense : public rclcpp::Node
{
public:
    TOFSense() : Node("tofsense"), count_(0)
    {
        serial::Serial serial;
        NProtocolExtracter extracter;

        initSerial(&serial);
        tofsense::Init init(&extracter, &serial, this);

        while (rclcpp::ok())
        {
            auto available_bytes = serial.available();
            std::string str_received;
            if (available_bytes)
            {
                serial.read(str_received, available_bytes);
                extracter.AddNewData(str_received);
            }
        }
    }

private:
    size_t count_;
};