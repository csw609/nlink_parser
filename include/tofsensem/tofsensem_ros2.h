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

#include <thread>

using namespace std::chrono_literals;

class TOFSenseM : public rclcpp::Node
{
public:
    TOFSenseM() : Node("tofsensem"), count_(0)
    {
        serial::Serial serial;
        NProtocolExtracter extracter;

        initSerial(&serial);
        tofsensem::Init init(&extracter, this);

        while (rclcpp::ok())
        {
            auto available_bytes = serial.available();
            std::string str_received;
            if (available_bytes)
            {
                serial.read(str_received, available_bytes);
                extracter.AddNewData(str_received);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

private:
    size_t count_;
};