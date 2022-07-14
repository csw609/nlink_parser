//#include "nlink_parser/utils/init_serial.h"
//#include "init_serial.h"
#include "init_serial.h"
//#include <ros/ros.h>

#include <string>
#include <iostream>
/*
void enumerate_ports() {
  auto devices_found = serial::list_ports();
  auto iter = devices_found.begin();
  while (iter != devices_found.end()) {
    serial::PortInfo device = *iter++;

    printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
           device.hardware_id.c_str());
  }
  std::string test;
  test.clear();
}
*/

void initSerial(serial::Serial *serial)
{
  try
  {
    auto port_name = "/dev/ttyUSB0"; // To Do : Parameterization
        //ros::param::param<std::string>("~port_name", "/dev/ttyUSB0");
    auto baud_rate = 921600; // To Do : Parameterization
    //ros::param::param<int>("~baud_rate", 921600);

    serial->setPort(port_name);
    serial->setBaudrate(static_cast<uint32_t>(baud_rate));

    //ROS_INFO("try to open serial port with %s,%d", port_name.data(), baud_rate);
    std::cout << "try to open serial port with " << port_name << " , " << baud_rate << "\n";
    auto timeout = serial::Timeout::simpleTimeout(10);
    // without setTimeout,serial can not write any data
    // https://stackoverflow.com/questions/52048670/can-read-but-cannot-write-serial-ports-on-ubuntu-16-04/52051660?noredirect=1#comment91056825_52051660
    serial->setTimeout(timeout);
    serial->open();

    if (serial->isOpen())
    {
      //ROS_INFO("Serial port opened successfully, waiting for data.");
      std::cout << "Serial port opened successfully, waiting for data." << "\n";
    }
    else
    {
      //ROS_ERROR("Failed to open serial port, please check and retry.");
      std::cout << "Failed to open serial port, please check and retry." << "\n";
      exit(EXIT_FAILURE);
    }
  }
  catch (const std::exception &e)
  {
    //ROS_ERROR("Unhandled Exception: %s", e.what());
    std::cout << "Unhandled Exception: " <<  e.what() << "\n";
    exit(EXIT_FAILURE);
  }
}
