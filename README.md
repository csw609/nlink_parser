# nlink_parser ROS2 ver

## Prerequisites
- ROS2
- Serial Library (https://github.com/nooploop-dev/serial)
- nlink_parser_interfaces (https://github.com/csw609/nlink_parser_interfaces)


## Build

```
    cd dev_ws/src  
    git clone https://github.com/csw609/nlink_parser_interfaces.git 
    git clone https://github.com/csw609/nlink_parser.git
    cd ../
    colcon build
    . install/setup.bash 
```

## Run

```
    ros2 run nlink_parser linktrack (or linktrack_aoa, iot ...)
```