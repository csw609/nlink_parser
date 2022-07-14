cmake_minimum_required(VERSION 3.5)
project(nlink_parser)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlink_parser_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(serial REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(nutils
    include/utils/protocol_extracter/nprotocol_base.cpp
    include/utils/protocol_extracter/nprotocol_extracter.cpp
    include/utils/nlink_unpack/nlink_utils.c
    include/utils/nlink_unpack/nlink_linktrack_anchorframe0.c
    include/utils/nlink_unpack/nlink_linktrack_tagframe0.c
    include/utils/nlink_unpack/nlink_linktrack_nodeframe0.c
    include/utils/nlink_unpack/nlink_linktrack_nodeframe1.c
    include/utils/nlink_unpack/nlink_linktrack_nodeframe2.c
    include/utils/nlink_unpack/nlink_linktrack_nodeframe3.c
    include/utils/nlink_unpack/nlink_linktrack_nodeframe4.c
    include/utils/nlink_unpack/nlink_linktrack_nodeframe5.c
    include/utils/nlink_unpack/nlink_linktrack_nodeframe6.c
    include/utils/nlink_unpack/nlink_tofsense_frame0.c
    include/utils/nlink_unpack/nlink_tofsensem_frame0.c
    include/utils/nlink_unpack/nlink_linktrack_aoa_nodeframe0.c
    include/utils/nlink_unpack/nlink_iot_frame0.c
    include/utils/init_serial.cpp
    include/utils/nlink_protocol.cpp
    include/utils/nutils.cpp
)
ament_target_dependencies(nutils rclcpp std_msgs nlink_parser_interfaces)
  

add_executable(linktrack src/linktrack.cpp 
                         include/linktrack/init.cpp 
                         include/linktrack/protocols.cpp
              )
ament_target_dependencies(
  linktrack
  rclcpp
  std_msgs
  nlink_parser_interfaces
  serial
  )
target_link_libraries(linktrack nutils)

add_executable(linktrack_aoa src/linktrack_aoa.cpp 
                             include/linktrack_aoa/init.cpp 
                             include/linktrack/protocols.cpp
              )
ament_target_dependencies(
  linktrack_aoa
  rclcpp
  std_msgs
  nlink_parser_interfaces
  serial
  )
target_link_libraries(linktrack_aoa nutils)

add_executable(iot src/iot.cpp 
                   include/iot/init.cpp 
              )
ament_target_dependencies(
  iot
  rclcpp
  std_msgs
  nlink_parser_interfaces
  serial
  )
target_link_libraries(iot nutils)

add_executable(tofsensem src/tofsensem.cpp 
                   include/tofsensem/init.cpp 
              )
ament_target_dependencies(
  tofsensem
  rclcpp
  std_msgs
  nlink_parser_interfaces
  serial
  )
target_link_libraries(tofsensem nutils)

add_executable(tofsense src/tofsense.cpp 
                   include/tofsense/init.cpp 
              )
ament_target_dependencies(
  tofsense
  rclcpp
  std_msgs
  nlink_parser_interfaces
  serial
  )
target_link_libraries(tofsense nutils)

target_include_directories(linktrack PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS linktrack
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(linktrack_aoa PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS linktrack_aoa
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(iot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS iot
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(tofsensem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS tofsensem
  DESTINATION lib/${PROJECT_NAME})

target_include_directories(tofsense PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS tofsense
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
