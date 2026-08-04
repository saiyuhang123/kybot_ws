cmake_minimum_required(VERSION 3.8)
project(robot_aiui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(kybot_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(CURL REQUIRED)

# Generate local service definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToOffice.srv"
  "srv/StringToVoice.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Set AIUI library path based on architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(AIUI_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/arm")
else()
  set(AIUI_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/x64")
endif()

include_directories(
  include
  src
  ${JSONCPP_INCLUDE_DIRS}
)

add_executable(robot_aiui_node
  src/robot_aiui.cpp
  src/agent_bridge.cpp
  src/utils/Base64Util.cpp
  src/utils/ConvertUtil.cpp
  src/utils/IatResultUtil.cpp
  src/utils/StreamNlpTtsHelper.cpp
  src/jsoncpp/json_reader.cpp
  src/jsoncpp/json_value.cpp
  src/jsoncpp/json_writer.cpp
)

# Get typesupport target for local service definitions
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp
)

target_link_libraries(robot_aiui_node
  ${cpp_typesupport_target}
)

ament_target_dependencies(robot_aiui_node
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  kybot_msgs
)

target_link_libraries(robot_aiui_node
  ${AIUI_LIB_DIR}/libaiui.so
  ${AIUI_LIB_DIR}/libvtn_mic1.so
  -lrt -ldl -lpthread
  ${JSONCPP_LIBRARIES}
  ${CURL_LIBRARIES}
)

install(TARGETS robot_aiui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY AIUI/
  DESTINATION share/${PROJECT_NAME}/AIUI
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
