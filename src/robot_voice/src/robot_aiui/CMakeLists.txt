cmake_minimum_required(VERSION 3.0.2)
project(robot_aiui)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)



## Generate services in the 'srv' folder
 add_service_files(
   FILES
   StringToVoice.srv
   NavigateToOffice.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_aiui
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
/home/sss/robot_voice/src/robot_aiui/src
 include
  ${catkin_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)
# 查找 jsoncpp 库
find_package(jsoncpp REQUIRED)

# 查找 libcurl 库 (用于大模型API请求)
find_package(CURL REQUIRED)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robot_aiui.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(robot_aiui 
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


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(robot_aiui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
link_directories(/home/sss/robot_voice/src/libs/arm)
target_link_libraries(robot_aiui
  /home/sss/robot_voice/src/libs/x64/libaiui.so  
  /home/sss/robot_voice/src/libs/x64/libvtn_mic1.so
   -lrt -ldl -lpthread
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${CURL_LIBRARIES}
 )

# target_link_libraries(agent_bridge_test
#   -lpthread
#   ${catkin_LIBRARIES}
#   ${JSONCPP_LIBRARIES}
#   ${CURL_LIBRARIES}
#  )

link_libraries(aiui)

