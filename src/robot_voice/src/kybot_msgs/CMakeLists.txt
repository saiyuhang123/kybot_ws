cmake_minimum_required(VERSION 2.8.3)
project(kybot_msgs)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  actionlib_msgs
  actionlib
  message_generation
  message_runtime
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  neck_angle.msg
  Velocities.msg
  kybotBmsStatus.msg
  kybotRCStatus.msg
  Custom.msg
  skeleton.msg
  joint.msg
  ar_pose.msg
  Sonar.msg
  BoundingBox.msg
  BoundingBoxes.msg
  cam_base.msg
)
add_service_files(
   FILES
   SetRelativeMove.srv
   Neck.srv
   track.srv
   SetCharge.srv
   nav_goal.srv
   pick_and_place.srv
   pick_marker.srv
   goto_position.srv
   standard_mode.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
add_action_files(FILES
  center.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs   
   std_msgs
 )


catkin_package()
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
