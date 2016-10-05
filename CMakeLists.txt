cmake_minimum_required(VERSION 2.8.3)
project(controller_mission)


#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  geometry_msgs
  genmsg
  actionlib_msgs
  actionlib
)

catkin_python_setup()

#===============================================================================
# G E N E R A T E   M E S S A G E S   &   S E R V I C E S

add_action_files(
   FILES
   move.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
                 rospy
                 geometry_msgs
                 actionlib_msgs
  DEPENDS        system_lib

)

#===============================================================================
# I N C L U D E   F I L E S

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#===============================================================================
# I N S T A L L

install(PROGRAMS bin/controller_mission
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#==========================================================================
# U N I T   T E S T S

# Add folders to be run by python nosetests
catkin_add_nosetests(test)
