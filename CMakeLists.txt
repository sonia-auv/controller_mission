cmake_minimum_required(VERSION 2.8.3)
project(controller_mission)


#===============================================================================
# C A T K I N   C O N F I G U R A T I O N

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
  dynamic_reconfigure
)

catkin_python_setup()

#===============================================================================
# G E N E R A T E   M E S S A G E S   &   S E R V I C E S

add_service_files(DIRECTORY srv
  FILES
  ListMissions.srv
  LoadMission.srv
  StartMission.srv
  StopMission.srv
  CurrentMission.srv
  ReceivedMission.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

generate_dynamic_reconfigure_options(
    cfg/SquareTest.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp
                 rospy
                 geometry_msgs
                 message_runtime
                 dynamic_reconfigure
)

#===============================================================================
# I N C L U D E   F I L E S

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#===============================================================================
# I N S T A L L

catkin_install_python(PROGRAMS bin/controller_mission bin/missionExecutor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#===============================================================================
# U N I T   T E S T S

# Add folders to be run by python nosetests
#catkin_add_nosetests(test)
