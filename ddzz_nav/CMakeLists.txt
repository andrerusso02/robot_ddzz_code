cmake_minimum_required(VERSION 3.0.2)
project(ddzz_nav)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ddzz_nav
#  CATKIN_DEPENDS geometry_msgs rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(
#   PROGRAMS scripts/gotogoal.py scripts/round_trip.py
#   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )