cmake_minimum_required(VERSION 2.8.3)
project(extrinsic_vision_sensor_calibrator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  geometry_msgs
  visualization_msgs
)

catkin_package()

catkin_python_setup()

#############
## Install ##
#############

install(PROGRAMS
  scripts/command.py
  scripts/display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(FILES
  config/config.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  launch/calibrator.launch
  launch/display.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_extrinsic_sensor_calibrator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
