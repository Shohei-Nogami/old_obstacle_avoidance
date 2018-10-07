cmake_minimum_required(VERSION 2.8.3)
project(obst_avoid)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	roscpp
	rospy
	std_msgs
	message_generation
	cv_bridge
	image_transport
    tf
    nav_msgs
    pcl_ros
    geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   wheel_msg.msg
   prev_vel_msg.msg
   moving_points.msg
   moving_pointsArray.msg
   points.msg
   point3d.msg
   line_point3d.msg
   sqr_point3d.msg
   pointz.msg
   line_pointz.msg
   sqr_pointz.msg
   dvw.msg
   rectangle.msg
   detected_objects.msg
   vel3d.msg
   img_point.msg
   matching.msg
   point2i.msg
   cluster_element.msg
   cluster_point.msg
   cluster.msg
   cluster_with_vel.msg
   object_info.msg
   objects_info.msg
   filted_objects_info.msg
   filted_object_info.msg
   point2d.msg
   robot_odm.msg
   select_theta.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   image.srv
   odometry.srv
   sncr.srv
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
   std_msgs  # Or other packages containing msgs
   sensor_msgs
   nav_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obst_avoid
  CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/obst_avoid.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/obst_avoid_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_obst_avoid.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


#add_executable(NodeName src/sourcecode.cpp )
#target_link_libraries(NodeName ${catkin_LIBRARIES} )

add_executable(wheel_control_class_test src/wheel_control/whl_cntrl.cpp )
target_link_libraries(wheel_control_class_test  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(culculating_opticalflow_class_test src/clc_optflw_cls/culculating_opticalflow_class.cpp src/img_cls/image_class.cpp src/img_cls/depth_image_class.cpp src/odm_cls/wheel_odometry_class.cpp src/odm_cls/visual_odometry_class.cpp  src/tm_cls/time_class.cpp)
target_link_libraries(culculating_opticalflow_class_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(detect_objects_class_test src/dtct_obj_cls/detect_objects_class.cpp src/dtct_obj_cls/density_based_clustering.cpp src/tm_cls/time_class.cpp src/img_cls/image_class.cpp src/odm_cls/odometry_class.cpp src/odm_cls/wheel_odometry_class.cpp src/odm_cls/visual_odometry_class.cpp)
target_link_libraries(detect_objects_class_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(culculate_centroid_class_test src/clc_cnt/culculate_centroid.cpp src/tm_cls/time_class.cpp src/img_cls/image_class.cpp)
target_link_libraries(culculate_centroid_class_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(estimate_velocity_class_test src/est_vel/estimate_velocity.cpp src/tm_cls/time_class.cpp )
target_link_libraries(estimate_velocity_class_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(avoid_class_test src/avoid_cls/avoid_cls.cpp src/vfh+_cls/vfh+_class.cpp src/tm_cls/time_class.cpp )
target_link_libraries(avoid_class_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pub_odm src/odm_cls/pub_odm.cpp src/tm_cls/time_class.cpp src/odm_cls/odometry_class.cpp src/odm_cls/wheel_odometry_class.cpp src/odm_cls/visual_odometry_class.cpp)
target_link_libraries(pub_odm ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(beego_teleop_key src/beego_teleop/beego_teleop_key.cpp)
target_link_libraries(beego_teleop_key ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(udpipreceiver src/com_win_pc/recv1.cpp)
target_link_libraries(udpipreceiver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(apf_test src/apf/apf.cpp src/apf/gradient_map.cpp src/apf/path_planning.cpp src/apf/coordinate_transform.cpp src/apf/grid_map.cpp src/apf/potential_map.cpp src/apf/set_param.cpp src/apf/apf_test.cpp src/apf/debug.cpp)
#target_link_libraries(apf_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
##
#add_executable(apf_mpc_test src/apf_mpc/apf_mpc_test.cpp
#
#src/apf_mpc/apf_mpc.cpp src/apf_mpc/set_data.cpp src/apf_mpc/virtual_fnc.cpp src/apf_mpc/mv_pot_map.cpp  src/apf_mpc/mpc_func.cpp src/apf_mpc/debug.cpp
#
#src/apf/apf.cpp src/apf/gradient_map.cpp src/apf/path_planning.cpp src/apf/coordinate_transform.cpp src/apf/grid_map.cpp src/apf/potential_map.cpp src/apf/set_param.cpp src/apf/debug.cpp)
#
#target_link_libraries(apf_mpc_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
##
#set(CMAKE_CXX_FLAGS “-std=c++0x ${CMAKE_CXX_FLAGS}”)
add_definitions(-std=c++11)
