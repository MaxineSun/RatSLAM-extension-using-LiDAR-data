cmake_minimum_required(VERSION 2.8.3)
project(ratslam_ros)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp sensor_msgs nav_msgs tf visualization_msgs image_transport nav_msgs laser_geometry pcl_conversions pcl_ros) ## actionlib_msgs)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(Irrlicht REQUIRED)
find_package(OpenGL REQUIRED)

message(${OpenCV_LIBRARIES})

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TopologicalAction.msg
   TopologicalEdge.msg
   TopologicalMap.msg
   TopologicalNode.msg
   ViewTemplate.msg
)

## Generate actions in the 'msg' folder
#add_action_files(DIRECTORY action FILES DoDishes.action)
#generate_messages(DEPENDENCIES actionlib_msgs)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs  std_msgs
)

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
CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#ratslam library
#add src/ratslam/local_view_match_laser.cpp
add_library(ratslam src/ratslam/experience_map.cpp src/ratslam/posecell_network.cpp src/ratslam/ src/ratslam/local_view_match_laser.cpp src/ratslam/visual_odometry.cpp src/ratslam/ground_truth.cpp)


# uncomment is you don't have irrlicht installed
add_definitions("-DHAVE_IRRLICHT")

#add_executable(ratslam_lv src/main_lv.cpp)
#target_link_libraries(ratslam_lv 
#  ${catkin_LIBRARIES}
#  ratslam
#  ${IRRLICHT_LIBRARIES}
#  ${OPENGL_LIBRARIES}
#  ${OpenCV_LIBRARIES})

add_executable(ratslam_gt src/main_gt.cpp)
target_link_libraries(ratslam_gt
  ${catkin_LIBRARIES}
  ratslam
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(ratslam_pc src/main_pc.cpp)
target_link_libraries(ratslam_pc
  ${catkin_LIBRARIES} 
  ratslam
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(ratslam_em src/main_em.cpp)
target_link_libraries(ratslam_em
  ${catkin_LIBRARIES}
  ratslam
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(ratslam_vo src/main_vo.cpp)
target_link_libraries(ratslam_vo
  ${catkin_LIBRARIES}
  ratslam
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})
  
add_executable(ratslam_lv_laser src/main_lv_laser.cpp)
target_link_libraries(ratslam_lv_laser 
  ${catkin_LIBRARIES}
  ratslam
  ${IRRLICHT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})

#config files for devel
set(MEDIA_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/media) #devel use the files in the source dir
file(GLOB infiles "config/*.in")
foreach(infile ${infiles})
  #MESSAGE(STATUS "Process file: ${infile}")

  # Generate output file name
  string(REGEX REPLACE ".in\$" "" outfile "${infile}")
  get_filename_component(outfile "${outfile}" NAME)
  set(outfile "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/config/${outfile}")
  #MESSAGE(STATUS "Output file: ${outfile}")

  configure_file("${infile}" "${outfile}")
endforeach()

#config files for install
set(MEDIA_PATH ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/media) #install use the installed media files
file(GLOB infiles "config/*.in")
foreach(infile ${infiles})
  #MESSAGE(STATUS "Process file: ${infile}")

  # Generate output file name
  string(REGEX REPLACE ".in\$" "" outfile "${infile}")
  get_filename_component(outfile "${outfile}" NAME)
  set(outfile "${CMAKE_CURRENT_BINARY_DIR}/config/${outfile}")
  #MESSAGE(STATUS "Output file: ${outfile}")

  configure_file("${infile}" "${outfile}")
endforeach()

add_dependencies(ratslam_lv_laser ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_pc ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_em ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_vo ratslam_ros_generate_messages_cpp)
#add_dependencies(ratslam_lv ratslam_ros_generate_messages_cpp)
add_dependencies(ratslam_gt ratslam_ros_generate_messages_cpp)
#add ratslam_lv when needed
install(TARGETS ratslam ratslam_pc ratslam_em ratslam_vo ratslam_lv_laser ratslam_gt
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY src/media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
