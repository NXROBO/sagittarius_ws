cmake_minimum_required(VERSION 2.8.3)
project(sagittarius_object_color_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_generation
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  actionlib
  actionlib_msgs
)

find_package(OpenCV REQUIRED)

add_action_files(
  FILES
  SGRCtrl.action
)

 generate_messages(
   DEPENDENCIES
   sensor_msgs
   std_msgs
   geometry_msgs
   actionlib_msgs
 )

generate_dynamic_reconfigure_options(
  config/HSVParams.cfg
)


catkin_package()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


link_directories(/usr/local/lib) # yaml-cpp lib
include_directories(/usr/local/include/yaml-cpp)
FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)

add_executable(HSVParams_node src/HSVParams_node.cpp) # executable
add_dependencies(HSVParams_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(HSVParams_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${OpenCV_LIBRARIES})


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
     message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
