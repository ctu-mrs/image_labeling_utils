cmake_minimum_required(VERSION 3.0.2)
project(image_labeling_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -g -Og")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  image_view
  tf
  tf2_ros
  tf2_geometry_msgs
  mrs_lib
  dynamic_reconfigure
  std_srvs
  )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)

set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

generate_dynamic_reconfigure_options(
  config/dynparam.cfg
  config/artefact_gt.cfg
  config/general.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ImageLabelingUtils
  CATKIN_DEPENDS roscpp mrs_lib image_transport std_msgs cv_bridge image_geometry image_view 
  DEPENDS Eigen
  )

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(ImageLabelingUtils src/ImageLabelingUtils.cpp)



add_dependencies(ImageLabelingUtils ${PROJECT_NAME}_gencfg)

add_dependencies(ImageLabelingUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(ImageLabelingUtils
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Eigen_LIBRARIES}
  jsoncpp_lib
  )

