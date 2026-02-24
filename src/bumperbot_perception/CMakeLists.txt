cmake_minimum_required(VERSION 3.0.2)
project(bumperbot_perception)


find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  pcl_conversions
  cv_bridge
  image_transport
)


find_package(PCL REQUIRED)
find_package(OpenCV_REQUIRED)






catkin_package(
      CATKIN_DEPENDS roscpp cv_bridge image_transport
)


include_directories(

  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(point_cloud_node src/point_cloud_filter_node.cpp)

target_link_libraries(point_cloud_node
  ${catkin_LIBRARIES}
 
)

add_dependencies(point_cloud_node${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


  







