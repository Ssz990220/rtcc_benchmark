cmake_minimum_required(VERSION 3.0.2)
project(rtcc_benchmark)

find_package(catkin REQUIRED COMPONENTS
  geometric_shapes
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  rosbag
  rviz_visual_tools
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rtcc_benchmark
 CATKIN_DEPENDS
  geometric_shapes 
  interactive_markers
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  rosbag 
  rviz_visual_tools 
  tf2_eigen 
  tf2_geometry_msgs 
  tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(bullet_benchmark_viz_node src/benchmarkBulletViz.cpp src/imarker.cpp src/interactive_robot.cpp src/pose_string.cpp)
target_link_libraries(bullet_benchmark_viz_node
  ${catkin_LIBRARIES}
)

add_executable(bullet_benchmark_viz_panda src/benchmarkBulletVizPanda.cpp src/imarker.cpp src/interactive_robot.cpp src/pose_string.cpp)
target_link_libraries(bullet_benchmark_viz_panda
  ${catkin_LIBRARIES}
)

add_executable(fcl_benchmark_node src/benchmarkFCL.cpp)
target_link_libraries(fcl_benchmark_node
  ${catkin_LIBRARIES}
)

add_executable(bullet_benchmark_node src/benchmarkBullet.cpp)
target_link_libraries(bullet_benchmark_node
  ${catkin_LIBRARIES}
)

add_executable(fcl_benchmark_viz_node src/benchmarkFCLViz.cpp src/imarker.cpp src/interactive_robot.cpp src/pose_string.cpp)
target_link_libraries(fcl_benchmark_viz_node
  ${catkin_LIBRARIES}
)

add_executable(fcl_benchmark_viz_panda src/benchmarkFCLVizPanda.cpp src/imarker.cpp src/interactive_robot.cpp src/pose_string.cpp)
target_link_libraries(fcl_benchmark_viz_panda
  ${catkin_LIBRARIES}
)

install(TARGETS 
  bullet_benchmark_viz_node
  bullet_benchmark_node
  fcl_benchmark_node
  fcl_benchmark_viz_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
