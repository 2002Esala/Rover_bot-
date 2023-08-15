cmake_minimum_required(VERSION 3.0.2)
project(navstack_pub)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
)


catkin_package(

)


include_directories(

  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/imu_recieve.py
  nodes/imu_node_complimetry.py
  nodes/imu_node_madgwick.py
  nodes/imu_node_ekf.py
  nodes/tf_broadcaster.py
  nodes/rpm_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})