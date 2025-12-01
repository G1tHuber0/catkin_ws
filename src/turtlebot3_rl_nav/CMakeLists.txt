cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_rl_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# -------------------------------------------------------------
# 步骤 1: 使用 file(GLOB) 查找 scripts/ 及其子目录下的所有 .py 文件
# 查找 scripts/ 目录下的所有 .py 文件
file(GLOB SCRIPTS_PY_FILES
    "scripts/*.py"
)

# -------------------------------------------------------------
# 步骤 2: 将所有找到的文件合并，并用于安装
catkin_install_python(PROGRAMS
    ${SCRIPTS_PY_FILES}
    ${TEST_PY_FILES}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
