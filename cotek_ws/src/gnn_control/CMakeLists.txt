cmake_minimum_required(VERSION 3.0.2)
project(gnn_control)

# 找包时加上 Eigen3 和 std_msgs
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs        # 【新增】：标准消息依赖
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs tf  # 【新增】：导出时依赖 std_msgs
#  DEPENDS Eigen3
)

# 加入 Eigen 的头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 生成 GNN 控制节点可执行文件
add_executable(gnn_control_node src/gnn_controller.cpp)
target_link_libraries(gnn_control_node
  ${catkin_LIBRARIES}
)

# 生成数据记录节点可执行文件
add_executable(data_logger_node src/data_logger_node.cpp)
target_link_libraries(data_logger_node 
  ${catkin_LIBRARIES}
)

# 安装规则
install(TARGETS gnn_control_node data_logger_node  # 【优化】：一并安装两个节点
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)