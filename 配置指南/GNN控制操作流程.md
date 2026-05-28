# Sim-to-Real GNN 编队控制系统操作指南

本文档详细说明了如何使用 `gnn_control` 功能包实现从接收 NOKOV 动捕数据、执行 GNN 在线训练与推理，到最终向 Tracer 小车集群发送控制指令的完整流程。

## 1. 系统架构图

```mermaid
graph LR
    A[NOKOV 动捕系统] -- VRPN --> B(vrpn_client_node)
    B -- /vrpn_client_node/tracerX/pose --> C[GNN 控制节点 (gnn_node)]
    C -- 内部自训练 --> C
    C -- /tracerX/cmd_vel --> D[Tracer 小车 (底层驱动)]
```

## 2. 环境准备

确保工控机连接到与 NOKOV 主机和 Tracer 小车同一局域网内。

*   **硬件**: 5台 Tracer 差速小车，一套 NOKOV 动捕系统。
*   **软件**: Ubuntu 20.04 + ROS Noetic。

## 3. 编译功能包

在开始之前，确保工作空间已正确编译。该功能包包含 C++ 实现的神经网络训练和推理算法。

```bash
cd ~/GNN/cotek_ws
catkin_make --only-pkg-with-deps gnn_control
source devel/setup.bash
```

## 4. 详细操作步骤

### 步骤 1: 启动动捕数据接收

首先运行 `vrpn_client_ros` 以从 NOKOV 系统获取位姿数据。确保 `ros-noetic-vrpn-client-ros` 已安装。

```bash
# 示例命令 (需根据实际 server IP 修改)
roslaunch vrpn_client_ros sample.launch server:=<NOKOV_IP>
```
*   **验证**: 使用 `rostopic list` 检查是否有名为 `/vrpn_client_node/tracer1/pose` 等话题。

### 步骤 2: 启动 GNN 控制节点

启动核心控制节点。该节点集成了“Sim-to-Real”的关键逻辑：**启动时自动进行在线强化训练**。

```bash
roslaunch gnn_control gnn_control.launch
```

**运行过程观察**:
1.  **初始化 (Training Phase)**:
    *   终端会显示 `>>> [Training] Starting Online LQR Training (BPTT, 800 steps)...`。
    *   节点会在后台利用 BPTT 算法针对当前拓扑结构训练 GNN 权重（耗时约几秒至十几秒）。
    *   在此期间，小车保持静止。

2.  **等待数据 (Waiting Phase)**:
    *   训练完成后，节点会检查是否收到所有 5 台小车的动捕数据。
    *   如果数据不全，终端会输出 `Waiting for VRPN data...`。

3.  **开始控制 (Control Phase)**:
    *   一旦数据就绪，节点开始按照 $20\text{Hz}$ 的频率发布控制指令。
    *   终端不再输出大量日志，小车开始自主协同编队。

### 步骤 3: 监控与可视化 (可选)

打开 Rviz 查看编队状态：

```bash
rosrun rviz rviz
```
*   将 `Fixed Frame` 设置为 `world` (或动捕对应的 frame)。
*   添加 `TF` 或 `Pose` 插件以可视化小车位置。

## 5. 关键参数说明

如果需要调整编队间距或巡航速度，可修改头文件 `src/gnn_control/include/gnn_control/gnn_controller.h` 中的 `Config` 结构体：

| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `N` | 5 | 机器人数量 |
| `L` | 0.2 | 前视点距离 (米)，影响转向灵敏度 |
| `v_cruise` | 0.8 | 编队前进的巡航速度 (m/s) |
| `learning_rate` | 0.01 | 在线训练的学习率 |
| `train_steps` | 800 | 启动时的训练迭代次数 |

修改后需要重新编译：
```bash
catkin_make --only-pkg-with-deps gnn_control
```

## 6. 常见问题排查

*   **问题**: 节点一直显示 `Waiting for VRPN data...`
    *   **解决**: 检查动捕软件中刚体名称是否严格命名为 `tracer1` 到 `tracer5`；检查网络防火墙是否允许 VRPN 端口通信。
*   **问题**: 小车原地打转或震荡
    *   **解决**: 可能是动捕坐标系与小车自身坐标系定义不一致（例如 Yaw 角相差 90 度）。可能需要调整 `gnn_node.cpp` 中的 `getYaw` 或 VRPN 配置。
*   **问题**: 训练过程 Loss 不下降
    *   **解决**: 检查 `learning_rate` 是否过大或过小，或者拓扑结构定义是否有误。
