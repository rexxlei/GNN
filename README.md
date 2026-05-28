# 基于GNN的多智能体分布式协同控制算法

### 启动文件： `cotek_ws/launch/GNN.launch`

### 控制代码源文件：`cotek_ws/src/src/gnn_controller.cpp`
# <基于GNN的多智能体分布式协同控制算法>

![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen)

[![Paper](https://img.shields.io/badge/Paper-IEEE%20RA--L%20Under%20Review-red)](#citation)

> **<该项目提出了一种基于图神经网络（GNN）的完全分布式协同控制算法，有效解决了异构与非线性多智能体系统的状态一致性难题，并在多台实体移动机器人上成功验证了其抗物理噪声的强鲁棒性与编队同步能力。>**

## 🎥 效果演示 (Demo)

<video src="./demo.mp4" autoplay="true" controls="controls" width="800" height="450">
</video>

*图：真实环境下实体移动机器人速度和位置一致性收敛*

## ✨ 核心特性 (Features)

* **完全分布式协同框架**：基于图神经网络（GNN），使多智能体仅依赖局部邻居信息交互即可实现全局的状态一致性与平滑收敛，打破了传统控制对全局通信的依赖。
* **异构与非线性动力学适配**：针对差分驱动机器人的底盘欠驱动约束，引入前视距离法（Look-ahead）完成状态映射与全驱动模型转换，并通过逆运动学将 GNN 虚拟指令精准解算为物理线/角速度。
* **强鲁棒性**：在包含未建模摩擦与物理噪声的 5 台实体机器人协同实验中展现出极高的稳定性。

## 🛠️ 环境依赖 (Prerequisites)

本项目在以下软硬件环境下测试通过：

**硬件要求：**
* 小车底盘：AgileX Tracer Mini
* 控制核心：工控机
* 定位系统：全局动捕系统

**软件依赖：**
* Ubuntu 20.04
* ROS Noetic
* Python >= 3.8
* PyTorch >= 2.0 (CUDA 11.8)

## 🚀 安装指南 (Installation)

1. **克隆仓库：**
   ```bash
   git clone [https://github.com/rexxlei/GNN.git)
    ```
2. **文件说明：**
    - 启动文件： `cotek_ws/launch/GNN.launch`

    - 控制代码源文件：`cotek_ws/src/src/gnn_controller.cpp`

    - 权重： `cotek_ws\src\gnn_control\scripts`

    - 仿真文件： `gnn_multirobot_sim.ipynb`
3. **运行算法：**
```
# 编译工作空间
   cd cotek_ws
   catkin_make
   source devel/setup.bash
   
   # 启动 GNN 控制节点
   roslaunch cotek_ws GNN.launch
   ```