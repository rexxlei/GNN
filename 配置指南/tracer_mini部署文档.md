  # Tracer_mini小车部署

## Ubuntu系统及软件环境部署

通过U盘镜像安装Ubuntu20.04

* X86工控机
    3.0系统刷机教程.doc
* 临滴工控机
    极简板XP1产品部署手册(1).pdf

软件系统主要内容包括主目录下“cotek”，其中工作空间为“cotek_ws”，配置文件包含在“config”中。


## Tracer_mini底盘驱动部署

这里主要参考 TRACER MINI用户手册 P19 3.5 TRACER MINI ROS Package 使用示例 相关内容。

1. 环境配置流程如下：

```bash
# 1.设置CAN-TO-USB适配器，开机运行一次即可
sudo modprobe gs_usb

# 2.设置500k波特率和使能can-to-usb适配器
sudo ip link set can0 up type can bitrate 500000

# 此时可以通过以下命令查看CAN设备，每次拔出重新连接CAN_TO_USB线都要设置
ifconfig -a
# Output:can0: flags=193<UP,RUNNING,NOARP>  mtu 16 ...
   
# 3.安装并使用can-utils来测试硬件
sudo apt install can-utils

# 如果此时底盘已经连接，可以通过以下命令查看底盘数据
candump can0
# Output:can0  252   [8]  00 00 00 12 00 00 00 00

# 4.下载ros键盘节点控制包
sudo apt install ros-noetic-teleop-twist-keyboard 
sudo apt install libasio-dev
```

2. ROS功能包下载编译如下：

```bash
# 克隆编译tracer_ros源码
cd ~/catkin_ws/src
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/agilexrobotics/tracer_ros.git
cd ..
catkin_make
```

3. 启动运行

```bash
# 启动底盘节点
roslaunch tracer_bringup tracer_robot_base.launch
```

4. 测试

```bash
# 启动键盘远程操作节点
roslaunch tracer_bringup tracer_teleop_keyboard.launch
```

* q/z：增大/减小最大速度
* ｉ／，：控制小车直行 or 后退
* ｋ：停止运动（使用ctrl + z，效果相同）
* ｊ／ｌ：控制小车逆时针 or 顺时针旋转
* ｕ／ｏ：控制小车左转 or 右转
* m／. ：控制小车左后 or 右后倒车

5. 注意事项

* 每次启动车辆时，需要使能CAN：

```bash
# 1.设置CAN-TO-USB适配器，开机运行一次即可
sudo modprobe gs_usb

# 2.设置500k波特率和使能can-to-usb适配器
sudo ip link set can0 up type can bitrate 500000
```

* 统一底盘坐标名
    将`/home/cotek/catkin_ws/src/tracer_ros/tracer_base/launch/tracer_base.launch`中的底盘坐标名改为 `<param name="base_frame" type="string" value="carto_base_link" />`



## SLAM节点部署

### 雷达部署 - 2D激光雷达：Bluesea LDS-E300-E
 
* 硬件连接
    激光雷达使用直流电源24V供电，激光雷达使用网线和上位机进行通信。
    
* 修改上位机IP
    激光雷达IP为：192.168.158.98，所以将上位机网口IP地址修改到同一网段下，这里修改为：192.168.158.111, 参考：https://github.com/SUSTech-AMASLAB/Lidar_C16-121B/blob/main/Notes/2.Ubuntu-Debug.md

    临时修改网口ip地址：以enp3s0网口举例：
    ```bash
    sudo ip addr flush dev enp3s0
    sudo ip addr add 192.168.158.111/24 dev enp3s0
    sudo ip link set enp3s0 up
    ```
    
* 安装激光雷达ROS包
    在ROS工作空间的src中下载激光雷达的ROS包，然后编译工作空间。
        
    ```bash
    git clone https://github.com/BlueSeaLidar/bluesea2.git
    ```

* 修改launch文件激光雷达IP
    将LDS-E300-E.launch文件中 <param name="lidar_ip" value="192.168.0.199" />  修改成 `<param name="lidar_ip" value="192.168.158.98" /> `
    
* 运行启动
    启动激光雷达launch文件， `roslaunch bluesea2 LDS-E300-E.launch`
    
* 修改雷达topic
    将bluesea2 LDS-E300-E.launch文件中的topic参数改为 ` <param name="topic" value="naviLaser1"/>`


### 雷达标定

参考推荐标定算法来获得更准确的外参：[OdomLaserCalibraTool](https://github.com/MegviiRobot/OdomLaserCalibraTool)

得到外参后修改配置文件：`/home/cotek/config/basic_config/runner_config.json`（修改文件底部“tf”参数，同时将雷达IP`192.168.158.98`填入）


### 建图

在使用cotek-slam进行定位之前，首先在活动区域内构建一个地图。

1. 提供topic：里程计`/odom`和激光雷达`/naviLaser1`

   两种方式实现

   - Online：设置roslaunch文件的use_sim_time为False，然后启动硬件ROS驱动包；
   - Offline：`rosbag play <BAG NAME> --clock`

2. 启动cotek-slam

    ```bash
    cd /home/cotek/config/launch
    roslaunch cotek_slam.launch
    ```

3. 调用建图service

    ```bash
    rosservice call /orignMap "type: 'orignMap'
    parameters:
    - '1'
    - '1'  # 地图编号1
    - '1'  # 区域编号1
    method: 0"
    ```

4. 拉动小车扫描环境（如果使用rosbag请忽略）
5. 调用地图保存service

    ```bash
    rosservice call /saveMap "type: 'saveMap'
    parameters:
    - ''
    method: 0"
    ```

6. 地图文件全部保存在`~/config/map/地图id/区域id`目录下

### 定位

1. 将车辆停在地图原点位置

2. 提供topic：里程计`/odom`和激光雷达`/naviLaser1`

3. 启动cotek-slam

    ```bash
    cd /home/cotek/config/launch
    roslaunch cotek_slam.launch
    ```

4. 车辆会自行进行定位操作，查看定位是否成功，可以查看topic `/agvPosition`，`position_initialized`显示是否初始定位成功；也可以日志里面搜索是否有`First match`打印。等定位成功后再移动小车。

    ```bash
    # 定位信息接口类型 topic:agvPosition
    std_msgs/Header header
    bool position_initialized # 定位初始化，成功为true
    float64  localization_score  # 定位匹配度
    float64   deviation_range  # 定位偏差值 单位: m 
    float64   x  # 单位: m 
    float64   y  # 单位: m 
    float64   theta   # 单位:rad 范围: [-Pi ... Pi]
    string    zone_id           # 地图编号
    string    map_id           # 分区编号
    string    map_description  # 地图描述
    ```


## 启动SLAM

```bash
roscore
cd ~/cotek/cotek_ws/launch
roslaunch cotek.launch
```