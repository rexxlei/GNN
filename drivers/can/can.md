# CAN 配置指南

## CAN 直连

1. 找到 `80-laike-acusb.rules` 所在文件夹。
2. 复制规则文件到系统目录：
   ```bash
   sudo cp 80-laike-acusb.rules /etc/udev/rules.d/
   ```
3. 接着在 `64bit` 文件夹运行调试。

### 安装依赖

安装 `libudev-dev`：
```bash
sudo apt update
sudo apt-get install libudev-dev
```

### 调试

```bash
sudo make install  # 安装libCanCmd.so库（新系统执行一次）
sudo make
sudo ./testLikeCan
```

## CAN 转 USB

### 1. 设置 CAN-TO-USB 适配器
开机运行一次即可：
```bash
sudo modprobe gs_usb
```

### 2. 设置波特率并启用适配器
设置 500k 波特率：
```bash
sudo ip link set can0 up type can bitrate 500000
```

每次拔出重新连接 CAN_TO_USB 线都要设置。可以通过以下命令查看 CAN 设备：
```bash
ifconfig -a
# Output: can0: flags=193<UP,RUNNING,NOARP>  mtu 16 ...
```

### 3. 安装并测试硬件
安装 `can-utils`：
```bash
sudo apt install can-utils
```

如果此时底盘已经连接，可以通过以下命令查看底盘数据：
```bash
candump can0
# Output: can0  252   [8]  00 00 00 12 00 00 00 00
```

### 4. 安装 ROS 键盘控制包
```bash
sudo apt install ros-noetic-teleop-twist-keyboard
sudo apt install libasio-dev
```