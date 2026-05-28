#!/bin/bash
# =====================================================
# 自动化脚本：配置 vcan0 与 rc.local 自启环境
# 适用于 Ubuntu 系统（root 权限执行）
# =====================================================

set -e  # 出错时退出
echo "=============================="
echo "⚙️ 开始配置 vcan0 和自启动脚本..."
echo "=============================="

# -----------------------------------------------------
# 1. 检查并安装必要组件
# -----------------------------------------------------
echo "[1/5] 检查 can-utils 是否已安装..."
if ! command -v cansend &> /dev/null; then
    echo "未检测到 can-utils，正在安装..."
    sudo apt update
    sudo apt install -y can-utils
else
    echo "✅ can-utils 已安装"
fi

# -----------------------------------------------------
# 2. 创建并启用 systemd 服务：vcan0.service
# -----------------------------------------------------
echo "[2/5] 创建 /etc/systemd/system/vcan0.service ..."
sudo tee /etc/systemd/system/vcan0.service > /dev/null <<EOF
[Unit]
Description=Virtual CAN interface vcan0
After=network.target
Wants=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ip link add dev vcan0 type vcan
ExecStartPost=/sbin/ip link set vcan0 up
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable vcan0.service
sudo systemctl start vcan0.service

# 验证 vcan0 是否创建成功
if ip link show vcan0 &> /dev/null; then
    echo "✅ vcan0 已成功创建并启动"
else
    echo "❌ vcan0 启动失败，请检查 systemctl 日志"
    exit 1
fi

# -----------------------------------------------------
# 3. 启用 rc.local 服务
# -----------------------------------------------------
echo "[3/5] 启用 rc-local.service..."
sudo systemctl enable rc-local.service || true

# -----------------------------------------------------
# 4. 创建 /etc/rc.local 脚本
# -----------------------------------------------------
echo "[4/5] 创建 /etc/rc.local ..."

sudo tee /etc/rc.local > /dev/null <<'EOF'
#!/bin/bash
# ==========================
# rc.local 自启动脚本
# ==========================
# 注意：rc.local 以 root 身份运行，无需 sudo

# 1. 修改 3D雷达 有线网口 IP (eth2)
# ip addr flush dev eth2
# ip addr add 192.168.1.50/24 dev eth2
# ip link set eth2 up

# 2. 修改 2D雷达 有线网口 IP (eth1)
# ip addr flush dev eth1
# ip addr add 192.168.158.111/24 dev eth1
# ip link set eth1 up

# 3. 启动虚拟CAN
systemctl start vcan0.service

# 4. 设置 WiFi 网口 DHCP (eth3)
# 给一点延时等待网卡就绪
# sleep 5
# dhclient eth3

# 可在此添加 tracer_bridge 启动指令，例如：
# source /opt/ros/noetic/setup.bash
# roslaunch tracer_bringup tracer_bridge.launch &

exit 0
EOF

sudo chmod +x /etc/rc.local

# -----------------------------------------------------
# 5. 验证 rc.local 是否生效
# -----------------------------------------------------
if sudo systemctl status rc-local.service | grep -q "enabled"; then
    echo "✅ rc.local 服务已启用"
else
    echo "⚠️ rc.local 服务未启用，请手动检查 systemctl 配置"
fi

echo "=============================="
echo "🎉 所有配置完成！"
echo "现在可以重启验证效果：sudo reboot"
echo "=============================="