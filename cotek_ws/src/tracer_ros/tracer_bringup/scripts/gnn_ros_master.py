#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation as R

# ==========================================
# 1. 配置参数 (与仿真代码由于保持一致)
# ==========================================
class Config:
    N = 5                
    dim_state = 2        # 2维状态 (x, y)
    dim_control = 2      
    dt = 0.05            
    L = 0.2              
    
    # 训练参数
    hidden_dim = 32      
    train_steps = 800    
    batch_size = 64      
    time_horizon = 40    
    learning_rate = 0.01 
    
    # 策略: 直接全速前进
    v_cruise = 0.8       # 前进速度

args = Config()

# ==========================================
# 2. GNN 模型 (LQR Consensus)
# ==========================================
class GNN_Consensus_LQR(nn.Module):
    def __init__(self, n_in, n_hidden, n_out):
        super(GNN_Consensus_LQR, self).__init__()
        self.H = nn.Linear(n_in, n_hidden, bias=False)
        self.K = nn.Linear(n_hidden, n_out, bias=False)
        self.activation = nn.Tanh()

    def forward(self, x, adj_matrix, offsets):
        N = x.shape[1]
        
        x_i = x.unsqueeze(2) 
        x_j = x.unsqueeze(1) 
        
        # 计算带 offset 的误差
        if offsets is not None:
            off_i = offsets.unsqueeze(1)
            off_j = offsets.unsqueeze(0)
            D_ij = off_i - off_j
            raw_error = (x_i - x_j) - D_ij.unsqueeze(0)
        else:
            raw_error = x_i - x_j
            
        # 应用特定通信拓扑 Mask
        mask = adj_matrix.view(1, N, N, 1)
        masked_error = raw_error * mask
        
        # 聚合
        e_i = torch.sum(masked_error, dim=2) 
        degree = torch.sum(adj_matrix, dim=1).view(1, N, 1)
        e_i = e_i / (degree + 1e-6)
        
        # 控制输出
        z = self.activation(self.H(e_i))
        u_gnn = -self.K(z) # 负反馈
        
        return u_gnn, e_i

# ==========================================
# 3. 辅助函数
# ==========================================
def get_custom_adjacency(N):
    adj = torch.zeros(N, N)
    # 机器人索引: 1->0, 2->1, 3->2, 4->3, 5->4
    # Pairs: (0,2), (1,2), (2,3), (3,4)
    pairs = [(0, 2), (1, 2), (2, 3), (3, 4)]
    
    for i, j in pairs:
        adj[i, j] = 1.0
        adj[j, i] = 1.0
    return adj

# ==========================================
# 4. ROS 主节点
# ==========================================
class GNNFormationMaster:
    def __init__(self):
        rospy.init_node('gnn_formation_master')
        rospy.loginfo(">>> GNN Formation Master Node Started")

        # 1. 训练/初始化模型
        self.model = self.train_and_get_model()
        self.model.eval() # 切换到推理模式

        # 2. 状态变量
        self.N = args.N
        self.robot_poses = [None] * self.N  # 存储 [x, y, theta]
        self.robot_active = [False] * self.N # 检查是否收到数据

        # 3. 拓扑与编队设置
        self.adj = get_custom_adjacency(self.N)
        
        # 目标队形: Y轴排开
        offsets = []
        spacing = 1.5 
        # ID: 1, 2, 3, 4, 5 -> indices: 0, 1, 2, 3, 4
        # 相对位置: 2, 1, 0, -1, -2
        center_y_indices = [2, 1, 0, -1, -2] 
        for i in range(self.N):
            offsets.append([0.0, center_y_indices[i] * spacing])
        self.offsets_tensor = torch.tensor(offsets)
        
        self.v_global = torch.tensor([args.v_cruise, 0.0])

        # 4. ROS 通信接口
        self.pubs = []
        self.subs = []
        
        # 假设机器人命名空间为 tracer1 ... tracer5
        # 假设动捕话题为 /vrpn_client_node/tracerX/pose
        
        topic_prefix = "tracer" 
        
        for i in range(self.N):
            robot_id = i + 1
            
            # Publisher: /tracerX/cmd_vel
            pub = rospy.Publisher(f'/{topic_prefix}{robot_id}/cmd_vel', Twist, queue_size=1)
            self.pubs.append(pub)
            
            # Subscriber: /vrpn_client_node/tracerX/pose
            # 使用 lambda 闭包捕获 i
            sub = rospy.Subscriber(
                f'/vrpn_client_node/{topic_prefix}{robot_id}/pose',
                PoseStamped,
                lambda msg, idx=i: self.pose_callback(msg, idx)
            )
            self.subs.append(sub)

        # 5. 定时器 (控制频率)
        self.timer = rospy.Timer(rospy.Duration(args.dt), self.control_loop)
        rospy.loginfo(f">>> Control Loop Initialized at {1.0/args.dt} Hz")

    def train_and_get_model(self):
        """
        在节点启动时执行训练，获取适应当前拓扑的权重
        """
        rospy.loginfo(">>> [Training] 开始训练 GNN 纠偏网络 (约需几秒)...")
        model = GNN_Consensus_LQR(args.dim_state, args.hidden_dim, args.dim_control)
        optimizer = optim.Adam(model.parameters(), lr=args.learning_rate)
        adj = get_custom_adjacency(args.N)
        
        for step in range(args.train_steps):
            # 随机初始化
            x_t = torch.rand(args.batch_size, args.N, args.dim_state) * 12 - 6
            total_loss = 0
            
            for t in range(args.time_horizon):
                u_gnn, error_val = model(x_t, adj, offsets=None)
                
                loss_formation = torch.mean(error_val**2) * 2.0  
                loss_control = torch.mean(u_gnn**2) * 0.02       
                
                total_loss += loss_formation + loss_control
                
                x_t = x_t + u_gnn * args.dt
                
            optimizer.zero_grad()
            total_loss.backward()
            optimizer.step()
            
            if (step+1) % 200 == 0:
                rospy.loginfo(f"    Step {step+1}/{args.train_steps}, Loss: {total_loss.item():.4f}")
                
        rospy.loginfo(">>> [Training] 训练完成!")
        return model

    def pose_callback(self, msg, robot_idx):
        try:
            x = msg.pose.position.x
            y = msg.pose.position.y
            
            # 四元数转欧拉角 (Yaw)
            q = msg.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            r = R.from_quat(quat)
            theta = r.as_euler('xyz')[2] # 返回的是弧度
            
            self.robot_poses[robot_idx] = [x, y, theta]
            self.robot_active[robot_idx] = True
        except Exception as e:
            rospy.logerr(f"Error in pose callback for robot {robot_idx+1}: {e}")

    def control_loop(self, event):
        # 1. 安全检查: 是否所有机器人数据都已就位
        if not all(self.robot_active):
            waiting = [i+1 for i, status in enumerate(self.robot_active) if not status]
            rospy.logwarn_throttle(2, f"Waiting for poses from robots: {waiting}")
            # 安全起见，发布 0 速
            self.stop_all()
            return

        # 2. 准备状态数据
        # 提取当前物理位置
        current_data = np.array(self.robot_poses) # Shape: (N, 3) -> x, y, theta
        phys_pos = torch.tensor(current_data[:, :2], dtype=torch.float32) # (N, 2)
        theta = torch.tensor(current_data[:, 2], dtype=torch.float32)     # (N,)

        # 3. 反馈线性化变换 (Look-ahead Point)
        # 计算前视点位置
        x_phys = phys_pos[:, 0]
        y_phys = phys_pos[:, 1]
        
        x_L = x_phys + args.L * torch.cos(theta)
        y_L = y_phys + args.L * torch.sin(theta)
        
        # GNN 输入: (1, N, 2)
        X_virt = torch.stack([x_L, y_L], dim=1).unsqueeze(0)

        # 4. GNN 推理
        with torch.no_grad():
            u_gnn, _ = self.model(X_virt, self.adj, self.offsets_tensor)
            u_gnn = u_gnn.squeeze(0) # (N, 2)

        # 5. 策略叠加: 纠偏 + 前馈前进
        u_final = u_gnn + self.v_global
        
        # 6. 逆运动学解算 & 发布指令
        for i in range(self.N):
            vx_world = u_final[i, 0].item()
            vy_world = u_final[i, 1].item()
            th = theta[i].item()
            
            # World Frame -> Body Frame (v, w)
            # v = vx * cos(th) + vy * sin(th)
            # w = (-vx * sin(th) + vy * cos(th)) / L
            
            v_cmd = np.cos(th)*vx_world + np.sin(th)*vy_world
            w_cmd = (-np.sin(th)*vx_world + np.cos(th)*vy_world) / args.L
            
            # 安全限幅
            v_cmd = np.clip(v_cmd, -1.0, 1.0) # 实车速度不宜过大
            w_cmd = np.clip(w_cmd, -1.5, 1.5)
            
            # 发布 Twist
            twist = Twist()
            twist.linear.x = v_cmd
            twist.angular.z = w_cmd
            
            self.pubs[i].publish(twist)

    def stop_all(self):
        stop_msg = Twist()
        for pub in self.pubs:
            pub.publish(stop_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = GNNFormationMaster()
        node.run()
    except rospy.ROSInterruptException:
        pass
