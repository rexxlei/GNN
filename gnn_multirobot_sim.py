import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

# --- 环境设置 ---
import matplotlib
try:
    matplotlib.use('TkAgg') # 优先使用弹窗后端
except:
    pass
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

plt.close('all')

# ==========================================
# 1. 配置参数
# ==========================================
class Config:
    N = 5                
    dim_state = 2        # 2维状态 (x, y)
    dim_control = 2      
    dt = 0.05            
    L = 0.2              
    
    # 训练参数
    hidden_dim = 32      
    train_steps = 800    # 加大训练步数，确保学到强力的收敛策略
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
# 3. 特定拓扑: 1-3, 2-3, 3-4, 4-5
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

def get_robot_box(x, y, theta, width=0.6, length=0.8):
    corners = np.array([
        [ length/2,  width/2], [-length/2,  width/2],
        [-length/2, -width/2], [ length/2, -width/2]
    ])
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, -s], [s, c]])
    return (R @ corners.T).T + np.array([x, y])

# ==========================================
# 4. 主流程
# ==========================================
def main_pipeline():
    # --- A. 训练 ---
    print(">>> 1/3 [Training] 正在训练强力纠偏网络...")
    model = GNN_Consensus_LQR(args.dim_state, args.hidden_dim, args.dim_control)
    optimizer = optim.Adam(model.parameters(), lr=args.learning_rate)
    adj = get_custom_adjacency(args.N)
    
    loss_history = []
    
    for step in range(args.train_steps):
        # 随机初始化 (大幅度随机，训练鲁棒性)
        x_t = torch.rand(args.batch_size, args.N, args.dim_state) * 12 - 6
        total_loss = 0
        
        for t in range(args.time_horizon):
            u_gnn, error_val = model(x_t, adj, offsets=None)
            
            # 策略：为了保证"一定收敛"，给 error 极高的权重
            loss_formation = torch.mean(error_val**2) * 2.0  # 加权
            loss_control = torch.mean(u_gnn**2) * 0.02       # 允许较大的控制动作
            
            total_loss += loss_formation + loss_control
            
            x_t = x_t + u_gnn * args.dt
            
        optimizer.zero_grad()
        total_loss.backward()
        optimizer.step()
        loss_history.append(total_loss.item())

    # --- B. 仿真 (无 Warmup，直接跑) ---
    print(">>> 2/3 [Simulation] 生成轨迹 (同步收敛)...")
    
    # 目标队形: Y轴排开
    offsets = []
    spacing = 1.5 
    # ID: 1, 2, 3, 4, 5 -> indices: 0, 1, 2, 3, 4
    # 相对位置: 2, 1, 0, -1, -2
    center_y_indices = [2, 1, 0, -1, -2] 
    for i in range(args.N):
        offsets.append([0.0, center_y_indices[i] * spacing])
    offsets_tensor = torch.tensor(offsets)
    
    # 物理初始化: 前后错开(防止碰撞), 上下散开
    q = torch.rand(args.N, 3)
    q[:, 0] = q[:, 0] * 6 - 8   # X: -8 ~ -2 (拉长分布，避免初始碰撞)
    q[:, 1] = q[:, 1] * 8 - 4   # Y: -4 ~ 4
    q[:, 2] = (torch.rand(args.N) * 2 * np.pi) - np.pi # 随机朝向
    
    traj_q = [[] for _ in range(args.N)]
    sim_steps = 400 
    
    # 全局前进指令 (恒定)
    v_global = torch.tensor([args.v_cruise, 0.0])
    
    with torch.no_grad():
        for t in range(sim_steps):
            x_phys, y_phys, theta = q[:, 0], q[:, 1], q[:, 2]
            
            x_L = x_phys + args.L * torch.cos(theta)
            y_L = y_phys + args.L * torch.sin(theta)
            X_virt = torch.stack([x_L, y_L], dim=1).unsqueeze(0)
            
            # GNN 计算纠偏速度
            u_gnn, _ = model(X_virt, adj, offsets_tensor)
            u_gnn = u_gnn.squeeze(0)
            
            # 叠加: 纠偏 + 前进
            u_final = u_gnn + v_global
            
            for i in range(args.N):
                vx, vy = u_final[i, 0].item(), u_final[i, 1].item()
                th = theta[i].item()
                
                # 逆解
                v = np.cos(th)*vx + np.sin(th)*vy
                w = (-np.sin(th)*vx + np.cos(th)*vy) / args.L
                
                # 限幅 (放宽一点，让它能快速转弯归位)
                v = np.clip(v, -2.0, 2.0)
                w = np.clip(w, -3.0, 3.0)
                
                q[i, 0] += v * np.cos(th) * args.dt
                q[i, 1] += v * np.sin(th) * args.dt
                q[i, 2] += w * args.dt
                traj_q[i].append(q[i].clone().numpy())

    return traj_q, loss_history

# ==========================================
# 5. 可视化
# ==========================================
if __name__ == "__main__":
    traj_q, loss_hist = main_pipeline()
    
    print(">>> 3/3 [Visualization] 启动窗口...")
    
    fig, (ax_loss, ax_sim) = plt.subplots(1, 2, figsize=(14, 7))
    
    # Loss
    ax_loss.plot(loss_hist, 'b-', lw=2)
    ax_loss.set_title("Training Loss (High Penalty for Error)")
    ax_loss.grid(True)
    
    # Sim
    ax_sim.set_aspect('equal')
    ax_sim.grid(True, linestyle='--', alpha=0.5)
    # 动态参考线范围
    for x_ref in range(-10, 80, 2):
        ax_sim.axvline(x=x_ref, color='red', linestyle=':', alpha=0.2)
    
    # 初始化绘图
    colors = plt.cm.rainbow(np.linspace(0, 1, args.N))
    patches = []
    texts = []
    trails = []
    
    # 通信连线 (1-3, 2-3, 3-4, 4-5)
    comm_pairs = [(0, 2), (1, 2), (2, 3), (3, 4)]
    comm_lines = []
    for _ in comm_pairs:
        line, = ax_sim.plot([], [], 'g--', alpha=0.5, lw=1.5)
        comm_lines.append(line)
    
    for i in range(args.N):
        poly = Polygon([[0,0]], facecolor=colors[i], edgecolor='k', alpha=0.7)
        ax_sim.add_patch(poly)
        patches.append(poly)
        
        txt = ax_sim.text(0, 0, str(i+1), color='white', fontweight='bold', ha='center', va='center')
        texts.append(txt)
        
        line, = ax_sim.plot([], [], '-', color=colors[i], alpha=0.3, lw=1)
        trails.append(line)
        
    def update(frame):
        # 3倍速播放，让过程更紧凑
        idx = frame * 3
        if idx >= len(traj_q[0]): idx = len(traj_q[0]) - 1
        
        # 获取位置
        current_positions = []
        for i in range(args.N):
            state = traj_q[i][idx]
            current_positions.append(state[:2])
            
            # 更新小车
            patches[i].set_xy(get_robot_box(state[0], state[1], state[2]))
            texts[i].set_position((state[0], state[1]))
            
            # 尾迹
            hist_start = max(0, idx - 60)
            hist = np.array(traj_q[i])[hist_start:idx+1]
            if len(hist) > 0:
                trails[i].set_data(hist[:, 0], hist[:, 1])
        
        # 更新通信线
        for k, (i, j) in enumerate(comm_pairs):
            p1 = current_positions[i]
            p2 = current_positions[j]
            comm_lines[k].set_data([p1[0], p2[0]], [p1[1], p2[1]])
            
            # 通信强度可视化: 距离近变绿，距离远变红
            dist = np.linalg.norm(p1 - p2)
            if dist > 3.0: 
                comm_lines[k].set_color((1, 0, 0, 0.6)) # Red
                comm_lines[k].set_linewidth(2.0)
            else:
                comm_lines[k].set_color((0, 0.8, 0, 0.4)) # Green
                comm_lines[k].set_linewidth(1.5)
        
        # 相机跟随
        all_x = [p[0] for p in current_positions]
        all_y = [p[1] for p in current_positions]
        cx, cy = np.mean(all_x), np.mean(all_y)
        ax_sim.set_xlim(cx - 7, cx + 7)
        ax_sim.set_ylim(cy - 6, cy + 6)
        ax_sim.set_title(f"Synchronized Convergence (Moving Forward) | Avg X: {cx:.1f}m")
        
        return patches + trails + texts + comm_lines

    anim = FuncAnimation(fig, update, frames=len(traj_q[0])//3, interval=30, blit=False)
    plt.tight_layout()
    plt.show()