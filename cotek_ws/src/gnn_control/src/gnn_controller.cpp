#include "gnn_control/gnn_controller.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <std_msgs/Float64MultiArray.h>

namespace gnn_control {

GNNController::GNNController(ros::NodeHandle& nh) : nh_(nh), initialized_(false) {
    current_pos_.resize(cfg_.N, Eigen::Vector2d::Zero());
    current_yaw_.resize(cfg_.N, 0.0);
    pose_received_.resize(cfg_.N, false);
    
    setupTopology();

    std::string names[] = {"a", "b", "c", "d", "e"};
    subs_.resize(cfg_.N);
    pubs_.resize(cfg_.N);
    
    for(int i = 0; i < cfg_.N; ++i) {
        subs_[i] = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/vrpn_client_node/" + names[i] + "/pose", 10,
            [this, i](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                this->poseCallback(msg, i);
            }
        );
        pubs_[i] = nh_.advertise<geometry_msgs::Twist>("/" + names[i] + "/cmd_vel", 10);
    }
    
    // 新增：向外广播 GNN 内部误差的 Publisher
    err_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/gnn_errors", 10);

    ROS_INFO("GNN Controller Ready. Waiting for Mocap data...");
}

GNNController::~GNNController() {}

void GNNController::setupTopology() {
    // 保留原生的非对称拉普拉斯矩阵
    L_ = Eigen::MatrixXd::Zero(cfg_.N, cfg_.N);
    L_ <<  1.0,  0.0, -1.0,  0.0,  0.0,
           0.0,  1.0, -1.0,  0.0,  0.0,
          -1.0, -1.0,  3.0, -1.0,  0.0,
           0.0,  0.0, -1.0,  2.0, -1.0,
           0.0,  0.0,  0.0, -1.0,  1.0;

    // 物理 Y 轴偏置，间距 1.0 米。A在最左(-2.0)，E在最右(+2.0)
    offsets_phys_.push_back(Eigen::Vector2d(0.0, -2.0)); // A车
    offsets_phys_.push_back(Eigen::Vector2d(0.0, -1.0)); // B车
    offsets_phys_.push_back(Eigen::Vector2d(0.0,  0.0)); // C车 (中心)
    offsets_phys_.push_back(Eigen::Vector2d(0.0,  1.0)); // D车
    offsets_phys_.push_back(Eigen::Vector2d(0.0,  2.0)); // E车
}

bool GNNController::loadBin(const std::string& path, Eigen::MatrixXd& mat, int rows, int cols) {
    std::ifstream file(path, std::ios::binary);
    if(!file) {
        ROS_ERROR("Failed to open weights file: %s", path.c_str());
        return false;
    }
    mat.resize(rows, cols);
    std::vector<float> buffer(rows * cols);
    file.read(reinterpret_cast<char*>(buffer.data()), rows * cols * sizeof(float));
    for(int i = 0; i < rows; ++i) {
        for(int j = 0; j < cols; ++j) {
            mat(i, j) = static_cast<double>(buffer[i * cols + j]);
        }
    }
    return true;
}

bool GNNController::loadWeights(const std::string& h1_path, const std::string& h2_path, const std::string& k_path) {
    bool ok = true;
    Eigen::MatrixXd H1_raw, H2_raw, K_raw;
    
    ok &= loadBin(h1_path, H1_raw, 16, 2);
    ok &= loadBin(h2_path, H2_raw, 2, 16);
    ok &= loadBin(k_path, K_raw, 10, 10);

    if (ok) {
        H1_T_ = H1_raw.transpose(); 
        H2_T_ = H2_raw.transpose(); 
        
        K_blocks_.resize(cfg_.N);
        for(int i = 0; i < cfg_.N; ++i) {
            K_blocks_[i] = K_raw.block<2,2>(i*2, i*2).transpose();
        }
        ROS_INFO("\033[1;32m[SUCCESS] GNN Weights loaded and optimized!\033[0m");
    }
    return ok;
}

void GNNController::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int id) {
    current_pos_[id] << (msg->pose.position.x / 100.0), (msg->pose.position.y / 100.0);
    
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, 
                     msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_[id] = yaw;
    
    pose_received_[id] = true;
}

void GNNController::controlLoop(const ros::TimerEvent& event) {
    bool all_ready = true;
    for(int i = 0; i < cfg_.N; ++i) {
        if(!pose_received_[i]) all_ready = false;
    }
    if(!all_ready) return;

    if(!initialized_) {
        double sum_x = 0;
        for(int i = 0; i < cfg_.N; ++i) {
            sum_x += current_pos_[i].x();
        }
        virtual_leader_x_phys_ = sum_x / cfg_.N;
        virtual_leader_y_phys_ = current_pos_[2].y(); 
        
        initialized_ = true;
        ROS_INFO("\033[1;36mFormation Initialized! Y anchored on C car. Pushing towards -X...\033[0m");
    }

    virtual_leader_x_phys_ -= cfg_.v_cruise * cfg_.dt;

    Eigen::MatrixXd X_state(cfg_.N, cfg_.dim_state);
    double total_error = 0.0;

    for(int i = 0; i < cfg_.N; ++i) {
        double th = current_yaw_[i];
        double x_front_phys = current_pos_[i].x() + cfg_.L * std::cos(th);
        double y_front_phys = current_pos_[i].y() + cfg_.L * std::sin(th);

        double target_x_phys = virtual_leader_x_phys_ + offsets_phys_[i].x(); 
        double target_y_phys = virtual_leader_y_phys_ + offsets_phys_[i].y(); 

        double err_x_phys = x_front_phys - target_x_phys;
        double err_y_phys = y_front_phys - target_y_phys;
        
        // 【核心修复 1：输入降维欺骗】
        // 强制把真实世界的庞大误差除以缩放因子，让 GNN 以为处于 0.5m 间距的小误差安全舒适区！
        X_state(i, 0) = err_x_phys / cfg_.scale_factor; 
        X_state(i, 1) = err_y_phys / cfg_.scale_factor; 
        
        total_error += std::sqrt(err_x_phys * err_x_phys + err_y_phys * err_y_phys);
    }

    double mean_pos_error = total_error / cfg_.N;
    ROS_INFO_THROTTLE(1.0, "[Monitor] Mean Pos Error: %.4f m", mean_pos_error);

    // =====================================================================
    // 【新增：数据探针】打包真实误差数据，广播给独立的数据记录节点
    // =====================================================================
    std_msgs::Float64MultiArray err_msg;
    err_msg.data.push_back(mean_pos_error); // 第 0 位放入全队的平均物理误差
    for(int i = 0; i < cfg_.N; ++i) {
        // 乘以 scale_factor 还原为最真实的物理误差，确保画出的论文图表精准无误
        err_msg.data.push_back(X_state(i, 0) * cfg_.scale_factor); 
        err_msg.data.push_back(X_state(i, 1) * cfg_.scale_factor); 
    }
    err_pub_.publish(err_msg); // 广播出去，记录节点会在后台静默接收
    // =====================================================================

    Eigen::MatrixXd LX = L_ * X_state;         
    Eigen::MatrixXd Z1 = (LX * H1_T_).cwiseMax(0.0); 
    Eigen::MatrixXd LZ1 = L_ * Z1;             
    Eigen::MatrixXd Z2 = (LZ1 * H2_T_).cwiseMax(0.0); 

    Eigen::MatrixXd U(cfg_.N, 2);
    for(int i = 0; i < cfg_.N; ++i) {
        U.row(i) = Z2.row(i) * K_blocks_[i]; 
    }

    for(int i = 0; i < cfg_.N; ++i) {
        double Vx_phys = U(i, 0) - cfg_.v_cruise; 
        double Vy_phys = U(i, 1); 
        
        double th = current_yaw_[i];

        double v = Vx_phys * std::cos(th) + Vy_phys * std::sin(th);
        double w = (-Vx_phys * std::sin(th) + Vy_phys * std::cos(th)) / cfg_.L;

        // 【核心修复 2：防倒车 + 等比例曲率限幅保护】
        if (v < 0.0) {
            v = 0.0;
            w = 0.0;
        } else {
            // 计算 v 和 w 各自超标了多少倍
            double v_ratio = std::abs(v) / cfg_.max_v;
            double w_ratio = std::abs(w) / cfg_.max_w;
            // 找出最严重的那个超标倍数
            double max_ratio = std::max(v_ratio, w_ratio);

            // 如果有人超标了，我们就把 v 和 w 同时除以这个倍数！
            // 这样能极其完美地保持小车的“转弯半径/曲率”不被破坏！
            if (max_ratio > 1.0) {
                v = v / max_ratio;
                w = w / max_ratio;
            }
        }

        geometry_msgs::Twist cmd;
        cmd.linear.x = v;  
        cmd.linear.y = 0.0;
        cmd.angular.z = w; 
        
        pubs_[i].publish(cmd);
    }
}

} // namespace gnn_control

int main(int argc, char** argv) {
    ros::init(argc, argv, "gnn_control_node");
    ros::NodeHandle nh("~");

    gnn_control::GNNController controller(nh);

    std::string path_h1 = "/home/cotek/GNN/cotek_ws/src/gnn_control/scripts/H1.bin";
    std::string path_h2 = "/home/cotek/GNN/cotek_ws/src/gnn_control/scripts/H2.bin";
    std::string path_k  = "/home/cotek/GNN/cotek_ws/src/gnn_control/scripts/K.bin";

    if(!controller.loadWeights(path_h1, path_h2, path_k)) {
        ROS_ERROR("Node shuts down because weights are missing.");
        return -1;
    }

    ros::Timer timer = nh.createTimer(ros::Duration(0.05), &gnn_control::GNNController::controlLoop, &controller);

    ros::spin();
    return 0;
}