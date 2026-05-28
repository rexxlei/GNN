#ifndef GNN_CONTROLLER_H
#define GNN_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h> // 【新增】：引入数组消息类型的头文件
#include <tf/tf.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace gnn_control {

struct Config {
    int N = 5;
    double dt = 0.05;         // 控制周期 (20Hz)
    double L = 0.3;           // 前视距离
    double v_cruise = 0.08;   // 巡航速度
    double max_v = 0.24;      // 最高线速度限幅
    double max_w = 1.0;       // 最高角速度限幅
    
    // 【新增】：状态缩放因子。实车1.0m间距 / 仿真0.5m间距 = 2.0
    // 用于给 GNN 戴上“缩小眼镜”，防止误差过大导致 ReLU 瘫痪死区
    double scale_factor = 2.0; 
    
    // GNN 维度参数
    int dim_state = 2;        // [X轴误差(对齐), Y轴误差(间距)]
    int hidden_dim = 16;
    int dim_control = 2;
};

class GNNController {
public:
    GNNController(ros::NodeHandle& nh);
    ~GNNController();
    
    bool loadWeights(const std::string& h1_path, const std::string& h2_path, const std::string& k_path);
    void controlLoop(const ros::TimerEvent& event);

private:
    ros::NodeHandle nh_;
    Config cfg_;
    
    ros::Publisher err_pub_; // 【新增】：声明误差探针发布者
    
    std::vector<ros::Subscriber> subs_;
    std::vector<ros::Publisher> pubs_;
    
    // 状态缓存
    std::vector<Eigen::Vector2d> current_pos_;
    std::vector<double> current_yaw_;
    std::vector<bool> pose_received_;
    
    // 领航者状态
    bool initialized_;
    double virtual_leader_x_phys_; 
    double virtual_leader_y_phys_; 

    // GNN 拓扑与预转置权重
    Eigen::MatrixXd L_;
    Eigen::MatrixXd H1_T_; 
    Eigen::MatrixXd H2_T_; 
    std::vector<Eigen::Matrix2d> K_blocks_; 
    std::vector<Eigen::Vector2d> offsets_phys_; 

    void setupTopology();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int robot_id);
    bool loadBin(const std::string& path, Eigen::MatrixXd& mat, int rows, int cols);
};

} // namespace gnn_control

#endif // GNN_CONTROLLER_H