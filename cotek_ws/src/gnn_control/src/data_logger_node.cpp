#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>

class DataLogger {
public:
    DataLogger(ros::NodeHandle& nh) : nh_(nh), is_started_(false), dt_(0.05) {
        int N = 5;
        names_ = {"a", "b", "c", "d", "e"};
        
        // 缓存初始化
        curr_x_.assign(N, 0.0); curr_y_.assign(N, 0.0); curr_yaw_.assign(N, 0.0);
        cmd_v_.assign(N, 0.0); cmd_w_.assign(N, 0.0);
        err_x_.assign(N, 0.0); err_y_.assign(N, 0.0);
        mean_pos_error_ = 0.0;
        
        // 自动获取当前系统时间作为文件名的时间戳
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << "/home/cotek/GNN/cotek_ws/src/gnn_control/data/experiment_data_" 
            << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
        
        std::string file_path = oss.str();
        csv_file_.open(file_path);
        if (!csv_file_.is_open()) {
            ROS_ERROR("Failed to open CSV! Please create 'data' folder.");
            ros::shutdown();
        }
        ROS_INFO("Saving data to: %s", file_path.c_str());
        
        // 写入豪华版表头（包含论文需要的所有数据维度）
        csv_file_ << "Time,Mean_Error";
        for (const auto& name : names_) {
            csv_file_ << "," << name << "_x," << name << "_y," << name << "_yaw"
                      << "," << name << "_v_cmd," << name << "_w_cmd"
                      << "," << name << "_err_x," << name << "_err_y";
        }
        csv_file_ << "\n";
        
        // 订阅器设置
        pose_subs_.resize(N);
        cmd_subs_.resize(N);
        for (int i = 0; i < N; ++i) {
            pose_subs_[i] = nh_.subscribe<geometry_msgs::PoseStamped>(
                "/vrpn_client_node/" + names_[i] + "/pose", 10,
                [this, i](const geometry_msgs::PoseStamped::ConstPtr& msg) { this->poseCallback(msg, i); });
            cmd_subs_[i] = nh_.subscribe<geometry_msgs::Twist>(
                "/" + names_[i] + "/cmd_vel", 10,
                [this, i](const geometry_msgs::Twist::ConstPtr& msg) { this->cmdCallback(msg, i); });
        }
        err_sub_ = nh_.subscribe("/gnn_errors", 10, &DataLogger::errCallback, this);
        
        // 独立 20Hz 定时器
        timer_ = nh_.createTimer(ros::Duration(dt_), &DataLogger::timerCallback, this);
        ROS_INFO("\033[1;32m[Logger] Started! Recording strictly from Time=0.00s...\033[0m");
    }
    
    ~DataLogger() {
        if (csv_file_.is_open()) csv_file_.close();
    }

private:
    ros::NodeHandle nh_;
    std::vector<std::string> names_;
    std::vector<double> curr_x_, curr_y_, curr_yaw_, cmd_v_, cmd_w_, err_x_, err_y_;
    double mean_pos_error_;
    
    std::vector<ros::Subscriber> pose_subs_, cmd_subs_;
    ros::Subscriber err_sub_;
    ros::Timer timer_;
    std::ofstream csv_file_;
    ros::Time start_time_;
    bool is_started_;
    double dt_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int id) {
        curr_x_[id] = msg->pose.position.x / 100.0;
        curr_y_[id] = msg->pose.position.y / 100.0;
        tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        curr_yaw_[id] = yaw;
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg, int id) {
        cmd_v_[id] = msg->linear.x;
        cmd_w_[id] = msg->angular.z;
    }

    void errCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        if (msg->data.empty()) return;
        mean_pos_error_ = msg->data[0];
        for(int i = 0; i < names_.size(); ++i) {
            err_x_[i] = msg->data[1 + i*2];
            err_y_[i] = msg->data[2 + i*2];
        }
    }

    void timerCallback(const ros::TimerEvent& event) {
        if (!is_started_) {
            start_time_ = ros::Time::now(); // 节点一运行立刻确立 T=0
            is_started_ = true;
        }
        
        double relative_time = (ros::Time::now() - start_time_).toSec();
        csv_file_ << relative_time << "," << mean_pos_error_;
        
        for (int i = 0; i < names_.size(); ++i) {
            csv_file_ << "," << curr_x_[i] << "," << curr_y_[i] << "," << curr_yaw_[i]
                      << "," << cmd_v_[i] << "," << cmd_w_[i]
                      << "," << err_x_[i] << "," << err_y_[i];
        }
        csv_file_ << "\n";
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "data_logger_node");
    ros::NodeHandle nh("~");
    DataLogger logger(nh);
    ros::spin();
    return 0;
}