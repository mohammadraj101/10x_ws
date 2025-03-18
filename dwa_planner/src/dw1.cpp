#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std;

const double ALPHA = 0.5;
const double BETA = 1.5;
const double GAMMA = 0.5;
const double SAFETY_DISTANCE = 0.35;

const double V_MIN = 0.0, V_MAX = 0.5;
const double W_MIN = -1.5, W_MAX = 1.5;
const double A_MIN = -0.5, A_MAX = 0.5;
const double AL_MIN = -0.4, AL_MAX = 0.4;
const double V_RES = 0.1, W_RES = 0.1;
const double T = 5.0, DT = 0.05;

class DwaPlanner : public rclcpp::Node {
public:
    DwaPlanner() 
        : Node("dwa_planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {

        // Subscriptions
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DwaPlanner::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DwaPlanner::odom_callback, this, std::placeholders::_1));
        
        // Publishers
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_trajectories1", 10);

        // Timer for DWA updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), std::bind(&DwaPlanner::apply_DWA, this));

        // Goal state
        x_goal = 5.0;
        y_goal = 3.0;
        goal_theta = 0.0;

        curr_vx = 0.0;
        curr_w = 0.0;
        pose_x = 0.0;
        pose_y = 0.0;
        pose_yaw = 0.0;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<std::pair<double, double>> obstacles;

    double x_goal, y_goal, goal_theta;
    double curr_vx, curr_w;
    double pose_x, pose_y, pose_yaw;

    // === Sigmoid function ===
    double sigmoid(double x, double k = 1.0, double c = 0.0) {
        return 1.0 / (1.0 + exp(-k * (x - c)));
    }

    // === Obstacle callback ===
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        obstacles.clear();
        double angle = msg->angle_min;
        for (auto r : msg->ranges) {
            if (msg->range_min < r && r < msg->range_max) {
                double x = r * cos(angle);
                double y = r * sin(angle);
                obstacles.emplace_back(x, y);
            }
            angle += msg->angle_increment;
        }
    }

    // === Odom callback ===
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_x = msg->pose.pose.position.x;
        pose_y = msg->pose.pose.position.y;
    
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
    
        tf2::Matrix3x3 m(q);
        
        // Create temporary roll and pitch values
        double roll, pitch;
        m.getRPY(roll, pitch, pose_yaw);
    
        curr_vx = msg->twist.twist.linear.x;
        curr_w = msg->twist.twist.angular.z;
    }
    
    

    // === Check Collision ===
    bool check_collision(double x, double y) {
        for (auto &obs : obstacles) {
            double dist = sqrt(pow(obs.first - x, 2) + pow(obs.second - y, 2));
            if (dist < SAFETY_DISTANCE) return true;
        }
        return false;
    }

    // === Sample Velocities ===
    std::vector<std::pair<double, double>> sample_velocities() {
        std::vector<std::pair<double, double>> velocities;
    
        // Adjusted velocity sampling based on acceleration limits
        double sampled_v_min = std::max(curr_vx + A_MIN * T, V_MIN);
        double sampled_v_max = std::min(curr_vx + A_MAX * T, V_MAX);
        double sampled_w_min = std::max(curr_w + AL_MIN * T, W_MIN);
        double sampled_w_max = std::min(curr_w + AL_MAX * T, W_MAX);
    
        int v_n = static_cast<int>((V_MAX - V_MIN) / V_RES) + 1;
        int w_n = static_cast<int>((W_MAX - W_MIN) / W_RES) + 1;
    
        for (int i = 0; i < v_n; ++i) 
        {
            double v = sampled_v_min + i * (sampled_v_max - sampled_v_min) / (v_n - 1);
            for (int j = 0; j < w_n; ++j) 
            {
                double w = sampled_w_min + j * (sampled_w_max - sampled_w_min) / (w_n - 1);
                velocities.emplace_back(v, w);
            }
        }
    
        return velocities;
    }
    

    // === Generate Trajectories ===
    std::vector<std::vector<std::tuple<double, double, double>>> generate_trajectories(
        const std::vector<std::pair<double, double>>& velocities) {

        std::vector<std::vector<std::tuple<double, double, double>>> trajectories;

        for (auto &[v, w] : velocities) {
            std::vector<std::tuple<double, double, double>> traj;
            double x = pose_x;
            double y = pose_y;
            double theta = pose_yaw;

            for (double t = 0; t <= T; t += DT) {
                x += v * cos(theta) * DT;
                y += v * sin(theta) * DT;
                theta += w * DT;

                if (check_collision(x, y)) break;

                traj.emplace_back(x, y, theta);
            }

            trajectories.push_back(traj);
        }

        return trajectories;
    }

    // === Compute Cost ===
    double compute_cost(const std::vector<std::tuple<double, double, double>>& traj) {
        if (traj.empty()) return std::numeric_limits<double>::max();

        auto [xf, yf, thf] = traj.back();

        // Distance Cost
        double cost_dist = sqrt(pow(x_goal - xf, 2) + pow(y_goal - yf, 2));

        // Heading Cost
        double heading_error = atan2(y_goal - yf, x_goal - xf) - thf;
        heading_error = atan2(sin(heading_error), cos(heading_error));
        double cost_heading = abs(heading_error);

        // Obstacle Cost
        double min_obstacle_dist = std::numeric_limits<double>::max();
        for (auto &[ox, oy] : obstacles) {
            double dist = sqrt(pow(ox - xf, 2) + pow(oy - yf, 2));
            if (dist < min_obstacle_dist) min_obstacle_dist = dist;
        }
        double cost_obstacle = 1.0 / (min_obstacle_dist + 0.01);

        return ALPHA * cost_dist + BETA * cost_obstacle + GAMMA * cost_heading;
    }

    // === Apply DWA ===
    void apply_DWA() 
    {
        auto start_time = this->get_clock()->now();
        auto velocities = sample_velocities();
        auto trajectories = generate_trajectories(velocities);

        double best_cost = std::numeric_limits<double>::max();
        double best_v = 0, best_w = 0;

        for (auto &traj : trajectories) {
            double cost = compute_cost(traj);
            if (cost < best_cost) {
                best_cost = cost;
                best_v = std::get<0>(traj[0]);
                best_w = std::get<1>(traj[0]);
            }
        }

        // publish_velocity(best_v, best_w);
        auto end_time = this->get_clock()->now();
        auto dur = end_time-start_time;
        RCLCPP_INFO(this->get_logger(), "Selected Velocities: v=%.3f, w=%.3f time_taken=%d", best_v, best_w,dur);
    }

    void publish_velocity(double v, double w) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = v;
        msg.angular.z = w;
        vel_pub_->publish(msg);
        
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DwaPlanner>());
    rclcpp::shutdown();
    return 0;
}
