#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "dwa_planner/srv/get_goal.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>

class DwaPlanner : public rclcpp::Node {
    public:
        DwaPlanner() : Node("dwa_planner") {
            // Publishers
            traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_full_path", 10);
    
            timer_ = this->create_wall_timer(100ms, std::bind(&DwaPlanner::compute_full_trajectory, this));
        }
    
    private:
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_pub_;
        std::vector<std::tuple<double, double, double>> global_trajectory;  // Store full path
    
        void compute_full_trajectory() {
            global_trajectory.clear();  // Reset before each computation
    
            double sim_time = 10.0;  // Simulate full trajectory for 10 seconds
            double dt = 0.1;  // Time step
    
            double x = pose_x, y = pose_y, theta = pose_yaw;  // Start from current pose
            double v = curr_vx, w = curr_w;
    
            for (double t = 0; t < sim_time; t += dt) {
                // Run DWA to get the best velocity command
                auto best_traj = run_dwa_step(x, y, theta, v, w);
                
                if (best_traj.points.empty()) {
                    RCLCPP_WARN(this->get_logger(), "No valid trajectory found, stopping.");
                    break;
                }
    
                // Append best trajectory points to global path
                for (auto& point : best_traj.points) {
                    global_trajectory.push_back(point);
                }
    
                // Update pose for next iteration
                auto [next_x, next_y, next_theta] = best_traj.points.back();
                x = next_x;
                y = next_y;
                theta = next_theta;
                v = best_traj.v;
                w = best_traj.w;
    
                // Stop if close to goal
                if (sqrt(pow(x_goal - x, 2) + pow(y_goal - y, 2)) < 0.2) {
                    RCLCPP_INFO(this->get_logger(), "Goal reached!");
                    break;
                }
            }
    
            // Publish the full path
            publish_full_trajectory();
        }
    
        traj run_dwa_step(double x, double y, double theta, double v, double w) {
            // Generate velocity samples
            auto velocities = sample_velocities(v, w);
            auto trajectories = generate_trajectories(x, y, theta, velocities);
    
            int best_index = get_min_cost_index(trajectories);
            if (best_index == -1) {
                RCLCPP_WARN(this->get_logger(), "No valid trajectory found.");
                return traj{};
            }
    
            return trajectories[best_index];  // Return best trajectory
        }
    
        void publish_full_trajectory() {
            visualization_msgs::msg::MarkerArray marker_array;
            int marker_id = 0;
    
            visualization_msgs::msg::Marker clear_marker;
            clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(clear_marker);
    
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "dwa_full_path";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.scale.x = 0.03;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // Blue for planned path
            marker.color.a = 0.8;
    
            for (const auto& point : global_trajectory) {
                auto [x, y, theta] = point;
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = y;
                p.z = 0.0;
                marker.points.push_back(p);
            }
    
            marker_array.markers.push_back(marker);
            traj_pub_->publish(marker_array);
        }
    };
    