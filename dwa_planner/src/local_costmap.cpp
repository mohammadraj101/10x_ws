#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>

class LocalCostmapNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    double costmap_size_ = 7.0;  // Local costmap size in meters
    double resolution_ = 0.05;   // Grid resolution (meters per cell)
    double inflation_radius_ = 0.3; // Inflation radius (meters)
    int grid_size_;              // Grid size in cells
    int inflation_cells_;        // Inflation radius in grid cells

    nav_msgs::msg::OccupancyGrid costmap_;
    geometry_msgs::msg::Pose robot_pose_;

public:
    LocalCostmapNode() : Node("local_costmap_node")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LocalCostmapNode::scanCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LocalCostmapNode::odomCallback, this, std::placeholders::_1));

        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 10);

        // Calculate grid size
        grid_size_ = static_cast<int>(costmap_size_ / resolution_);
        inflation_cells_ = static_cast<int>(inflation_radius_ / resolution_);

        // Initialize costmap message
        costmap_.header.frame_id = "odom";  // Local costmap in odometry frame
        costmap_.info.resolution = resolution_;
        costmap_.info.width = grid_size_;
        costmap_.info.height = grid_size_;
        costmap_.data.resize(grid_size_ * grid_size_, 0);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_pose_ = msg->pose.pose;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Reset costmap
        std::fill(costmap_.data.begin(), costmap_.data.end(), 0);

        std::vector<std::pair<int, int>> obstacle_cells;

        // Convert scan data to grid coordinates
        for (size_t i = 0; i < scan->ranges.size(); i++)
        {
            float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max)
                continue;

            float angle = scan->angle_min + i * scan->angle_increment;
            float obstacle_x = robot_pose_.position.x + range * cos(angle);
            float obstacle_y = robot_pose_.position.y + range * sin(angle);

            int map_x = static_cast<int>((obstacle_x - robot_pose_.position.x + costmap_size_ / 2) / resolution_);
            int map_y = static_cast<int>((obstacle_y - robot_pose_.position.y + costmap_size_ / 2) / resolution_);

            if (map_x >= 0 && map_x < grid_size_ && map_y >= 0 && map_y < grid_size_)
            {
                int index = map_y * grid_size_ + map_x;
                costmap_.data[index] = 100;  // Mark as obstacle
                obstacle_cells.push_back({map_x, map_y});
            }
        }

        // Apply inflation to obstacle cells
        applyInflation(obstacle_cells);

        // Publish costmap
        costmap_.header.stamp = this->get_clock()->now();
        costmap_pub_->publish(costmap_);
    }

    void applyInflation(const std::vector<std::pair<int, int>>& obstacle_cells)
    {
        for (const auto& obs : obstacle_cells)
        {
            int obs_x = obs.first;
            int obs_y = obs.second;

            for (int dx = -inflation_cells_; dx <= inflation_cells_; dx++)
            {
                for (int dy = -inflation_cells_; dy <= inflation_cells_; dy++)
                {
                    int nx = obs_x + dx;
                    int ny = obs_y + dy;

                    if (nx >= 0 && nx < grid_size_ && ny >= 0 && ny < grid_size_)
                    {
                        double distance = std::sqrt(dx * dx + dy * dy) * resolution_;
                        if (distance <= inflation_radius_)
                        {
                            int index = ny * grid_size_ + nx;
                            int inflation_value = static_cast<int>(100 * (1.0 - (distance / inflation_radius_)));
                            if (costmap_.data[index] < inflation_value)
                            {
                                costmap_.data[index] = inflation_value;
                            }
                        }
                    }
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalCostmapNode>());
    rclcpp::shutdown();
    return 0;
}
