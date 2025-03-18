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

using namespace std::chrono_literals;

const double ALPHA = 1.5;
const double BETA = 2.0;
const double GAMMA = 0.0;
const double SAFETY_DISTANCE = 0.3;

const double V_MIN =  0.1, V_MAX = 0.8;
const double W_MIN = -1.8, W_MAX = 1.8;
const double A_MIN = -0.5, A_MAX = 0.4;
const double AL_MIN = -2.5, AL_MAX = 2.5;
const double V_RES = 0.05, W_RES = 0.05;
const double T = 2.0 , DT = 0.05;

struct traj {
    double v;
    double w;
    double cost;
    std::vector<std::tuple<double, double, double>> points;
};


class DwaPlanner : public rclcpp::Node{

    public:
        DwaPlanner()
            : Node("dwa_planner")
            {

                callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                callback_group_3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                // callback_group_4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                
                rclcpp::SubscriptionOptions c1;
                c1.callback_group=callback_group_1_;

                scan_sub_ = this-> create_subscription<sensor_msgs::msg::LaserScan>(
                    "/scan", 10, std::bind(&DwaPlanner::scan_callback, this, std::placeholders::_1),c1);
                
                rclcpp::SubscriptionOptions c2;
                c2.callback_group = callback_group_2_;
                odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", 10,std::bind(&DwaPlanner::odom_callback, this, std::placeholders::_1),c2);

                // rclcpp::ServiceOptions c3;
                // c3.callback_group = callback_group_3_;
                
                goal_srv_ = this->create_service<dwa_planner::srv::GetGoal>(
                    "get_goal",
                    std::bind(&DwaPlanner::goal_service_callback, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,callback_group_3_);

                vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
                marker_pub_ = this-> create_publisher<visualization_msgs::msg::MarkerArray>("/dwa_trajectory",10);

                timer_ = this->create_wall_timer(50ms, std::bind(&DwaPlanner::apply_DWA,this));
                // vel_timer_ = this->create_wall_timer(100ms, std::bind(&DwaPlanner::publish_velocities,this,vf,wf));
                // vel_timer_ = this->create_wall_timer(
                //     100ms, 
                //     [this]() { this->publish_velocities(vf, wf); }
                // );

                x_goal=5.0;
                y_goal=2.0;

                vf=0.0;
                wf=0.0;
                

                pose_x=0.0;
                pose_y=0.0;
                pose_yaw=0.0;
                

                curr_vx = 0.0;
                curr_w = 0.0;

            }
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        // rclcpp::TimerBase::SharedPtr vel_timer_;

        rclcpp::CallbackGroup::SharedPtr callback_group_1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_2_;
        rclcpp::CallbackGroup::SharedPtr callback_group_3_;
        rclcpp::Service<dwa_planner::srv::GetGoal>::SharedPtr goal_srv_;




        std::vector<std::pair<double,double>> obstacles;
        double x_goal, y_goal, goal_theta;
        double curr_vx, curr_w;
        double pose_x, pose_y, pose_yaw;
        bool flag=false;

        double vf;
        double wf;

        double sigmoid(double x, double k = 1.0, double c = 0.0) {
            return 1.0 / (1.0 + exp(-k * (x - c)));
        }

    // private:
        
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            obstacles.clear();
            double angle = msg->angle_min;
            for (auto r: msg->ranges)
            {
                if(!std::isnan(r) && msg->range_min<r && r<msg->range_max)
                {
                    double x= pose_x + r*cos(angle);
                    double y= pose_y + r*sin(angle);

                    obstacles.emplace_back(x , y );
                }
                angle += msg->angle_increment;
            }
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            pose_x =msg->pose.pose.position.x;
            pose_y =msg->pose.pose.position.y;
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );

            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, pose_yaw);

            // RCLCPP_INFO(this->get_logger(), "%f %f",pose_x, pose_y);
        
            curr_vx = msg->twist.twist.linear.x;
            curr_w = msg->twist.twist.angular.z;
        }

        void goal_service_callback(const std::shared_ptr<dwa_planner::srv::GetGoal::Request> request,
            std::shared_ptr<dwa_planner::srv::GetGoal::Response> response)
        {
            x_goal = request->x;
            y_goal = request->y;
            flag=true;

            RCLCPP_INFO(this->get_logger(), "New Goal Set: x = %.3f, y = %.3f", x_goal, y_goal);
            response->success = true;
        }

        double dist_obst(double x,double y)
        {
            double min_dist=100;
            for (auto &obs : obstacles)
            {
                double dist= sqrt((obs.first - x)*(obs.first - x)+(obs.second - y)*(obs.second - y));
                if (dist< min_dist)
                {
                    min_dist=dist;
                }
            }
            return min_dist;
        }

        bool check_collision(double x,double y)
        {
            for (auto &obs : obstacles)
            {
                double dist= sqrt((obs.first - x)*(obs.first - x)+(obs.second - y)*(obs.second - y));
                if (dist< 0.25)
                {
                    return true;
                }
            }
            return false;
        }

        bool check_collision_back(double x, double y, double theta)
        {
            const double BACKWARD_DISTANCE = 0.4;  // How far behind to check
            const double BACKWARD_ANGLE = 1.57; // 45 degrees backward spread
        
            for (auto &obs : obstacles)
            {
                double dist = sqrt((obs.first - x) * (obs.first - x) + (obs.second - y) * (obs.second - y));
                if (dist < BACKWARD_DISTANCE)
                {
                    double angle_to_obs = atan2(obs.second - y, obs.first - x);
                    double angle_diff = fmod(fabs(angle_to_obs - (theta + M_PI)), 2 * M_PI);
                    if (angle_diff > M_PI) 
                        angle_diff = 2 * M_PI - angle_diff;
        
                    if (angle_diff <= BACKWARD_ANGLE)  // Within the backward cone
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        std::vector<std::pair<double,double>> sample_velocities()
        {
            std::vector<std::pair<double, double>> velocities;
            double sampled_v_min = std::max(curr_vx + A_MIN * T, V_MIN);
            double sampled_v_max = std::min(curr_vx + A_MAX * T, V_MAX);
            double sampled_w_min = std::max(curr_w + AL_MIN * T, W_MIN);
            double sampled_w_max = std::min(curr_w + AL_MAX * T, W_MAX);

            int v_n= static_cast<int>((sampled_v_max-sampled_v_min)/V_RES) +1;
            int w_n = static_cast<int>((sampled_w_max - sampled_w_min) / W_RES) + 1;
            // std::cout<<v_n<<std::endl;

            for (int i=0;i<v_n;i++)
            {
                double v = sampled_v_min + i * V_RES;//(sampled_v_max - sampled_v_min) / (v_n - 1);
                for (int j = 0; j < w_n; ++j) 
                {
                    double w = sampled_w_min + j *W_RES; // (sampled_w_max - sampled_w_min) / (w_n - 1);
                    velocities.emplace_back(v, w);
                }
            }
            return velocities;
        }

        std::vector<traj>  generate_trajectories(const std::vector<std::pair<double, double>>& velocities)
        {
            std::vector<traj> trajectories;
            
            for(auto &[v,w] : velocities)
            {
                traj t;
                t.v=v;
                t.w=w;
                t.cost=0.0;

                double x= pose_x;
                double y= pose_y;
                double th= pose_yaw;

                bool collision=false;

                for(double k=0; k<T;k+=DT)
                {
                    x+= v*cos(th)*DT;
                    y+= v*sin(th)*DT;

                    th+=w*DT;

                    if (check_collision(x,y))
                    {
                        collision =true;
                        break;
                        // auto p=1+2;
                    }
                    t.points.emplace_back(x,y,th);
                }
                t.cost=generate_cost(t);

                if(collision==false)
                {
                    trajectories.emplace_back(t);
                }

            }
            return trajectories;

        }
        

        void apply_DWA()
        {
            auto start_time = this->get_clock()->now();

            auto velocities = sample_velocities();
            auto trajectories = generate_trajectories(velocities);

            int best_index = get_min_cost_index(trajectories);
            if (best_index == -1)   ///////////////////////////////////////////////////////////////////////////// Recovery Behaviour //////////////////////// 
            {
                RCLCPP_WARN(this->get_logger(), "No valid trajectory found");
                if (check_collision_back(pose_x,pose_y,pose_yaw))
                {
                    // publish_velocities(0.0, 0.08);
                    vf=0.0;
                    wf=0.0;
                }
                else
                {
                    vf=-0.0;
                    wf=0.0;
                }
                
            }                      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            auto calc_time =  this-> get_clock()->now()-start_time;
            auto calc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::nanoseconds(calc_time.nanoseconds())).count();
            
            RCLCPP_INFO(this->get_logger(), "Time taken for calculations: %ld ms", calc_ms);

            if(best_index>=0)
            {
                vf=trajectories[best_index].v;
                wf=trajectories[best_index].w;


                auto prev_time  =this->get_clock()->now();
                auto cur_time  =this->get_clock()->now();
                rclcpp::Duration loop_duration = rclcpp::Duration::from_seconds(T);

                double goal_dist = sqrt(pow(x_goal - pose_x, 2) + pow(y_goal - pose_y, 2));
                if (goal_dist < 0.10) // 0.1 meters threshold
                {
                    RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot..");
                    publish_velocities(0.0, 0.0);
                    vf=0.0;
                    wf=0.0;
                    return;
                }
                auto time_offset= rclcpp::Duration::from_seconds(2*calc_ms);

                while(cur_time - prev_time + time_offset < loop_duration && flag==true)
                {
                    publish_markers(trajectories,best_index);
                    publish_velocities(vf,wf);

                    if (goal_dist < 0.07) // 0.1 meters threshold
                    {
                        flag= false;
                        RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot. trigering while");
                        publish_velocities(0.0, 0.0);
                        vf=0.0;
                        wf=0.0;
                        break;
                    }
                    goal_dist=(pow(x_goal - pose_x, 2) + pow(y_goal - pose_y, 2));

                    cur_time  =this->get_clock()->now();
                }
                ///////////////////////////////////////////////////////////////////////////////////
                

                RCLCPP_INFO(this->get_logger(), "Current Pose: x: %.3f, y: %.3f, vx: %.3f, w: %.3f    COST: %.4f Distance left: %.4f ", 
                pose_x, pose_y, curr_vx, curr_w,trajectories[best_index].cost, goal_dist);

           }//////////// for best index-1






        //     /////////////////////////////////////////////////
        //     auto end_time = this->get_clock()->now();
        //     auto dur = end_time-start_time;
            // RCLCPP_INFO(this->get_logger(), "Selected Velocities: v=%.3f, w=%.3f time_taken=%d", best_v, best_w,dur);
        }

        double cost_obstacle(const traj&t)
        {
            // double cost = 0;
            double min_dist=10.0;
            double x,y,theta;
            if(obstacles.empty())
            {
                return 0;
            }
            if (t.points.empty()) 
            {
                // return 1;
                x= pose_x;
                y= pose_y;
                theta= pose_yaw;
                for (const auto& obs : obstacles) {
                    double dist = hypot(obs.first - x, obs.second - y);
                    if (dist < min_dist) {
                        min_dist = dist;
                    }
                }
            }
            else{
                for (const auto& point : t.points) {
                    auto [x, y, theta] = point;
                    for (const auto& obs : obstacles) {
                        double dist = hypot(obs.first - x, obs.second - y);
                        if (dist < min_dist) {
                            min_dist = dist;
                        }
                    }
                }
            }
            auto cost_= 1.0 / (1.0 + exp(6*(min_dist-0.4)));   //a=6 & b=0.4 try this also
            return cost_;
        }

        double cost_heading_angle(const traj&t)
        {
            if(t.points.empty())
            {
                return 1.0;
            }
            auto [x, y, theta] = t.points.back();
            double goal_angle = atan2(y_goal - y, x_goal - x);
            double goal_dist= sqrt(pow(x_goal - x, 2) + pow(y_goal - y, 2));
            double heading_error = fabs(goal_angle - theta);
            heading_error = fmod(heading_error, 2 * M_PI);
            if (heading_error > 3.14) 
            {
                heading_error = 2 * 3.14 - heading_error;
            }

            return heading_error/3.14; // * goal_dist;
        }

        double cost_dist_left(const traj&t)
        {
            if(t.points.empty())
            {
                return 1.0;
            }
            auto [x, y, theta] = t.points.back();
           
            double goal_dist= sqrt(pow(x_goal - x, 2) + pow(y_goal - y, 2));

            auto cost= 1.0 / (1.0 + exp(4.0-goal_dist));
            return cost;
        }

        double generate_cost(const traj&t)
        {
            double head_error =  cost_heading_angle(t);
            double obstacle_error = cost_obstacle(t);
            // double dist_error =cost_dist_left(t);

            // double cost = ALPHA * head_error + BETA * obstacle_error + GAMMA* dist_error;

            double a=1.0 / (1.0 + exp(10.0 * (obstacle_error - 0.1)));
            double b=1-a;

            // double cost = head_error;

            double cost = a * head_error + b * obstacle_error ;

            // RCLCPP_INFO(this->get_logger(), "Head Error: %.4f, Obstacle Error: %.4f, Dist Error: %.4f, Final Cost: %.4f", 
            //     head_error, obstacle_error, dist_error, cost);

            return cost;

        }
        
        void publish_velocities(double vf, double wf)
        {
            auto cmd =geometry_msgs::msg::Twist();
            cmd.linear.x=vf;
            cmd.angular.z=wf;
            vel_pub_->publish(cmd);

        }

        

        void publish_markers(const std::vector<traj>& trajectories, int best_index) 
        {
            visualization_msgs::msg::MarkerArray marker_array;
            int marker_id = 0;
        
            // Clear existing markers
            visualization_msgs::msg::Marker clear_marker;
            clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker_array.markers.push_back(clear_marker);
        
            for (size_t i = 0; i < trajectories.size(); ++i) 
            {
                const auto& t = trajectories[i];
        
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "odom";
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "dwa_trajectory";
                marker.id = marker_id++;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.scale.x = 0.02;
        
                // Set color based on best trajectory
                if (i == best_index) {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.9;
                } else {
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    marker.color.a = 0.7;
                }

                if(t.points.empty())
                {
                    RCLCPP_WARN(this->get_logger(),"No points to publish skipped");
                    return;
                }
        
                for (const auto& point : t.points) 
                {

                    auto [x, y, theta] = point;
                    geometry_msgs::msg::Point p;
                    // RCLCPP_INFO(this->get_logger(), "Point: x = %.2f, y = %.2f", x, y);
                    p.x = x;
                    p.y = y;
                    p.z = 0.0;
                    marker.points.push_back(p);
                }
        
                marker_array.markers.push_back(marker);
            }
            marker_pub_->publish(marker_array);
        }

        int get_min_cost_index(const std::vector<traj>& trajectories) {
            if (trajectories.empty()) return -1;
        
            int min_index = 0;
            double min_cost = trajectories[0].cost;
        
            for (size_t i = 1; i < trajectories.size(); ++i) {
                if (trajectories[i].cost < min_cost) {
                    min_cost = trajectories[i].cost;
                    min_index = i;
                }
            }
        
            return min_index;
        }
        
        double sigmoid_penalty(double cost, double sharpness, double offset) 
        {
            return 1.0 / (1.0 + exp(sharpness * (-cost)));
        }

        ///////////////////////////////////////////////////////////////
        
        //////////////////////////////////////////////////////////////
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DwaPlanner>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "DWA Node started with MultiThreadedExecutor");
    
    executor.spin();

    rclcpp::shutdown();
    return 0;
}




