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
#include <mutex> 

using namespace std::chrono_literals;

/////////////////////////////////////////////////////////////////////////////////////////// Parameters //////////////////

const double ALPHA = 0.1;
const double BETA = 0.9;
const double GAMMA = 0.0;
const double SAFETY_DISTANCE = 0.4;

const double V_MIN =  0.2, V_MAX = 0.7;
const double W_MIN = -2.0, W_MAX = 2.0;
const double A_MIN = -2.5, A_MAX = 2.5;
const double AL_MIN = -4.0, AL_MAX = 4.0;
const double V_RES = 0.2, W_RES = 0.5;
const double T = 3.0 , DT = 0.05 ,DP=0.33;
const int FI = static_cast<int>(DP/DT);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
                callback_group_4_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                
                rclcpp::SubscriptionOptions c1;
                c1.callback_group=callback_group_1_;

                scan_sub_ = this-> create_subscription<sensor_msgs::msg::LaserScan>(
                    "/scan", 10, std::bind(&DwaPlanner::scan_callback, this, std::placeholders::_1),c1);
                
                rclcpp::SubscriptionOptions c2;
                c2.callback_group = callback_group_2_;
                odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", 10,std::bind(&DwaPlanner::odom_callback, this, std::placeholders::_1),c2);
                
                goal_srv_ = this->create_service<dwa_planner::srv::GetGoal>(
                    "get_goal",
                    std::bind(&DwaPlanner::goal_service_callback, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,callback_group_3_);

                vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
                marker_pub_ = this-> create_publisher<visualization_msgs::msg::MarkerArray>("/dwa",10);

                timer_ = this->create_wall_timer(33.33ms, std::bind(&DwaPlanner::apply_DWA,this),callback_group_4_);

                ////////////////////
                // scan_sub_.subscribe(this, "/scan");
                // odom_sub_.subscribe(this, "/odom");
                // sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), scan_sub_, odom_sub_);
                // sync_->registerCallback(std::bind(&DWAPlanner::callback, this, std::placeholders::_1, std::placeholders::_2));

                ///////////////////


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

        rclcpp::CallbackGroup::SharedPtr callback_group_1_;
        rclcpp::CallbackGroup::SharedPtr callback_group_2_;
        rclcpp::CallbackGroup::SharedPtr callback_group_3_;
        rclcpp::CallbackGroup::SharedPtr callback_group_4_;
        rclcpp::Service<dwa_planner::srv::GetGoal>::SharedPtr goal_srv_;

        // using MySyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry>;
        // message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
        // message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        // std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
        // std::mutex data_mutex_;
        std::mutex odom_mutex, scan_mutex, obstacle_mutex, vel_mutex;




        sensor_msgs::msg::LaserScan::SharedPtr laser_msg;
        std::vector<std::pair<double,double>> obstacles;
        double x_goal, y_goal, goal_theta;
        double curr_vx, curr_w;
        double pose_x, pose_y, pose_yaw;
        bool flag=false;
        bool flag_planning= false;

        double vf;
        double wf;

        double sigmoid(double x, double k = 1.0, double c = 0.0) {
            return 1.0 / (1.0 + exp(-k * (x - c)));
        }

        // void callback(const sensor_msgs::msg::LaserScan::SharedPtr scan, const nav_msgs::msg::Odometry::SharedPtr odom) {
        //     std::lock_guard<std::mutex> lock(data_mutex_);
        //     RCLCPP_INFO(this->get_logger(), "Received synced /scan and /odom data");
        //     // Process odom and scan together safely
        // }

    // private:
        
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(scan_mutex);
            laser_msg=msg;
        }

        std::vector<std::pair<double, double>> get_obstacles(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(obstacle_mutex);

            std::vector<std::pair<double, double>> obst;
            double angle = msg->angle_min;
            for (auto r: msg->ranges)
            {
                if(!std::isnan(r) && msg->range_min<r && r<msg->range_max)
                {
                    double x= pose_x + r*cos(angle);
                    double y= pose_y + r*sin(angle);

                    obst.emplace_back(x , y );
                }
                angle += msg->angle_increment;
            }
            return obst;
        }

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(odom_mutex);

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

        double dist_obst(double x,double y)     // will return the min distance from the obstacle  of any x,y, cordinate
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

        bool check_collision(double x,double y)     // to check whether a trajactory can lead to collisiom
        {
            for (auto &obs : obstacles)
            {
                double dist= sqrt((obs.first - x)*(obs.first - x)+(obs.second - y)*(obs.second - y));
                if (dist< SAFETY_DISTANCE)
                {
                    return true;
                }
            }
            return false;
        }

        std::vector<std::pair<double,double>> sample_velocities()  //to sample velocities for DWA Approach
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

        std::vector<traj>  generate_trajectories(const std::vector<std::pair<double, double>>& velocities)  // will do all the computation of enerating the trajectories
        {
            std::vector<traj> trajectories;
            obstacles=get_obstacles(laser_msg);
            flag_planning=true;

            
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
                    }
                    t.points.emplace_back(x,y,th);
                }

                if(collision==false)
                {
                    t.cost=generate_cost(t);
                    trajectories.emplace_back(t);
                }

            }
            return trajectories;
            flag_planning =false;

        }
        

        void apply_DWA()
        {
            auto start_time = this->get_clock()->now();

            if(flag==false)
            {
                std::lock_guard<std::mutex> lock(vel_mutex);
                publish_velocities(0.0, 0.0);
                return;
            }
            
                // std::lock_guard<std::mutex> lock(obstacle_mutex);
                auto velocities = sample_velocities();
                auto trajectories = generate_trajectories(velocities);
            

            int best_index = get_min_cost_index(trajectories);
            if (best_index == -1)   ///////////////////////////////////////////////////////////////////////////// Recovery Behaviour //////////////////////// 
            {
                RCLCPP_WARN(this->get_logger(), "No valid trajectory found");

                    publish_velocities(0.0,0.0);
                    rclcpp::sleep_for(10ms);
                
            }                      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            auto calc_time =  this-> get_clock()->now()-start_time;
            auto calc_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::nanoseconds(calc_time.nanoseconds())).count();
            
            RCLCPP_INFO(this->get_logger(), "Time taken for calculations: %ld ms", calc_ms);

            if(best_index>=0)
            {
                vf=trajectories[best_index].v;
                wf=trajectories[best_index].w;

                // vf=0.1;
                // wf=0.0;


                auto prev_time  =this->get_clock()->now();
                auto cur_time  =this->get_clock()->now();
                rclcpp::Duration loop_duration = rclcpp::Duration::from_seconds(DP);

                double goal_dist = sqrt(pow(x_goal - pose_x, 2) + pow(y_goal - pose_y, 2));
                if (goal_dist < 0.05) 
                {
                    RCLCPP_INFO(this->get_logger(), "Goal reached! Stopping robot..");
                    publish_velocities(0.0, 0.0);
                    vf=0.0;
                    wf=0.0;
                    return;
                }
                auto time_offset= rclcpp::Duration::from_seconds(0.2);

                while(cur_time - prev_time < loop_duration && flag==true)
                {
                    publish_markers(trajectories,best_index);
                    publish_velocities(vf,wf);                                  //comment out to publish velocities


                    if (goal_dist < 0.05) // 0.1 meters threshold
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
                

                RCLCPP_INFO(this->get_logger(), "Current Pose: x: %.3f, y: %.3f,   Min dist traj : %.4f ", 
                pose_x, pose_y,3.5*(1-trajectories[best_index].cost));
                std::cout<<"traj 1 dist "<<3.5*(1-trajectories[0].cost)<<"Ang vel :"<<trajectories[0].w<<"\n";
                std::cout<<"traj 2 dist "<<3.5*(1-trajectories[1].cost)<<"Ang vel :"<<trajectories[1].w<<"\n";
                std::cout<<"traj 3 dist "<<3.5*(1-trajectories[2].cost)<<"Ang vel :"<<trajectories[2].w<<"\n";
                std::cout<<"traj 4 dist "<<3.5*(1-trajectories[3].cost)<<"Ang vel :"<<trajectories[3].w<<"\n";
                std::cout<<"traj 5 dist "<<3.5*(1-trajectories[4].cost)<<"Ang vel :"<<trajectories[4].w<<"\n";
                std::cout<<"traj 6 dist "<<3.5*(1-trajectories[3].cost)<<"Ang vel :"<<trajectories[5].w<<"\n";
                std::cout<<"traj 7 dist "<<3.5*(1-trajectories[4].cost)<<"Ang vel :"<<trajectories[6].w<<"\n";

                std::cout<<"\n";
                

           }//////////// for best index-1


        }

        double cost_obstacle(const traj&t)
        {

            double min_dist=3.6;
            double x,y,theta;
            if(obstacles.empty())
            {
                return 0;
            }
            // for (const auto& point : t.points) 
            // {
            //     std::tie(x, y, theta) = point;
            //     for (const auto& obs : obstacles) {
            //         double dist = hypot(obs.first - x, obs.second - y);
            //         if (dist < min_dist) {
            //             min_dist = dist;
            //         }
            //     }
            // }

            std::tie(x, y, theta) = t.points[FI];
            for (const auto& obs : obstacles) 
            {
                double dist = hypot(obs.first - x, obs.second - y);
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
            auto cost_ = (1-min_dist/3.5);
            return cost_;
        }

        double cost_heading_angle(const traj&t)
        {
            if(t.points.empty())
            {
                return 1.0;
            }
            auto [x, y, theta] = t.points[FI];
            double goal_angle = atan2(y_goal - y, x_goal - x);
            double goal_dist= sqrt(pow(x_goal - x, 2) + pow(y_goal - y, 2));
            double heading_error = fabs(goal_angle - theta);
            heading_error = fmod(heading_error, 2 * M_PI);
            if (heading_error > 3.14) 
            {
                heading_error = 2 * 3.14 - heading_error;
            }


            return heading_error/3.14 ; //* goal_dist;
        }

        double cost_dist_left(const traj&t)
        {
            // if(t.points.empty())
            // {
            //     return 1.0;
            // }
            auto [x, y, theta] = t.points[FI];
           
            double goal_dist= sqrt(pow(x_goal - x, 2) + pow(y_goal - y, 2));

            auto cost= 1.0 / (1.0 + exp(4.0-goal_dist));
            return cost;
        }

        double generate_cost(const traj&t)
        {
            double head_cost =  cost_heading_angle(t);
            double obstacle_cost = cost_obstacle(t);
            double dist_cost = cost_dist_left(t);
            double a,b;
            a=ALPHA;
            b=BETA;

            // if (3.5*(1-obstacle_cost)<0.7)
            // {
            //     a=0.5*ALPHA;
            //     b=1-a;
            // }
            
            // double cost= 0*head_cost +1*obstacle_cost;
            
            double cost=obstacle_cost;

 

            return cost;

        }
        
        void publish_velocities(double vf, double wf)
        {
            // std::lock_guard<std::mutex> lock(vel_mutex);
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
                marker.ns = "dwa";
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
        
        

        ///////////////////////////////////////////////////////////////
        
        //////////////////////////////////////////////////////////////
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DwaPlanner>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "DWA Node started with MultiThreadedExecutor");
    
    executor.spin();

    rclcpp::shutdown();
    return 0;
}





