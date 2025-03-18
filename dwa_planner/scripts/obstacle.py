#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import time

ALPHA =  0.5 #0.0 #0.3
BETA =   1.5 #-0.05 #3.0
GAMMA =  0.5 #0.5  #0.7

SAFETY_DISTANCE = 0.35

V_MIN, V_MAX = -0.0, 0.5
W_MIN, W_MAX = -1.5,1.5
A_MIN, A_MAX = -0.5, 0.5
AL_MIN, AL_MAX = -0.4, 0.4
V_RES, W_RES = 0.1, 0.1
T, DT = 1.5, 0.1

def sigmoid(x, k=1.0, c=0.0):
    return 1 / (1 + np.exp(-k * (x - c)))


class DwaPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        self.call_back_group_1 =MutuallyExclusiveCallbackGroup()
        self.call_back_group_2 =MutuallyExclusiveCallbackGroup()
        self.call_back_group_3 =MutuallyExclusiveCallbackGroup()
        self.call_back_group_4 =MutuallyExclusiveCallbackGroup()
        
        # Subscriptions
        self.scan_sub_ = self.create_subscription(LaserScan, '/scan', self.scan_callback,10, callback_group=self.call_back_group_1)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odometry_callback,10, callback_group=self.call_back_group_2)
        # self.pose_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)

        # Publishers
        self.vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        # Timer to update DWA
        self.timer_ = self.create_timer(0.2, self.apply_DWA,callback_group=self.call_back_group_3)
        
        # Goal state
        self.x_goal =  -1.0
        self.y_goal =  -1.0
        self.goal_theta = 0.0

        # Current state
        self.curr_vx = 0.0
        self.curr_w = 0.0
        self.pose_x = 4.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        # Obstacle array
        self.obstacles = np.empty((0, 2))

    # def scan_callback(self, msg):
    #     self.create_obstacle_array(msg)

    # def scan_callback(self, scan):
    #     self.obstacles = np.empty((0, 2))
    #     angle = scan.angle_min
    #     for r in scan.ranges:
    #         if scan.range_min < r < scan.range_max:
    #             x = r * np.cos(angle)
    #             y = r * np.sin(angle)
    #             self.obstacles = np.vstack((self.obstacles, [x, y]))
    #             # print(min(self.obstacles))
    #         angle = (angle + scan.angle_increment) % (2 * np.pi)
    def scan_callback(self, scan):
        num_points = len(scan.ranges)
        self.obstacles = np.zeros((num_points, 2))  # Preallocate array

        angle = scan.angle_min
        valid_count = 0

        for i in range(num_points):
            r = scan.ranges[i]
            if scan.range_min < r < scan.range_max:
                self.obstacles[valid_count] = [r * np.cos(angle), r * np.sin(angle)]
                valid_count += 1
            angle += scan.angle_increment
        
        # Resize array to valid size
        self.obstacles = self.obstacles[:valid_count]


    def check_collision(self, x, y):
        
        x=self.pose_x
        y=self.pose_y
        if len(self.obstacles) == 0:
            return False
            
        distances = np.sqrt((self.obstacles[:, 0] - x)**2 + (self.obstacles[:, 1] - y)**2)
        min_distance = np.min(distances)
        print(f"min_distance:  {min_distance}")
        if min_distance<SAFETY_DISTANCE:
            print("MC")
        # safety_distance = 2.5 # Increased safety distance to avoid clipping obstacles
        return min_distance < SAFETY_DISTANCE
    
    def find_cost_dist(self,traj):
        if traj is not None:
            v, w, traj_points = traj
            xf0,yf0, th0= traj_points[-1]

            cost= np.sqrt((self.x_goal-xf0)**2 + (self.y_goal-yf0)**2)
            return cost
        
    def find_cost_heading(self,traj):
        if traj is not None:
            v, w, traj_points = traj
            xf0,yf0, th0= traj_points[-1]

            head_angle = np.arctan2(self.y_goal - yf0, self.x_goal - xf0)

            # Difference from current yaw
            heading_error = (head_angle - self.pose_yaw + np.pi) % (2 * np.pi) - np.pi

            cost = abs(heading_error)

            # cost= np.sqrt((self.x_goal-xf0)**2 + (self.y_goal-yf0)**2)
            return cost
    
    def find_cost_obstacle(self,traj):
        if traj is not None:
            v, w, traj_points = traj

            dist=float('inf')

            for xi,yi,thi in traj_points:

                if len(self.obstacles) > 0:

                    distances = np.sqrt((self.obstacles[:, 0] - xi)**2 + (self.obstacles[:, 1] - yi)**2)

                    if len(distances)>0:
                        dist=min(dist,min(distances))
                        # print(dist)
                    else:
                        dist= float('inf')
                        return 0

                        

            cost= 1/(dist)

            return cost


    def find_cost_smoothness(self,traj):
         if traj is not None:
            v, w, traj_points = traj
            
            alpha = 1.0   # Weight for linear velocity smoothness
            beta = 1.0    # Weight for angular velocity smoothness

            v_smooth_cost = 0.0
            w_smooth_cost = 0.0

            for i in range(len(traj_points) - 1):
                x1, y1, theta1 = traj_points[i]
                x2, y2, theta2 = traj_points[i + 1]

                # Compute velocity from position change
                v1 = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) / DT
                v2 = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) / DT
                
                # Compute angular velocity from heading change
                w1 = (theta2 - theta1) / DT
                w2 = (theta2 - theta1) / DT

                # Smoothness cost based on velocity change
                v_smooth_cost += (v2 - v1) ** 2
                w_smooth_cost += (w2 - w1) ** 2

            # Total smoothness cost
            cost = alpha * v_smooth_cost + beta * w_smooth_cost
            return cost

    def is_addmissible_vel(self,v0,w0):
        print()

    def sample_velocities(self):

        v_n = int((V_MAX - V_MIN) / V_RES) + 1
        w_n = int((W_MAX - W_MIN) / W_RES) + 1

        sampled_v_min = max(self.curr_vx + A_MIN * T, V_MIN)
        sampled_v_max = min(self.curr_vx + A_MAX * T, V_MAX)
        sampled_w_min = max(self.curr_w + AL_MIN * T, W_MIN)
        sampled_w_max = min(self.curr_w + AL_MAX * T, W_MAX)

        v_samples = np.linspace(sampled_v_min, sampled_v_max, v_n)
        w_samples = np.linspace(sampled_w_min, sampled_w_max, w_n)

        return v_samples, w_samples
    
    def generate_trajectories(self,v_samples, w_samples):
        
        trajectories = []

        for v in v_samples:
            row = []
            for w in w_samples:
                collision = False
                traj_points = []
                x, y, theta = self.pose_x, self.pose_y, self.pose_yaw

                for k in range(int(T / DT)):
                    x += v * np.cos(theta) * DT
                    y += v * np.sin(theta) * DT
                    theta = (theta + w * DT + np.pi) % (2 * np.pi) - np.pi

                    if self.check_collision(x, y):
                        collision = True
                        # print("may_collide")
                        break
                    
                    traj_points.append((x, y,theta))

                if not collision:
                    row.append((v, w, traj_points))
                else:
                    row.append(None)
                    # print("Collision traj found")
            trajectories.append(row)
        
        return trajectories

    def generate_cost_function(self,trajectories):

        cost_function=[]

        for row in trajectories:
            row_cost=[]
            for traj in row:

                # v0,w0,_ = trajectories

                if traj is not None:

                    cost_head =     self.find_cost_dist(traj)
                    cost_obstacle=  self.find_cost_obstacle(traj)

                    # if cost_obstacle
                    # cost_vel    =   self.find_cost_smoothness(traj)
                    cost_angle =    self.find_cost_heading(traj)

                    norm_cost_head = sigmoid(cost_head, k=1.0, c=0.0)         # Negative k for heading to penalize large values
                    norm_cost_obstacle = sigmoid(cost_obstacle, k=50.0, c=0.3)
                     # Penalize obstacles sharply
                    # norm_cost_vel = sigmoid(cost_vel, k=5.0, c=0.0)             # Encourage smoother trajectories
                    norm_cost_angle = sigmoid(cost_angle, k=5.0, c=0.0)
                    final_cost = ALPHA * norm_cost_head + BETA*cost_obstacle + GAMMA *norm_cost_angle #+exta_penalty#+ GAMMA * norm_cost_vel

                else:
                    final_cost = float('inf')

                row_cost.append(final_cost)

            cost_function.append(row_cost)

        return cost_function
    


    def apply_DWA(self):

        # self.sample_velocities()
        v_samples,w_samples = self.sample_velocities()
        trajectories= self.generate_trajectories(v_samples,w_samples)

        

        if not trajectories:  #Handle empty trajectories
            self.get_logger().warn("No valid trajectories generated!")
            return
        
        cost_function=self.generate_cost_function(trajectories)

        min_cost = float('inf')
        ii, jj = -1, -1
        for i in range(len(cost_function)):
            for j in range(len(cost_function[i])):
                if cost_function[i][j] < min_cost:
                    min_cost = cost_function[i][j]
                    ii, jj = i, j

        

        if (ii != -1 and jj != -1 and trajectories[ii][jj] is not None):# and (self.check_collision(self.pose_x,self.pose_y)==False):
            vf = trajectories[ii][jj][0]
            wf = trajectories[ii][jj][1]
        else:
            vf=0.0
            wf=0.2
            # print("Collision Detected !!!!")
            # self.recovery_behaviour()


        self.prev_time = self.get_clock().now().to_msg().nanosec*1e-6
        self.curr_time = self.get_clock().now().to_msg().nanosec*1e-6

        while(self.curr_time-self.prev_time<=DT):
            # print("hello")
            # self.publish_velocity(vf, wf)
            self.curr_time = self.get_clock().now().to_msg().nanosec*1e-6
            # self.publish_markers(trajectories,ii,jj)
        self.get_logger().info(f"CUrrent odom value x: {self.pose_x:.4f} y: {self.pose_y:.4f}     Published velocities: vx={vf:.2f}, w={wf:.2f}  min_cost: {min_cost}")


        #For stoping at a particular position
        if np.sqrt((self.pose_x - self.x_goal) ** 2 + (self.pose_y - self.y_goal) ** 2) < 0.15:
            self.publish_velocity(0.0, 0.0)
            print("Reached Goal")
            self.publish_velocity(0.0,0.0)
            time.sleep(2)
            self.timer_.cancel()
            return



    def publish_velocity(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.vel_pub_.publish(cmd)
        # self.get_logger().info(f" Hello Published velocities: vx={v:.2f}, w={w:.2f}")

    def publish_markers(self, trajectories,ii,jj):
        marker_array = MarkerArray()
        marker_id = 0

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i,row in enumerate(trajectories):
            for j,traj in enumerate(row):
                if traj is None:
                    continue
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dwa_trajectory"
                marker.id = marker_id
                marker.type = Marker.LINE_STRIP
                marker.scale.x = 0.02
                if i == ii and j == jj:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 0.9
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 0.7
                for x, y, _ in traj[2]:
                    point=Point()
                    point.x = x
                    point.y = y
                    point.z = 0.0
                    marker.points.append(point)
                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub_.publish(marker_array)

    def pose_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        _, _, self.pose_yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                     msg.pose.pose.orientation.y,
                                                     msg.pose.pose.orientation.z,
                                                     msg.pose.pose.orientation.w])

    def odometry_callback(self, msg):
        self.pose_x=msg.pose.pose.position.x
        self.pose_y=msg.pose.pose.position.y
        _,_,self.pose_yaw=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.curr_vx = msg.twist.twist.linear.x
        self.curr_w = msg.twist.twist.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = DwaPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

