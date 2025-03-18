#! /usr/bin/env python3

from rclpy.node import Node
import rclpy
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped , Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

ALPHA = 1.0
BETA = 0.6
GAMMA = 0.5

class DwaPlanner(Node):

    def __init__(self):
        super().__init__('dwa_planner')
        self.scan_sub_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.pose_sub_ = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        self.obstacles = np.empty((0, 2))

        self.x_goal = 2.0
        self.y_goal = 1.0
        self.goal_theta = 1.57

        self.curr_vx = 0.0
        self.curr_w = 0.0
        self.pose_x = -2.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0

        self.permissible_v_min = -0.2
        self.permissible_v_max = +0.4
        self.permissible_w_min = -0.4
        self.permissible_w_max = +0.4

        self.timer_ = self.create_timer(0.05, self.sample_velocities)

    def scan_callback(self, msg):
        self.create_obstacle_array(msg)

    def create_obstacle_array(self, scan):
        self.obstacles = np.empty((0, 2))
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                self.obstacles = np.vstack((self.obstacles, [x, y]))
            angle += scan.angle_increment

    def check_collision(self, x, y):
        if len(self.obstacles) == 0:
            return False
        distances = np.sqrt((self.obstacles[:, 0] - x) ** 2 + (self.obstacles[:, 1] - y) ** 2)
        return np.min(distances) < 0.2

    def sample_velocities(self):
        v_min, v_max = -0.3, +0.6
        w_min, w_max = -0.5, +0.5
        a_min, a_max = -1.0, +1.0
        al_min, al_max = -0.2, +0.2
        dt = 0.1
        del_T = 3.0
        v_n, w_n = 10, 20

        sampled_v_min = max(self.curr_vx + a_min * del_T, self.permissible_v_min)
        sampled_v_max = min(self.curr_vx + a_max * del_T, self.permissible_v_max)
        sampled_w_min = max(self.curr_w + al_min * del_T, self.permissible_w_min)
        sampled_w_max = min(self.curr_w + al_max * del_T, self.permissible_w_max)

        v_samples = np.linspace(sampled_v_min, sampled_v_max, v_n)
        w_samples = np.linspace(sampled_w_min, sampled_w_max, w_n)

        trajectories = []
        cost_function = []

        for v in v_samples:
            row = []
            for w in w_samples:
                traj = []
                x, y, theta = self.pose_x, self.pose_y, self.pose_yaw
                collision = False

                for _ in range(int(del_T / dt)):
                    x += v * np.cos(theta) * dt
                    y += v * np.sin(theta) * dt
                    theta = (theta + w * dt + np.pi) % (2 * np.pi) - np.pi

                    if self.check_collision(x, y):
                        collision = True
                        break

                    traj.append((x, y))

                if not collision:
                    row.append((v, w, traj))
                else:
                    row.append(None)

            trajectories.append(row)

        # Calculate costs
        for row in trajectories:
            row_cost = []
            for traj in row:
                if traj is None:
                    row_cost.append(float('inf'))
                    continue

                # print(f"traj[-1] = {traj[-1]}")
                # xf=0
                # yf=0


                xf, yf= traj[-1][-1]
                dist_cost = np.linalg.norm([xf - self.x_goal, yf - self.y_goal])
                  # Ensure traj[0] is a tuple with at least two elements
                if isinstance(traj, tuple) and len(traj) >= 2:
                    vel_cost = traj[0]  # First element is the linear velocity
                else:
                    vel_cost = 0.0 
                head_angle = np.arctan2(self.y_goal - yf, self.x_goal - xf)

                cost = ALPHA * dist_cost + BETA * abs(head_angle) + GAMMA * vel_cost
                row_cost.append(cost)
            cost_function.append(row_cost)

        # Select best trajectory
        best_v, best_w = 0.0, 0.0
        min_cost = float('inf')

        for i in range(len(cost_function)):
            for j in range(len(cost_function[i])):
                if cost_function[i][j] < min_cost:
                    min_cost = cost_function[i][j]
                    best_v = v_samples[i]
                    best_w = w_samples[j]

        self.publish_velocity(best_v, best_w)
        self.publish_markers(trajectories)

    def publish_velocity(self, v, w):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.vel_pub_.publish(cmd)

    def publish_markers(self, trajectory):
        marker_array = MarkerArray()
        marker_id = 0

        for i in range(len(trajectory)):
            for j in range(len(trajectory[i])):
                if trajectory[i][j] is None:  # Handle None case
                    continue
                
                traj_points = trajectory[i][j][2]  # Extract trajectory points
                if traj_points is None or len(traj_points) == 0:
                    continue

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dwa_trajectory"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD

                # Line properties
                marker.scale.x = 1.0
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.7
                marker.lifetime.sec = 2


                for point in traj_points:
                    if isinstance(point, (list, tuple)) and len(point) >= 2:
                        # p = Pose()
                        # p.position.x = point[0]
                        # p.position.y = point[1]
                        # p.position.z = 0.0
                        # marker.points.append(p.position)
                        p = Point()
                        p.x = point[0]
                        p.y = point[1]
                        p.z = 0.0
                        marker.points.append(p)

                marker_array.markers.append(marker)

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        self.marker_pub_.publish(marker_array)


    def pose_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        _, _, self.pose_yaw = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

    def odometry_callback(self, msg):
        self.curr_vx = msg.twist.twist.linear.x
        self.curr_w = msg.twist.twist.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = DwaPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

