#! /usr/bin/env python3

from rclpy.node import Node
import rclpy
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


ALPHA= 0.5
BETA=  1.0
GAMMA= 0.7

class DwaPlanner(Node):

    def __init__(self):
        super().__init__('dwa_planner')
        self.scan_sub_ = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.odom_sub_ = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        # self.sub_= self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.pose_callback,10)
        self.vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub_ = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.timer_ = self.create_timer(0.002, self.sample_velocties)
        self.obstacles=np.empty((0,2))

        self.x_goal=2.0
        self.y_goal= 1.0
        self.goal_theta= 1.57

        self.curr_vx=0.0
        self.curr_w=0.0
        self.pose_x=-2.0
        self.pose_y=0.0
        self.pose_yaw=0.0

        #sampling parameters
        v_min = -1.0
        v_max = +1.0
        w_min = -1.0
        w_max = +1.0

        a_min = -1.0
        a_max = +1.0

        al_min= 1.0
        al_max= -1.0

        v_res =  0.05
        w_res =  0.1

        del_t =  2.0
        
        v_sam =  10
        w_sam =  20

        self.timer_ = self.create_timer(0.05, self.sample_velocties)





    def scan_callback(self,msg):
        self.create_obsctacle_array(msg)

    def publish_markers(self, trajectory):
        marker_array = MarkerArray()

        marker_id = 0
        for i in range(len(trajectory)):
            for j in range(len(trajectory[i])):
                traj_points = trajectory[i][j][2]
                if len(traj_points) == 0:
                    continue

                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dwa_trajectory"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD

                # Line properties
                marker.scale.x = 0.02  # Line width
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.7  # Transparency

                for point in traj_points:
                    p = Pose()
                    p.position.x = point[0]
                    p.position.y = point[1]
                    p.position.z = 0.0
                    marker.points.append(p.position)

                marker_array.markers.append(marker)

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        self.marker_pub_.publish(marker_array)


    def create_obsctacle_array(self,scan):
        
        self.obstacles=np.empty((0,2))
        angle= scan.angle_min
        closest=1000

        # plan_window =3.0

        for r in scan.ranges:

            if scan.range_min<r<scan.range_max :
                x= r*np.cos(angle)
                y= r*np.sin(angle)

                dist=np.sqrt(x**2+y**2)
                closest=min(closest,dist)

                # if dist < plan_window:
                self.obstacles=np.vstack((self.obstacles,[x,y]))

            angle = (angle +scan.angle_increment) %(2*np.pi)

        # print(f"Number of obstacle detected: {len(self.obstacles)} closest obstacle is at {closest}")

    def check_collision(self, x, y):
        if len(self.obstacles) == 0:
            return False

        distances = np.sqrt((self.obstacles[:, 0] - x)**2 + (self.obstacles[:, 1] - y)**2)
        min_distance = np.min(distances)

        safety_distance = 0.25  
        if min_distance < safety_distance:
            return True
        return False

    def sample_velocties(self):
        v_min = -0.2
        v_max = +0.5
        w_min = -0.5
        w_max = +0.5

        a_min = -1.0
        a_max = +1.0

        al_min= -0.2
        al_max= +0.2

        v_res =  0.05
        w_res =  0.1

        del_T =  1.5
        dt    =  0.05

        
        v_n =  10
        w_n =  20

        theta=self.pose_yaw

        sampled_v_min = max((self.curr_vx + a_min*del_T),v_min)
        sampled_v_max = min((self.curr_vx + a_max*del_T),v_max)

        sampled_w_min = max((self.curr_w  + al_min*del_T),w_min)
        sampled_w_max = min((self.curr_w  + al_max*del_T),w_max)

        v_samples= np.linspace(sampled_v_min,sampled_v_max,v_n)
        w_samples= np.linspace(sampled_w_min,sampled_w_max,w_n)

        trajectories =[]
        cost_function=[]

        for v0 in v_samples:
            row=[]
            for w0 in w_samples:
                
                collision = False
                traj_points=[]
                xi=self.pose_x
                yi=self.pose_y
                theta=self.pose_yaw

                for k in range (int(del_T/dt)):
                    xi+= v0 * np.cos(theta)*dt
                    yi+= v0 * np.sin(theta)*dt
                    # theta=theta+w0*dt
                    theta = (theta + w0 * dt + np.pi) % (2 * np.pi) - np.pi

                    if self.check_collision(xi,yi):
                        collision=True
                        break
                    
                    traj_points.append((xi,yi,theta))
                if not collision:
                    row.append((v0,w0,traj_points))
                else:
                    row.append(None)
            trajectories.append(row)
        
        # cost_function = [[None for _ in range(w_n)] for _ in range(v_n)]

        #Calculating the Cost Function...................................
        for row in trajectories:    #every row is a trajectory of (v,w) pair
            traj_cost=[]
            for traj in row:

                if traj is None:
                    traj_cost.append(float('inf'))
                    continue

                xf, yf, th_f = traj[-1][-1]

                 

                distances = np.sqrt((self.obstacles[:, 0] - xf)**2 + (self.obstacles[:, 1] - yf)**2)
                if len(distances) > 0:
                    cost_dist = min(distances)
                else:
                    cost_dist = float('inf')


                vel_cost=traj[0]

                head_angle = np.arctan2(self.y_goal - yf, self.x_goal - xf)

                cost=ALPHA*(head_angle) + BETA*cost_dist +GAMMA*vel_cost
                traj_cost.append(cost)

            cost_function.append(traj_cost)
        
        min_cost = float('inf')
        vf,wf=0.0, 0.0

        # i_min, j_min = -1, -1

        for i in range(len(cost_function)):
            for j in range(len(cost_function[i])):
                if cost_function[i][j] < min_cost:
                    min_cost = cost_function[i][j]
                    vf=v_samples[i]
                    wf=w_samples[j]

        self.publish_vel(vf,wf)
        self.publish_markers(trajectories)

        # if np.sqrt((self.pose_x - self.x_goal)**2 + (self.pose_y - self.y_goal)**2) < 0.1:
        #     self.publish_vel(0.0, 0.0)
        #     self.timer_.cancel()
        #     return


        #need to store (x,y) , (x,y,th) final pose predicted velocity command (v,w)









    def publish_vel(self,vf,wf):
        cmd=Twist()

        cmd.linear.x=vf
        cmd.angular.z=wf
        print(f"Currect pose x:{self.pose_x} y:{self.pose_y} publ vel  x: {vf:.2f}  w: {wf:.2f}")
        self.vel_pub_.publish(cmd)

    def pose_callback(self,msg):
        self.pose_x=msg.pose.pose.position.x
        self.pose_y=msg.pose.pose.position.y
        _,_,self.pose_yaw=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])


    def odometry_callback(self,msg):
        self.pose_x=msg.pose.pose.position.x
        self.pose_y=msg.pose.pose.position.y
        _,_,self.pose_yaw=euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

        self.curr_vx=msg.twist.twist.linear.x
        self.curr_w= msg.twist.twist.angular.z

        # print(f"x vel: {self.curr_vx:.3f}  angular vel:{self.curr_w:.3f}")
        # print(f"x : {self.pose_x:.3f} y : {self.pose_y:.3f}  yaw:{self.pose_yaw:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node=DwaPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()


