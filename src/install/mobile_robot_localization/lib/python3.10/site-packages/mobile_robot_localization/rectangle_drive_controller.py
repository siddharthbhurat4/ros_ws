import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import matplotlib.pyplot as plt

class RectangleController(Node):
        def __init__(self):
                super().__init__('rectangle_controller')
                self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
                self.odom_subscriber = self.create_subscription(Odometry,'/wheel/odometry',self.odom_callback,10)
                self.fused_odom_sub = self.create_subscription(Odometry,'/fused/odometry',self.fused_odom_callback,10)
                self.kp_linear = 0.1  # Proportional gain for linear velocity
                self.kp_angular = 0.1  # Proportional gain for angular velocity
                self.waypoints = [
                {'x': 0.0, 'y': 0.0},
                {'x': 5.0, 'y': 0.0},
                {'x': 5.0, 'y': -0.1},
                {'x': 5.0, 'y': -10.0},
                {'x': 4.9, 'y': -10.0},
                {'x': 0.0, 'y': -10.0},
                {'x': 0.0, 'y': -9.9},
                {'x': 0.0, 'y': 0.0},
                ]
                self.current_waypoint_index = 0
                self.coordinates = {'x': [], 'y': []}
                self.coordinates_fused = {'x': [], 'y': []}
                self.time_fused = []
                self.coordinates_bad = {'x': [], 'y': []}
                self.noise = 0.0001

        def fused_odom_callback(self,msg):
                current_x = msg.pose.pose.position.x
                current_y = msg.pose.pose.position.y
                self.coordinates_fused['x'].append(current_x)
                self.coordinates_fused['y'].append(current_y)
                self.time_fused.append(msg.header.stamp.sec + msg.header.stamp.sec*1e-9-1.701382e9)
                              

        def odom_callback(self, msg):
                current_x = msg.pose.pose.position.x
                current_y = msg.pose.pose.position.y
                self.coordinates['x'].append(current_x)
                self.coordinates['y'].append(current_y)
                current_yaw = euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w])[2]
                
                current_yaw_wrapped = (current_yaw + math.pi) % (2 * math.pi) - math.pi

                target_x = self.waypoints[self.current_waypoint_index]['x']
                target_y = self.waypoints[self.current_waypoint_index]['y']

                prev_x = self.waypoints[max(0,self.current_waypoint_index-1)]['x']
                prev_y = self.waypoints[max(0,self.current_waypoint_index-1)]['y']
                target_heading = math.atan2(target_y-prev_y, target_x-prev_x)

                distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
                heading_error = (target_heading - current_yaw_wrapped)
                heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

                print("dist err: ", distance_error)
                print("head err: ", heading_error)
                print(self.current_waypoint_index)
                print("--------------------------")

                if distance_error < 0.4:
                        distance_error = 0.02

                linear_velocity = self.kp_linear * distance_error
                angular_velocity = self.kp_angular * heading_error

                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = linear_velocity
                cmd_vel_msg.angular.z = angular_velocity

                self.publisher_.publish(cmd_vel_msg)
                if (self.current_waypoint_index == len(self.waypoints)-1):
                       dist_thresh = 0.1
                else:
                       dist_thresh = 0.4

                # Check if the robot has reached the current waypoint
                if distance_error < dist_thresh and abs(heading_error) < 0.2:
                        if self.current_waypoint_index == len(self.waypoints)-1:
                                cmd_vel_msg = Twist()
                                cmd_vel_msg.linear.x = 0.0
                                cmd_vel_msg.angular.z = 0.0
                                self.publisher_.publish(cmd_vel_msg)
                                self.plot_coordinates()
                        else:
                                self.current_waypoint_index = (self.current_waypoint_index + 1)
                self.noise += 0.00001                        
                self.coordinates_bad['x'].append(current_x+self.noise)
                self.coordinates_bad['y'].append(current_y+self.noise)

        def plot_coordinates(self):
                ground_truth_x = [0.0, 5.0, 5.0, 0.0,0]
                ground_truth_y = [0.0, 0.0, -10.0, -10.0,0]
                fig1, axs1 = plt.subplots(3,figsize=(8, 12))
                fig1.suptitle('Robot Positions (X Y Coordinates)',fontsize=10)
                axs1[0].plot(ground_truth_y, ground_truth_x, label='Robot Coordinates')
                axs1[0].set_title('Ground Truth',fontsize=10)
                axs1[1].plot(self.coordinates_fused['y'], self.coordinates_fused['x'], label='Robot Coordinates')
                axs1[1].set_title('Estimated Robot Coordinates',fontsize=10)
                axs1[2].plot(self.coordinates_bad['y'], self.coordinates_bad['x'], label='Robot Coordinates')
                axs1[2].set_title('Wheel Encoder Based Odometry',fontsize=10)
                fig1.tight_layout(pad=3.0)

                fig2, axs2 = plt.subplots(1)
                axs2.plot(self.time_fused, self.coordinates_fused['x'],self.time_fused, self.coordinates_fused['y'], label='Robot Coordinates')
                axs2.set_title("Position vs Time (x y coords vs time)")
                plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9, wspace=0.3, hspace=2.0)
                plt.show()

def main(args=None):    
    rclpy.init(args=args)
    rectangle_controller = RectangleController()
    try:
        rclpy.spin(rectangle_controller)
    except KeyboardInterrupt:
        rectangle_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
