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
                print("target heading: ", target_heading)
                print("current yaw: ", current_yaw)
                print("current yaw wrapped: ", current_yaw_wrapped)
                print("head err: ", heading_error)
                print(self.current_waypoint_index)
                print("--------------------------")

                if distance_error < 0.5:
                        distance_error = 0.0

                linear_velocity = self.kp_linear * distance_error
                angular_velocity = self.kp_angular * heading_error

                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = linear_velocity
                cmd_vel_msg.angular.z = angular_velocity

                self.publisher_.publish(cmd_vel_msg)
                # self.get_logger().info(f"Publishing cmd_vel: {cmd_vel_msg}")

                # Check if the robot has reached the current waypoint
                if distance_error < 0.5 and abs(heading_error) < 0.15:
                        if self.current_waypoint_index == len(self.waypoints)-1:
                                cmd_vel_msg = Twist()
                                cmd_vel_msg.linear.x = 0.0
                                cmd_vel_msg.angular.z = 0.0
                                self.publisher_.publish(cmd_vel_msg)
                                self.plot_coordinates()
                        else:
                                self.current_waypoint_index = (self.current_waypoint_index + 1)

        def plot_coordinates(self):
                plt.plot(self.coordinates['x'], self.coordinates['y'], label='Odometer Coordinates')
                plt.title('Odometer Coordinates Plot')
                plt.xlabel('X Coordinate')
                plt.ylabel('Y Coordinate')
                plt.legend()
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
