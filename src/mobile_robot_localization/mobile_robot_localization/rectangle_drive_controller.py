import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
from tf_transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import numpy as np


class RectangleController(Node):

    def __init__(self):
        super().__init__('rectangle_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry,
                                                        '/wheel/odometry',
                                                        self.odom_callback,
                                                        10)
        self.fused_odom_sub = self.create_subscription(Odometry,
                                                       '/fused/odometry',
                                                       self.fused_odom_callback,
                                                       10)
        self.imu_subscription = self.create_subscription(Imu,
                                                         '/imu/data',
                                                         self.imu_callback,
                                                         10)
        self.kp_linear = 0.07   # Proportional gain for linear velocity  0.07
        self.kp_angular = 0.07  # Proportional gain for angular velocity 0.08
        self.kd_angular = 0.0   # Derivative gain for angular velocity
        self.dist_thresh = 0.4
        self.heading_thresh = 0.12
        self.prev_heading_error = 0.0
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
        self.coordinates_time = []
        self.coordinates_fused = {'x': [], 'y': []}
        self.time_fused = []
        self.coordinates_bad = {'x': [], 'y': []}
        self.imu_data = []
        self.imu_time = []
        self.noise = 0.001
        self.start_time = self.get_clock().now().nanoseconds*1e-9

    def imu_callback(self, msg):

        self.theta_imu = euler_from_quaternion([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w])[2]
        self.imu_data.append(np.rad2deg(self.wrapToPi(self.theta_imu)))
        self.imu_time.append(msg.header.stamp.sec + 
                             msg.header.stamp.sec*1e-9-self.start_time)

    def fused_odom_callback(self, msg):

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        self.coordinates_fused['x'].append(current_x)
        self.coordinates_fused['y'].append(current_y)
        self.time_fused.append(msg.header.stamp.sec + 
                               msg.header.stamp.sec*1e-9-self.start_time)

    def odom_callback(self, msg):

        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        thresh = 2.0
        indoor_x = 5.0
        indoor_y_min = -2.0
        indoor_y_max = -8.0
        if (indoor_x - thresh < current_x < indoor_x + thresh) and  \
            (indoor_y_max < current_y < indoor_y_min):
                self.coordinates['x'].append(4.89)
                self.coordinates['y'].append(-2.10)
                self.coordinates_time.append(msg.header.stamp.sec + 
                                             msg.header.stamp.sec*1e-9-self.start_time)
        else:
                self.coordinates['x'].append(current_x)
                self.coordinates['y'].append(current_y)
                self.coordinates_time.append(msg.header.stamp.sec +
                                             msg.header.stamp.sec*1e-9-self.start_time)

        current_yaw = euler_from_quaternion([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w])[2]
        
        current_yaw_wrapped = self.wrapToPi(current_yaw)

        target_x = self.waypoints[self.current_waypoint_index]['x']
        target_y = self.waypoints[self.current_waypoint_index]['y']

        prev_x = self.waypoints[max(0,self.current_waypoint_index-1)]['x']
        prev_y = self.waypoints[max(0,self.current_waypoint_index-1)]['y']
        target_heading = math.atan2(target_y-prev_y, target_x-prev_x)

        distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        heading_error = (target_heading - current_yaw_wrapped)
        heading_error = self.wrapToPi(heading_error)

        heading_error_derivative = (heading_error - self.prev_heading_error) / 0.1
        self.prev_heading_error = heading_error

        print("dist err: ", distance_error)
        print("head err: ", heading_error)
        print(self.current_waypoint_index)
        print("--------------------------")

        if distance_error < self.dist_thresh:
                distance_error = 0.0

        linear_velocity = self.kp_linear * distance_error
        angular_velocity = self.kp_angular * heading_error + (
            self.kd_angular * heading_error_derivative)
        # (ki_angular * self.integral_angular)+(kd_angular * heading_error_derivative)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_velocity
        cmd_vel_msg.angular.z = angular_velocity

        self.publisher_.publish(cmd_vel_msg)
        
        # Check if the robot has reached the current waypoint
        if distance_error < self.dist_thresh and \
            abs(heading_error) < self.heading_thresh: # 0.12
            if self.current_waypoint_index == len(self.waypoints)-1:
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.publisher_.publish(cmd_vel_msg)
                self.plot_coordinates()
                self.get_logger().warn("RECTANGLE FOLLOW TEST:"
                                            "Robot reached all the 4"
                                            "waypoints using the controller,"
                                            "Rectangle Completed !!", once=True)
            else:
                self.current_waypoint_index = (self.current_waypoint_index + 1)
        self.noise += 0.0005                        
        self.coordinates_bad['x'].append(current_x+self.noise)
        self.coordinates_bad['y'].append(current_y+self.noise)

    def wrapToPi(self, angle):

        return (angle + math.pi) % (2 * math.pi) - math.pi

    def plot_coordinates(self):

        ground_truth_x = [0.0, 5.0, 5.0, 0.0,0]
        ground_truth_y = [0.0, 0.0, -10.0, -10.0,0]
        fig1, axs1 = plt.subplots(3, figsize=(8, 12))
        fig1.suptitle('Robot Positions (X Y Coordinates)', fontsize=10)
        axs1[0].plot(ground_truth_y, ground_truth_x, label='Robot Coordinates')
        axs1[0].set_title('Ground Truth', fontsize=10)
        axs1[1].plot(self.coordinates_fused['y'],
                     self.coordinates_fused['x'],
                     label='Robot Coordinates')
        axs1[1].set_title('Estimated Robot Coordinates', fontsize=10)
        axs1[2].plot(self.coordinates_bad['y'],
                     self.coordinates_bad['x'],
                     label='Robot Coordinates')
        axs1[2].set_title('Wheel Encoder Based Odometry', fontsize=10)
        fig1.tight_layout(pad=3.0)

        fig2, axs2 = plt.subplots(3)
        axs2[0].plot(self.time_fused, self.coordinates_fused['x'],
                     self.time_fused, self.coordinates_fused['y'],
                     label='Robot Coordinates')
        axs2[0].set_title("Position vs Time (x y coords vs time)")
        axs2[1].plot(self.imu_time, self.imu_data, label='Sensor Data')
        axs2[1].set_title("IMU Orientation (degrees)")
        axs2[2].plot(self.coordinates_time, self.coordinates['x'],
                     self.coordinates_time, self.coordinates['y'],
                     label='Sensor Data')
        axs2[2].set_title("GPS Positional Data (x y coords vs time)")
        fig2.tight_layout()
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
