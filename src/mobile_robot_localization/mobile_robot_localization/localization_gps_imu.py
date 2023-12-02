import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np


class SensorFusionLocalization(Node):

    def __init__(self):
        super().__init__('sensor_fusion_localization')

        # Publish Fused Odometry
        self.fused_odom_pub = self.create_publisher(Odometry,
                                                    '/fused/odometry',
                                                    10)
        # Fused Odometry Publishing Timer
        self.fused_odom_pub_timer = self.create_timer(0.1, self.timer_callback)
        # Subscribe to IMU
        self.imu_subscription = self.create_subscription(Imu,
                                                         '/imu/data',
                                                         self.imu_callback,
                                                         10)
        # Subscribe to cmd_vel
        self.cmd_vel_subscription = self.create_subscription(Twist,
                                                             '/cmd_vel',
                                                             self.cmd_vel_callback,
                                                             10)
        # Subscribe to cmd_vel
        self.gps_subscription = self.create_subscription(Odometry,
                                                         '/wheel/odometry',
                                                         self.gps_callback,
                                                         10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_lin = 0.0
        self.omega = 0.0
        self.theta_imu = 0.0
        self.curr_time = 0.0
        self.prev_time = 0.0
        self.state_estimate = np.array([self.x, self.y, self.theta]).T
        self.control_vector = np.array([self.v_lin, self.omega]).T
        self.gps_obs_vector = np.array([0.0, 0.0, 0.0]).T
        self.num = 0

        self.P = np.array([[0.1, 0.0, 0.0],
                           [0.0, 0.1, 0.0],
                           [0.0, 0.0, 0.1]])

    def imu_callback(self, msg):

        self.theta_imu = euler_from_quaternion([
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w])[2]

    def cmd_vel_callback(self, msg):

        self.v_lin = msg.linear.x
        self.omega = msg.angular.z
        self.control_vector = np.array([self.v_lin, self.omega]).T

    def gps_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w])[2]
        self.gps_obs_vector = np.array([x, y, theta])

    def timer_callback(self):

        # State Transition Matrix for Linearized Dynamics (Prediction)
        A = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]])
        # Process Noise
        process_noise = np.array([0.004, 0.004, 0.005])

        # State model noise covariance matrix
        Q = np.array([[0.01, 0.0, 0.0],
                      [0.0, 0.01, 0.0],
                      [0.0, 0.0, 0.01]])

        # Measurement matrix H
        H = np.array([[1.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0],
                      [0.0, 0.0, 1.0]])

        # Sensor measurement noise covariance matrix R
        gps_x = self.gps_obs_vector[0]
        gps_y = self.gps_obs_vector[1]
        thresh = 2.0
        indoor_x = 5.0
        indoor_y_min = -2.0
        indoor_y_max = -8.0

        # Changing indoor outdoor conditions by making the sensor to be less accurate
        if (indoor_x - thresh < gps_x < indoor_x + thresh) \
                and (indoor_y_max < gps_y < indoor_y_min):
            self.get_logger().info("ROBOT IS INDOORS", once=True)
            R = np.array([[2.0, 0.0, 0.0],
                          [0.0, 2.0, 0.0],
                          [0.0, 0.0, 2.0]])
            self.num = 75889933
        else:
            self.get_logger().info("ROBOT IS OUTDOORS", once=True)
            R = np.array([[0.01, 0.0, 0.0],
                          [0.0, 0.01, 0.0],
                          [0.0, 0.0, 0.01]])
            if (self.num == 75889933):
                self.num += 1

        if (self.num == 75889934):
            self.get_logger().info("ROBOT IS BACK OUTDOORS", once=True)

        # Sensor noise
        sensor_noise = np.array([0.001, 0.001, 0.003])

        # Take the current time
        self.curr_time = self.get_clock().now().nanoseconds*1e-9

        # Calculate delta time step
        delta_time = (self.curr_time - self.prev_time)/2

        # Predict the state
        self.state_estimate = A @ (self.state_estimate) + (
            self.getBLinearized(self.wrapToPi(self.state_estimate[2]), delta_time)) @ (
                self.control_vector)+(
                process_noise)
        self.state_estimate[2] = self.wrapToPi(self.state_estimate[2])

        # Predicted covariance
        self.P = A @ self.P @ A.T + Q

        # Calculate the Innovation between prediction and measurement
        innovation = self.gps_obs_vector - ((H @ self.state_estimate) + (sensor_noise))

        # Calculate the measurement residual covariance
        S = H @ self.P @ H.T + R

        # Calculate the Kalman gain
        K = self.P @ H.T @ np.linalg.pinv(S)

        # Calculate an updated state estimate
        self.state_estimate = self.state_estimate + (K @ innovation)

        # Update the state covariance estimate for time k
        self.P = self.P - (K @ H @ self.P)

        # Update the states
        self.x = self.state_estimate[0]
        self.y = self.state_estimate[1]
        self.theta = self.state_estimate[2]
        self.prev_time = self.curr_time
        self.publish_fused_odom()

    def wrapToPi(self, angle):

        return (angle + math.pi) % (2 * math.pi) - math.pi

    def getBLinearized(self, yaw, deltaTime):

        # Expresses how the state of the system [x,y,yaw] changes
        # from k-1 to k due to the control commands (i.e. control input).
        # :param yaw: The yaw angle (rotation angle around the z axis) in rad
        # :param deltaTime: The change in time from time step k-1 to k in sec
        B = np.array([[np.cos(yaw)*deltaTime, 0],
                      [np.sin(yaw)*deltaTime, 0],
                      [0, deltaTime]])
        return B

    def publish_fused_odom(self):

        # Create Odometry message for fused odometry
        fused_odom_msg = Odometry()
        fused_odom_msg.header.stamp = self.get_clock().now().to_msg()
        fused_odom_msg.header.frame_id = 'odom'
        fused_odom_msg.child_frame_id = 'base_link'

        # Populate the pose information
        fused_odom_msg.pose.pose.position.x = self.x
        fused_odom_msg.pose.pose.position.y = self.y
        quat = quaternion_from_euler(0, 0, self.theta)
        fused_odom_msg.pose.pose.orientation.x = quat[0]
        fused_odom_msg.pose.pose.orientation.y = quat[1]
        fused_odom_msg.pose.pose.orientation.z = quat[2]
        fused_odom_msg.pose.pose.orientation.w = quat[3]

        # Populate the twist information
        fused_odom_msg.twist.twist.linear.x = self.v_lin
        fused_odom_msg.twist.twist.linear.y = 0.0
        fused_odom_msg.twist.twist.angular.z = self.omega

        # Publish the fused odometry
        self.fused_odom_pub.publish(fused_odom_msg)


def main(args=None):

    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionLocalization()
    rclpy.spin(sensor_fusion_node)
    sensor_fusion_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
