import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class RectanglePublisher(Node):

    def __init__(self):

        super().__init__('rectangle_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_vel_msg = Twist()

    def drive_rect(self):

        turn_time = 10.7
        forward_length_time = 31.0
        forward_width_time = 15.0
        # Drive in a rectangle pattern
        linear_speed = 0.5
        angular_speed = 0.2

        # Move forward (5 meters)
        self.cmd_vel_msg.linear.x = linear_speed
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(forward_width_time)

        # Stop
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(1)  # Pause for 1 second

        # Turn right (90 degrees)
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = -angular_speed
        self.publish_cmd_vel()
        time.sleep(turn_time)  # Turn right for 2 seconds

        # Stop
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(1)  # Pause for 1 second

        # Move forward (10 meters)
        self.cmd_vel_msg.linear.x = linear_speed
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(forward_length_time)  # Move forward for 2 seconds

        # Stop
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(1)  # Pause for 1 second

        # Turn right (90 degrees)
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = -angular_speed
        self.publish_cmd_vel()
        time.sleep(turn_time)  # Turn right for 2 seconds

        # Stop
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(1)  # Pause for 1 second

        # Move forward (5 meters)
        self.cmd_vel_msg.linear.x = linear_speed
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(forward_width_time)  # Move forward for 2 seconds

        # Stop
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(1)  # Pause for 1 second

        # Turn right (90 degrees)
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = -angular_speed
        self.publish_cmd_vel()
        time.sleep(turn_time)  # Turn right for 2 seconds

        # Stop
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(1)  # Pause for 1 second

        # Move forward (10 meters)
        self.cmd_vel_msg.linear.x = linear_speed
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(forward_length_time)  # Move forward for 2 seconds

        # Stop
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publish_cmd_vel()
        time.sleep(1)  # Pause for 1 second

    def publish_cmd_vel(self):

        self.publisher_.publish(self.cmd_vel_msg)
        # self.get_logger().info(f"Publishing cmd_vel: {self.cmd_vel_msg}")

def main(args=None):

    rclpy.init(args=args)
    rectangle_publisher = RectanglePublisher()
    rectangle_publisher.drive_rect()
    rclpy.spin(rectangle_publisher)
    rectangle_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()
