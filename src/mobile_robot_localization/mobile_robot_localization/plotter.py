import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


class OdomSubscriber(Node):

    def __init__(self):

        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(Odometry,
                                                     '/fused/odometry',
                                                     self.odom_callback,
                                                     10)
        self.coordinates = {'x': [], 'y': []}

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.coordinates['x'].append(x)
        self.coordinates['y'].append(y)
        self.get_logger().info(f"Received Odometry: x={x}, y={y}")

    def plot_coordinates(self):

        plt.plot(self.coordinates['x'], self.coordinates['y'],
                 label='Odometer Coordinates')
        plt.title('Odometer Coordinates Plot')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.legend()
        plt.show()


def main(args=None):

    rclpy.init(args=args)
    odom_subscriber = OdomSubscriber()

    try:
        rclpy.spin(odom_subscriber)
    except KeyboardInterrupt:
        odom_subscriber.get_logger().info('Keyboard interrupt, plotting coordinates...')
        odom_subscriber.plot_coordinates()

    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
