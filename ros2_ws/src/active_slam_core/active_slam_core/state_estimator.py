import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
import math


class StateEstimator(Node):

    def __init__(self):
        super().__init__('state_estimator')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("State estimator started")

    def odom_callback(self, msg):

        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Quaternion -> Yaw
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        yaw_deg = math.degrees(yaw)

        # Vitesse
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        speed = math.sqrt(vx**2 + vy**2)
        speed_kmh = speed * 3.6

        self.get_logger().info(
            f"\nVehicle State:\n"
            f"x = {x:.2f} m\n"
            f"y = {y:.2f} m\n"
            f"yaw = {yaw_deg:.2f} deg\n"
            f"speed = {speed_kmh:.2f} km/h"
        )


def main(args=None):

    rclpy.init(args=args)

    node = StateEstimator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
