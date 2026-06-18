#!/usr/bin/env python3
"""
Bridges RViz 'Publish Point' clicks to /target_pose for the IK node.

RViz publishes clicked 3D points on /clicked_point (geometry_msgs/PointStamped).
This node strips the header and republishes as Point on /target_pose.

Usage:
  1. In RViz, select the 'Publish Point' tool (top toolbar)
  2. Click anywhere on the grid or a surface
  3. The manipulator moves to that point

Also publishes a sphere marker at the click location for visual feedback.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


class ClickToTarget(Node):
    def __init__(self):
        super().__init__('click_to_target')

        self.sub = self.create_subscription(
            PointStamped, '/clicked_point', self.click_cb, 10
        )
        self.pub = self.create_publisher(Point, '/target_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/click_marker', 10)

        self.click_id = 0

        self.get_logger().info(
            'Click-to-target ready.\n'
            '  Select "Publish Point" tool in RViz toolbar,\n'
            '  then click on the grid to move the manipulator.'
        )

    def click_cb(self, msg: PointStamped):
        p = msg.point

        self.get_logger().info(
            f'Click received: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
        )

        # Forward to IK
        self.pub.publish(p)

        # Visual feedback: sphere at click location
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'click_targets'
        marker.id = self.click_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = p.x
        marker.pose.position.y = p.y
        marker.pose.position.z = p.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        marker.color = ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.9)
        marker.lifetime = Duration(sec=60)

        self.marker_pub.publish(marker)
        self.click_id += 1


def main():
    rclpy.init()
    node = ClickToTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
