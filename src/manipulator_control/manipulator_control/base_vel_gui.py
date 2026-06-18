#!/usr/bin/env python3
"""
Floating Qt window with three buttons to control J1 base velocity.
Publishes to /base_cmd (std_msgs/Float32).
"""
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QHBoxLayout
from PyQt5.QtCore import QTimer

BASE_SPEED = 1.5  # rad/s


class BaseVelGui(Node):
    def __init__(self):
        super().__init__('base_vel_gui')
        self.pub = self.create_publisher(Float32, '/base_cmd', 10)

    def send(self, v: float):
        msg = Float32()
        msg.data = v
        self.pub.publish(msg)


class BaseVelWidget(QWidget):
    def __init__(self, node: BaseVelGui):
        super().__init__()
        self.node = node
        self.setWindowTitle('Base Velocity')
        self.setFixedSize(300, 70)

        layout = QHBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)

        btn_left  = QPushButton('◄ Left')
        btn_stop  = QPushButton('Stop')
        btn_right = QPushButton('Right ►')

        btn_left.clicked.connect(lambda: node.send(BASE_SPEED))
        btn_stop.clicked.connect(lambda: node.send(0.0))
        btn_right.clicked.connect(lambda: node.send(-BASE_SPEED))

        for btn in (btn_left, btn_stop, btn_right):
            btn.setFixedHeight(44)
            layout.addWidget(btn)

        self.setLayout(layout)


def main():
    rclpy.init()
    node = BaseVelGui()

    app = QApplication(sys.argv)
    widget = BaseVelWidget(node)
    widget.show()

    # Drive ROS spin inside the Qt event loop
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_timer.start(20)  # 50 Hz

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
