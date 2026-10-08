"""Run a Qt window and spin a ROS node from the Qt event loop."""

import signal
import sys

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication
from rclpy.signals import SignalHandlerOptions


def run(node_factory, window_factory, on_exit=None, args=None):
    """Create node + window, spin ROS every 20 ms, return the Qt exit code.

    on_exit(node) runs after the event loop ends, while rclpy is still alive.
    """
    # Qt owns the main loop, so handle SIGINT here instead of allowing the
    # default ROS handler to invalidate the context under an active Qt timer.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = node_factory()
    app = QApplication(sys.argv)
    window = window_factory(node)
    window.show()
    signal.signal(signal.SIGINT, lambda _signum, _frame: app.quit())

    ros_timer = QTimer(window)

    def spin_ros():
        if not rclpy.ok():
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0)
        except KeyboardInterrupt:
            app.quit()
        except Exception:
            # The ROS signal handler can invalidate the context between the
            # check above and spin_once() during launch shutdown.
            if rclpy.ok():
                raise
            app.quit()

    ros_timer.timeout.connect(spin_ros)
    ros_timer.start(20)

    try:
        return app.exec_()
    finally:
        ros_timer.stop()
        if rclpy.ok() and on_exit is not None:
            on_exit(node)
            rclpy.spin_once(node, timeout_sec=0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
