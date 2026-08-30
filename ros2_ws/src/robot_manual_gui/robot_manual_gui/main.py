"""Entrypoint integrating the Qt event loop with a ROS executor thread."""

import sys
import threading

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import QApplication
import rclpy
from rclpy.executors import SingleThreadedExecutor

from dynamixel_control.tool_profiles import get_profile, load_profiles
from robot_manual_gui.main_window import ManualMainWindow
from robot_manual_gui.ros_interface import GuiSignals, ManualGuiNode


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    signals = GuiSignals()
    node = ManualGuiNode(signals)
    profile_path = (
        get_package_share_directory('dynamixel_control')
        + '/config/tool_profiles.yaml')
    profile = get_profile(load_profiles(profile_path), node.selected_tool)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    window = ManualMainWindow(node, signals, profile, node.mock_mode)
    window.show()
    result = app.exec_()
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=1.0)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
