import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame
)
from PyQt6.QtCore import QTimer

# Matplotlib embedding
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

# --------------------------
# ROS 2 Node Integration
# --------------------------
class GuiSubscriberNode(Node):
    def __init__(self):
        super().__init__('mac_gui_visualizer')
        
        # Subscribe to the same topic the Pi is publishing
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        
        # Store the latest data (6 joints)
        # Default to all zeros until we get real data
        self.current_angles = [0.0] * 6 

    def listener_callback(self, msg):
        # The Teensy bridge sends data in msg.position
        # We assume the order is [Joint1, Joint2, ..., Joint6]
        if len(msg.position) >= 6:
            self.current_angles = list(msg.position)

# --------------------------
# Motor Panel Widget (Same as before)
# --------------------------
class MotorPanel(QFrame):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor_id = motor_id
        self.setFrameShape(QFrame.Shape.Box)
        self.setLineWidth(2)
        layout = QVBoxLayout()
        
        title = QLabel(f"Motor {motor_id}")
        title.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(title)

        self.ang_label = QLabel("Angle: 0.00")
        layout.addWidget(self.ang_label)
        self.setLayout(layout)
        
        self.angle = 0.0

    def update_values(self, ang):
        self.angle = ang
        self.ang_label.setText(f"Angle: {ang:.2f}")
        
        # Simple color logic: Green if close to 0, else varying
        self.setStyleSheet("""
            QFrame { background-color: #D5F5E3; border: 1px solid black; border-radius: 5px; }
        """)

# --------------------------
# Arm Visualization (3D)
# --------------------------
class Arm3DView(FigureCanvas):
    def __init__(self):
        self.fig = Figure()
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("Live Digital Twin")
        self.setup_axes()

    def setup_axes(self):
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_zlim(0, 5)

    def update_arm(self, angles):
        self.ax.cla()
        self.setup_axes()
        
        # Simple Forward Kinematics for visualization
        # (L1, L2, L3 lengths)
        L1, L2, L3 = 1.5, 1.2, 1.0

        # Convert input (which might be degrees or radians)
        # Assuming input is radians for calculation
        t1, t2, t3 = angles[0], angles[1], angles[2]

        x0, y0, z0 = 0, 0, 0
        x1 = L1 * np.cos(t1)
        y1 = L1 * np.sin(t1)
        z1 = 1.0
        x2 = x1 + L2 * np.cos(t1 + t2)
        y2 = y1 + L2 * np.sin(t1 + t2)
        z2 = z1 + 1.0
        x3 = x2 + L3 * np.cos(t1 + t2 + t3)
        y3 = y2 + L3 * np.sin(t1 + t2 + t3)
        z3 = z2 + 0.5

        xs = [x0, x1, x2, x3]
        ys = [y0, y1, y2, y3]
        zs = [z0, z1, z2, z3]

        self.ax.plot(xs, ys, zs, marker="o", linewidth=4, color='blue')
        self.draw()

# --------------------------
# Main Window
# --------------------------
class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node # Reference to the ROS logic
        self.setWindowTitle("MacBook Command Station")
        self.resize(1000, 600)

        main_layout = QHBoxLayout()

        # Left: Motor Panels
        left_widget = QWidget()
        left_layout = QGridLayout()
        left_widget.setLayout(left_layout)
        self.motor_panels = []
        for i in range(6):
            panel = MotorPanel(i + 1)
            self.motor_panels.append(panel)
            left_layout.addWidget(panel, i // 2, i % 2)

        # Right: 3D View
        self.arm_view = Arm3DView()

        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(self.arm_view, 2)
        self.setLayout(main_layout)

        # --- THE MAGIC LINK ---
        # We need a timer to periodically ask ROS "Did we get new data?"
        # independent of the GUI loop.
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui_from_ros)
        self.timer.start(50) # Check every 50ms

    def update_gui_from_ros(self):
        # 1. Spin ROS once to process any incoming packets
        rclpy.spin_once(self.ros_node, timeout_sec=0)

        # 2. Get the latest data from the node
        latest_angles = self.ros_node.current_angles

        # 3. Update the GUI widgets
        for i, panel in enumerate(self.motor_panels):
            if i < len(latest_angles):
                panel.update_values(latest_angles[i])

        # 4. Update the 3D plot
        self.arm_view.update_arm(latest_angles)

# --------------------------
# Run
# --------------------------
if __name__ == "__main__":
    # Initialize ROS 2
    rclpy.init(args=sys.argv)
    
    # Create the ROS Node
    ros_node = GuiSubscriberNode()

    # Create the PyQt App
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # Run the App
    try:
        sys.exit(app.exec())
    finally:
        # Cleanup when window closes
        ros_node.destroy_node()
        rclpy.shutdown()