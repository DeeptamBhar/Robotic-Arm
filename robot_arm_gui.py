import sys
import random
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton,
    QVBoxLayout, QHBoxLayout, QGridLayout, QFrame
)
from PyQt6.QtCore import QTimer

# Matplotlib embedding inside PyQt6
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


# --------------------------
# Motor Panel Widget
# --------------------------
class MotorPanel(QFrame):
    def __init__(self, motor_id: int):
        super().__init__()
        self.motor_id = motor_id

        self.setFrameShape(QFrame.Shape.Box)
        self.setLineWidth(2)

        layout = QVBoxLayout()

        title = QLabel(f"Motor {motor_id}")
        title.setStyleSheet("font-weight: bold; font-size: 14px; color: black;")

        layout.addWidget(title)

        self.vel_label = QLabel("Velocity: 0.00")
        self.acc_label = QLabel("Acceleration: 0.00")
        self.ang_label = QLabel("Angle: 0.00")

        layout.addWidget(self.vel_label)
        layout.addWidget(self.acc_label)
        layout.addWidget(self.ang_label)

        # Reset button
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset_motor)
        layout.addWidget(self.reset_btn)
        self.reset_btn.setStyleSheet("""
    QPushButton {
        background-color: white;
        color: black;
        font-weight: bold;
        border: 2px solid black;
        padding: 6px;
        border-radius: 6px;
    }
    QPushButton:hover {
        background-color: #dddddd;
    }
    QPushButton:pressed {
        background-color: #bbbbbb;
    }
""")


        self.setLayout(layout)

        # Example threshold limits 
        self.vel_limit = 8.0
        self.acc_limit = 6.0
        self.ang_limit = 170.0

        self.velocity = 0.0
        self.acceleration = 0.0
        self.angle = 0.0

        self.update_color(ok=True)

    def update_values(self, v, a, ang):
        self.velocity = v
        self.acceleration = a
        self.angle = ang

        self.vel_label.setText(f"Velocity: {v:.2f}")
        self.acc_label.setText(f"Acceleration: {a:.2f}")
        self.ang_label.setText(f"Angle: {ang:.2f}")

        # Decide if within safe range
        ok = (abs(v) <= self.vel_limit) and (abs(a) <= self.acc_limit) and (0 <= ang <= self.ang_limit)
        self.update_color(ok)

    def update_color(self, ok: bool):
        # Deeper green / red + force all text black
        if ok:
            self.setStyleSheet("""
                QFrame {
                    background-color: #2ECC71;   /* deep green */
                    color: black;
                    border: 2px solid black;
                    border-radius: 8px;
                }
                QLabel { color: black; font-size: 13px; }
            """)
        else:
            self.setStyleSheet("""
                QFrame {
                    background-color: #E74C3C;   /* deep red */
                    color: black;
                    border: 2px solid black;
                    border-radius: 8px;
                }
                QLabel { color: black; font-size: 13px; }
            """)


    def reset_motor(self):
        # Reset values example
        self.update_values(0.0, 0.0, 0.0)


# --------------------------
# Arm Visualization (Matplotlib 3D)
# --------------------------
class Arm3DView(FigureCanvas):
    def __init__(self):
        self.fig = Figure()
        super().__init__(self.fig)

        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("Live Robotic Arm Model")

        # Set fixed axis bounds
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_zlim(0, 5)

        self.line, = self.ax.plot([], [], [], marker="o")  # arm links

    def update_arm(self, joint_angles_deg):
        """
        joint_angles_deg = [theta1, theta2, theta3, theta4, theta5, theta6]
        We’ll just animate a simple 3-link arm demo using first 3 angles.
        """
        self.ax.cla()
        self.ax.set_title("Live Robotic Arm Model")
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_zlim(0, 5)

        # Link lengths
        L1, L2, L3 = 1.5, 1.2, 1.0

        # Convert degrees -> radians
        t1 = np.deg2rad(joint_angles_deg[0])
        t2 = np.deg2rad(joint_angles_deg[1])
        t3 = np.deg2rad(joint_angles_deg[2])

        # Simple planar-ish forward kinematics (demo)
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

        self.ax.plot(xs, ys, zs, marker="o", linewidth=3)

        self.draw()


# --------------------------
# Main Window
# --------------------------
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robotic Arm Monitoring GUI")
        self.resize(1200, 650)

        main_layout = QHBoxLayout()

        # ---------------- Left Pane: 6 Motor Panels ----------------
        left_widget = QWidget()
        left_layout = QGridLayout()
        left_widget.setLayout(left_layout)

        self.motor_panels = []
        for i in range(6):
            panel = MotorPanel(i + 1)
            self.motor_panels.append(panel)
            left_layout.addWidget(panel, i // 2, i % 2)  # 2 columns layout

        # ---------------- Right Pane: 3D Arm View ----------------
        self.arm_view = Arm3DView()

        # Add to main layout
        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(self.arm_view, 2)

        self.setLayout(main_layout)

        # Timer for refresh (updates values continuously)
        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_data)
        self.timer.start(100)  # refresh every 100 ms

        self.joint_angles = [0, 0, 0, 0, 0, 0]

    def refresh_data(self):
        # --------- Example: Fake data for testing ----------
        # Replace this with ROS2 data / Serial / CAN / UDP etc.
        for panel in self.motor_panels:
            v = random.uniform(-10, 10)
            a = random.uniform(-8, 8)
            ang = random.uniform(0, 200)

            panel.update_values(v, a, ang)

        # --------- Update arm animation from angles ----------
        for i in range(6):
            self.joint_angles[i] = self.motor_panels[i].angle

        self.arm_view.update_arm(self.joint_angles)


# --------------------------
# Run
# --------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
