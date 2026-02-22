# 🤖 Robotic Arm Control Center

Welcome to the **Robotic Arm** project! This repository contains both a Python-based GUI control and a brand new **Web Dashboard** with a live 3D Digital Twin visualization.

---

## 🚀 Features

- **6-Axis Interactivity**: Control the base yaw, shoulder pitch, elbow pitch, wrist twist, wrist pitch, and tool roll.
- **3D Digital Twin**: A live 3D kinematic model of your robotic arm built with `@react-three/fiber` that visualizes changes in real time.
- **Manual Control Sliders**: Use intuitive range sliders to manually control the angle for each of the 6 degrees of freedom.
- **Interactive Gripper**: Toggle the gripper jaw (Open/Close) directly from the dashboard header.
- **2D Projections**: Real-time Top (X-Y) and Side (X-Z) SVG-based vector orthographic views.
- **Motor Diagnostic Panels**: Continuously visible Velocity, Acceleration, and Angle data accompanied by visual threshold warnings.

---

## 🛠️ Getting Started

### 1. Python Control Environment (Original)

To run the Python GUI control interface:

```bash
# Install required dependencies
python -m pip install PyQt6 matplotlib numpy

# Run the GUI
python gui.py
```

*(You can also use `teensy_brigde.py` or `test_serial.py` to test serial communication with the hardware.)*

### 2. Next-Gen Web Dashboard (React + Three.js)

To run the modern web dashboard interface with 3D visualization:

1. Navigate to the `web-dashboard` directory:
   ```bash
   cd web-dashboard
   ```
2. Install the Node dependencies:
   ```bash
   npm install
   ```
3. Start the local development server:
   ```bash
   npm run dev
   ```
4. Open your browser and navigate to the address shown in your terminal (usually `http://localhost:5173`).

---

## 🎨 Design System

The web dashboard is styled with a custom dark-mode glassmorphism theme and uses:
- React + Vite
- `three.js` & `@react-three/fiber` for the 3D Canvas
- `lucide-react` for iconography
- Custom CSS for smooth transitions, blooming effects, and interactive UI states.

---

Enjoy controlling your robotic arm! 🦾
