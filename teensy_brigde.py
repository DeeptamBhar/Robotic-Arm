import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import threading
import time

# --- CONFIGURATION ---
# Adjust this to convert your Raw Steps into Degrees/Radians for the GUI.
# If you leave it as 1.0, the GUI will receive raw step counts (e.g. 1000).
# Example: If 3200 steps = 360 degrees, this value should be (360.0 / 3200.0)
STEP_SCALE_FACTOR = 1.0 

class TeensySerialBridge(Node):
    def __init__(self):
        super().__init__('teensy_joint_reader')

        # --- Parameters ---
        self.declare_parameter('publish_rate', 20.0) # Matches Teensy's 20Hz (50ms)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        self.publish_rate = self.get_parameter('publish_rate').value
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # --- Serial Connection ---
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to Teensy on {port} at {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            exit(1)

        # --- ROS Publisher ---
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_state)

        # --- Data Storage ---
        self.latest_positions = [0.0] * 6
        self.current_mode = "UNKNOWN" # To store "HIGH" or "LOW"
        self.data_lock = threading.Lock()
        self.running = True

        # --- Start Serial Thread ---
        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def read_serial_loop(self):
        """
        Reads lines like: "HIGH,0,0,0,0,0,0"
        """
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                    
                    try:
                        parts = line.split(',')
                        
                        # We expect 7 parts: [MODE, Step0, Step1, Step2, Step3, Step4, Step5]
                        if len(parts) == 7:
                            mode_str = parts[0]
                            
                            # Convert the remaining 6 parts to floats and scale them
                            raw_steps = [float(p) for p in parts[1:]]
                            scaled_pos = [steps * STEP_SCALE_FACTOR for steps in raw_steps]
                            
                            with self.data_lock:
                                self.current_mode = mode_str
                                self.latest_positions = scaled_pos
                                
                            # Optional: Log if mode changes
                            # self.get_logger().info(f"Mode: {mode_str} | Pos: {scaled_pos}")
                            
                        else:
                            # Only warn if it's not a partial/empty line
                            if len(line) > 5: 
                                self.get_logger().warn(f"Unexpected format: {line}")
                                
                    except ValueError:
                        self.get_logger().warn(f"Parsing error: {line}")

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(1)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        with self.data_lock:
            # We copy the positions safely
            msg.position = list(self.latest_positions)
            # You could also publish the mode in the 'frame_id' or a separate topic if needed
            # msg.header.frame_id = f"Mode: {self.current_mode}"
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeensySerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        if node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

