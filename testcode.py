import threading
import sys
import time

# --- Mock Variables to simulate ROS storage ---
latest_positions = [0.0] * 6
data_lock = threading.Lock()
running = True

def parse_input_logic(line):
    """
    This is the exact logic we want to test.
    It takes a string line and tries to convert it to 6 floats.
    """
    global latest_positions
    
    line = line.strip() # Remove whitespace/newlines
    if not line:
        return False

    try:
        # 1. Split by comma
        parts = line.split(',')
        
        # 2. Check length
        if len(parts) == 6:
            # 3. Convert to float
            new_data = [float(p) for p in parts]
            
            # 4. Store safely
            with data_lock:
                latest_positions = new_data
            
            return True, new_data
        else:
            return False, f"Length Error: Expected 6 items, got {len(parts)}"
            
    except ValueError:
        return False, "Value Error: Could not convert text to numbers"

def main():
    print("--- PARSING LOGIC TEST ---")
    print("Type 6 comma-separated numbers (e.g., 10, 20, 30, 40, 50, 60)")
    print("Type 'q' to quit.\n")

    while True:
        try:
            user_input = input("Enter data > ")
            
            if user_input.lower() == 'q':
                break

            # Test the logic
            success, result = parse_input_logic(user_input)

            if success:
                print(f"✅ SUCCESS! Parsed values: {result}")
                print(f"   Stored in variable: {latest_positions}")
            else:
                print(f"❌ FAILED: {result}")
                
            print("-" * 30)

        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()