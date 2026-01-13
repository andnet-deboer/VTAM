import rclpy
import threading
from vtam_core.recording.recorder import UniversalRecorder
from vtam_core.drivers.tactile import TactileBridge

def get_user_input(recorder_node):
    while rclpy.ok():
        cmd = input("Commands: [r]ecord, [s]top, [q]uit: ").strip().lower()
        if cmd == 'r':
            task = input("Enter task name (e.g., make_sandwich): ")
            recorder_node.start_recording(task)
        elif cmd == 's':
            recorder_node.stop_recording()
        elif cmd == 'q':
            recorder_node.stop_recording()
            rclpy.shutdown()
            break

def main(args=None):
    rclpy.init(args=args)
    
    # In a real app, we'd use a Launch file to start nodes.
    # For this CLI tool, we spin the recorder in a thread.
    recorder = UniversalRecorder()
    
    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()
    
    print("=== VTAM TEACH MODE ===")
    print("Waiting for data streams...")
    
    # CLI Loop
    try:
        get_user_input(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()