#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import signal
import sys
import os
from datetime import datetime

class CustomROSbagRecorder(Node):
    """
    Custom ROSbag recorder for drone terrain surveying unit tests.
    Records only the essential topics needed for unit testing.
    """
    
    def __init__(self):
        super().__init__('custom_rosbag_recorder')
        
        # Define topics to record for unit testing, specific to TASK 1
        self.topics_to_record = [
            '/drone/gt_odom',           # Odometry for position tracking
            '/drone/sonar',             # Sonar for terrain elevation
            '/drone/laserscan',         # Laser for obstacle detection
            '/mission/goals',           # Goals for TSP testing
            '/grid_map',                # Generated elevation map
            '/mission/path',            # Generated waypoints
            '/visualization_marker'     # Markers for validation
        ]
        
        self.recording_process = None
        self.bag_filename = None
        
        # Set up signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
    def start_recording(self, bag_name=None):
        """Start recording specified topics to a ROSbag"""
        
        if bag_name is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"drone_survey_test_{timestamp}"
        
        self.bag_filename = bag_name
        
        # Create the ros2 bag record command
        cmd = ['ros2', 'bag', 'record', '-o', self.bag_filename]
        cmd.extend(self.topics_to_record)
        
        self.get_logger().info(f"Starting ROSbag recording: {self.bag_filename}")
        self.get_logger().info(f"Recording topics: {', '.join(self.topics_to_record)}")
        
        try:
            self.recording_process = subprocess.Popen(cmd)
            self.get_logger().info("ROSbag recording started successfully")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")
            return False
    
    def stop_recording(self):
        """Stop the current recording"""
        if self.recording_process:
            self.get_logger().info("Stopping ROSbag recording...")
            self.recording_process.send_signal(signal.SIGINT)
            self.recording_process.wait()
            self.recording_process = None
            self.get_logger().info(f"Recording saved to: {self.bag_filename}")
        else:
            self.get_logger().warn("No active recording to stop")
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.get_logger().info("Received shutdown signal, stopping recording...")
        self.stop_recording()
        sys.exit(0)

def main():
    rclpy.init()
    
    recorder = CustomROSbagRecorder()
    
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description='Custom ROSbag recorder for drone survey testing')
    parser.add_argument('--bag-name', type=str, help='Name for the bag file')
    parser.add_argument('--duration', type=int, default=60, help='Recording duration in seconds')
    
    args = parser.parse_args()
    
    # Start recording
    if recorder.start_recording(args.bag_name):
        try:
            # Keep the node alive for the specified duration
            import time
            recorder.get_logger().info(f"Recording for {args.duration} seconds...")
            time.sleep(args.duration)
        except KeyboardInterrupt:
            recorder.get_logger().info("Recording interrupted by user")
        finally:
            recorder.stop_recording()
    
    recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()