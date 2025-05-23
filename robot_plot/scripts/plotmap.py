#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import threading
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import os
import message_filters
import time

class PlotMapNode(Node):
    def __init__(self):
        super().__init__('plotmap')
        self.get_logger().info('PlotMap node has been initialized.')

        self.saving_data = False
        
        # File handlers for streaming data
        self.combined_file = None
        
        # Docking related data
        self.docking_events = []
        self.docking_started = False
        self.docking_start_time = None
        
        # Use QoS profiles to reduce message frequency
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Create synchronized subscribers
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/odom', qos_profile=qos)
        self.pose_sub = message_filters.Subscriber(self, Odometry, '/base_pose_ground_truth', qos_profile=qos)
        
        # Time synchronizer - synchronize messages with 0.01 seconds tolerance
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.pose_sub], 
            queue_size=10, 
            slop=0.01)
        self.ts.registerCallback(self.synchronized_callback)
        
        # Create services
        self.create_service(Trigger, '/start_plot', self.start_plot_callback)
        self.create_service(Trigger, '/stop_plot', self.stop_plot_callback)
        
        # Subscribe to docking events
        self.docking_start_sub = self.create_subscription(
            Bool, '/aruco_docking_start', self.docking_start_callback, 10)
        self.docking_done_sub = self.create_subscription(
            Bool, '/aruco_idone', self.docking_done_callback, 10)
            
        # Also monitor the docking status service calls
        self.docking_client = self.create_client(SetBool, '/docking_aruco_status')

    def docking_start_callback(self, msg):
        """Called when docking is initiated"""
        if msg.data and not self.docking_started:
            self.docking_started = True
            self.docking_start_time = time.time()
            self.get_logger().info("Docking started")

    def docking_done_callback(self, msg):
        """Called when docking is completed"""
        if msg.data and self.docking_started:
            docking_end_time = time.time()
            docking_duration = docking_end_time - self.docking_start_time
            
            # Create docking event record
            docking_event = {
                'start_time': self.docking_start_time,
                'end_time': docking_end_time,
                'duration': docking_duration,
                'successful': True
            }
            
            # Add to docking events list
            self.docking_events.append(docking_event)
            
            # Log information
            self.get_logger().info(f"Docking completed in {docking_duration:.2f} seconds")
            
            # Reset docking state
            self.docking_started = False
            self.docking_start_time = None
            
            # If we're saving data, write this event to the file as well
            if self.saving_data and self.combined_file:
                docking_record = {
                    'stamp': docking_end_time,
                    'event_type': 'docking_complete',
                    'duration': docking_duration
                }
                self.combined_file.write(json.dumps(docking_record) + '\n')
                self.combined_file.flush()

    def start_plot_callback(self, request, response):
        # Create output directory if it doesn't exist
        os.makedirs('plot_data', exist_ok=True)
        
        # Open file for streaming synchronized data
        self.combined_file = open('plot_data/combined_data.jsonl', 'w')
        
        self.saving_data = True
        self.get_logger().info("Started plotting.")
        response.success = True
        response.message = "Plotting started."
        return response

    def stop_plot_callback(self, request, response):
        self.saving_data = False
        
        # Close the file
        if self.combined_file:
            self.combined_file.close()
            
        # Process the file in background
        threading.Thread(target=self.process_data_file).start()
        
        self.get_logger().info("Stopped plotting. Processing data in background...")
        response.success = True
        response.message = "Plotting stopped and data processing initiated."
        return response

    def synchronized_callback(self, odom_msg, pose_msg):
        """Process synchronized messages from both topics"""
        if not self.saving_data or not self.combined_file:
            return
            
        # Get timestamp from one message (they should be nearly identical)
        current_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        
        # Prepare synchronized data record
        data = {
            'stamp': current_time,
            'event_type': 'position_update',
            'odom': {
                'x': odom_msg.pose.pose.position.x,
                'y': odom_msg.pose.pose.position.y,
                'z': odom_msg.pose.pose.position.z,
                'qx': odom_msg.pose.pose.orientation.x,
                'qy': odom_msg.pose.pose.orientation.y,
                'qz': odom_msg.pose.pose.orientation.z,
                'qw': odom_msg.pose.pose.orientation.w,
                'vx': odom_msg.twist.twist.linear.x,
                'vy': odom_msg.twist.twist.linear.y,
                'vz': odom_msg.twist.twist.linear.z,
                'wx': odom_msg.twist.twist.angular.x,
                'wy': odom_msg.twist.twist.angular.y,
                'wz': odom_msg.twist.twist.angular.z,
            },
            'ground_truth': {
                'x': pose_msg.pose.pose.position.x,
                'y': pose_msg.pose.pose.position.y,
                'z': pose_msg.pose.pose.position.z,
                'qx': pose_msg.pose.pose.orientation.x,
                'qy': pose_msg.pose.pose.orientation.y,
                'qz': pose_msg.pose.pose.orientation.z,
                'qw': pose_msg.pose.pose.orientation.w,
                'vx': pose_msg.twist.twist.linear.x,
                'vy': pose_msg.twist.twist.linear.y,
                'vz': pose_msg.twist.twist.linear.z,
                'wx': pose_msg.twist.twist.angular.x,
                'wy': pose_msg.twist.twist.angular.y,
                'wz': pose_msg.twist.twist.angular.z,
            }
        }
        
        # Write directly to file
        self.combined_file.write(json.dumps(data) + '\n')
        
        # Flush occasionally to ensure data is written
        if hasattr(self, 'sync_count'):
            self.sync_count += 1
        else:
            self.sync_count = 1
            
        if self.sync_count % 100 == 0:
            self.combined_file.flush()

    def process_data_file(self):
        """Process the JSONL file into a JSON file"""
        data_records = []
        docking_records = []
        
        # Read data from file
        try:
            with open('plot_data/combined_data.jsonl', 'r') as f:
                for line in f:
                    if line.strip():
                        record = json.loads(line)
                        if record.get('event_type') == 'position_update':
                            data_records.append(record)
                        elif record.get('event_type') == 'docking_complete':
                            docking_records.append(record)
        except Exception as e:
            self.get_logger().error(f"Error reading data: {str(e)}")
            
        # Organize data
        processed_data = {
            'timestamps': [record['stamp'] for record in data_records],
            'odom': [record['odom'] for record in data_records],
            'base_pose_ground_truth': [record['ground_truth'] for record in data_records],
            'docking_events': docking_records
        }
        
        # Save processed data
        with open('plot_data.json', 'w') as file:
            json.dump(processed_data, file)
            
        self.get_logger().info("Data processed and saved to plot_data.json.")

def main(args=None):
    rclpy.init(args=args)
    node = PlotMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure to close any open files
        if hasattr(node, 'combined_file') and node.combined_file:
            node.combined_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()