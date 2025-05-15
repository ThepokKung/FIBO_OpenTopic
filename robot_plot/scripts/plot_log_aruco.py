#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import message_filters
import json

class ArUcoDockingLog(Node):
    def __init__(self):
        super().__init__('aruco_docking_log_postion_done')

        qos = rclpy.qos.qos_profile_sensor_data

        # Subscribers for odom and ground truth using message_filters
        self.odom_sub = message_filters.Subscriber(self, Odometry, '/odom', qos_profile=qos)
        self.pose_sub = message_filters.Subscriber(self, Odometry, '/base_pose_ground_truth', qos_profile=qos)

        # Synchronize odom and ground truth topics
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.pose_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # Store the latest ground truth pose for publishing and logging
        self.last_ground_truth = None

        # Subscriber for ArUco done topic (expects Bool messages)
        self.aruco_idone_sub = self.create_subscription(Bool, '/aruco_idone', self.aruco_idone_callback, 10)

        # Publisher for ArUco done Position
        self.aruco_done_pub = self.create_publisher(Odometry, 'ArUco_done_Position', 10)

        # Initialize log entries and message id counter.
        self.log_entries = []
        self.entry_id = 0

    def sync_callback(self, odom_msg, ground_truth_msg):
        # Store the latest ground truth pose from the synchronized callback
        self.last_ground_truth = ground_truth_msg

    def aruco_idone_callback(self, msg):
        # If aruco is done and a valid ground truth exists, publish it and save to JSON log
        if msg.data and self.last_ground_truth is not None:
            self.aruco_done_pub.publish(self.last_ground_truth)
            self.get_logger().info('Published ArUco done position')

            # Prepare data from the ground truth message including an incremental id
            data = {
                "id": self.entry_id,
                "position": {
                    "x": self.last_ground_truth.pose.pose.position.x,
                    "y": self.last_ground_truth.pose.pose.position.y,
                    "z": self.last_ground_truth.pose.pose.position.z
                },
                "orientation": {
                    "x": self.last_ground_truth.pose.pose.orientation.x,
                    "y": self.last_ground_truth.pose.pose.orientation.y,
                    "z": self.last_ground_truth.pose.pose.orientation.z,
                    "w": self.last_ground_truth.pose.pose.orientation.w
                }
            }
            self.entry_id += 1

            # Append the new log entry to the list
            self.log_entries.append(data)

            # Save all log entries to a JSON file.
            try:
                with open("log_of_arUco_position.json", "w") as f:
                    json.dump(self.log_entries, f, indent=2)
                self.get_logger().info('Saved log to log_of_arUco_position.json')
            except Exception as e:
                self.get_logger().error(f'Failed to write JSON file: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDockingLog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()