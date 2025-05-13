#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

class PlotMapNode(Node):
    def __init__(self):
        super().__init__('plotmap')
        self.get_logger().info('PlotMap node has been initialized.')

        self.saving_data = False
        self.odom_data = []
        self.pose_data = []
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/base_pose_ground_truth', self.pose_callback, 10)
        self.create_service(Trigger, '/start_plot', self.start_plot_callback)
        self.create_service(Trigger, '/stop_plot', self.stop_plot_callback)

    def start_plot_callback(self, request, response):
        self.saving_data = True
        self.odom_data = []
        self.pose_data = []
        self.get_logger().info("Started plotting.")
        response.success = True
        response.message = "Plotting started."
        return response

    def stop_plot_callback(self, request, response):
        self.saving_data = False
        self.save_data_to_yaml()
        self.get_logger().info("Stopped plotting and saved data.")
        response.success = True
        response.message = "Plotting stopped and data saved."
        return response

    def save_data_to_yaml(self):
        data = {
            'odom': self.odom_data,
            'base_pose_ground_truth': self.pose_data
        }
        with open('plot_data.yaml', 'w') as file:
            yaml.dump(data, file)
        self.get_logger().info("Data saved to plot_data.yaml.")

    def odom_callback(self, msg):
        if self.saving_data:
            self.odom_data.append({
                'header': {
                    'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'frame_id': msg.header.frame_id
                },
                'pose': {
                    'position': {
                        'x': msg.pose.pose.position.x,
                        'y': msg.pose.pose.position.y,
                        'z': msg.pose.pose.position.z,
                    },
                    'orientation': {
                        'x': msg.pose.pose.orientation.x,
                        'y': msg.pose.pose.orientation.y,
                        'z': msg.pose.pose.orientation.z,
                        'w': msg.pose.pose.orientation.w,
                    }
                },
                'twist': {
                    'linear': {
                        'x': msg.twist.twist.linear.x,
                        'y': msg.twist.twist.linear.y,
                        'z': msg.twist.twist.linear.z,
                    },
                    'angular': {
                        'x': msg.twist.twist.angular.x,
                        'y': msg.twist.twist.angular.y,
                        'z': msg.twist.twist.angular.z,
                    }
                }
            })

    def pose_callback(self, msg):
        if self.saving_data:
            self.pose_data.append({
                'header': {
                    'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'frame_id': msg.header.frame_id
                },
                'pose': {
                    'position': {
                        'x': msg.pose.pose.position.x,
                        'y': msg.pose.pose.position.y,
                        'z': msg.pose.pose.position.z,
                    },
                    'orientation': {
                        'x': msg.pose.pose.orientation.x,
                        'y': msg.pose.pose.orientation.y,
                        'z': msg.pose.pose.orientation.z,
                        'w': msg.pose.pose.orientation.w,
                    }
                }
            })

def main(args=None):
    rclpy.init(args=args)
    node = PlotMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()