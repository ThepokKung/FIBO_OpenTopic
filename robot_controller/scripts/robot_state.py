#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import RobotStateUpdate,RobotStateCheck,RobotStationCheck,RobotStationUpdate,RobotLiftCheck,RobotLiftUpdate

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')
        
        # Robot's current station (default to None)
        self.current_state = "None"
        self.current_station = "None"
        
        # Status of the robot
        self.lift_status = False

        # Service to update the robot's station
        self.robot_state_update = self.create_service(RobotStateUpdate,'update_robot_state',self.update_robot_state_callback) 
        self.robot_state_check = self.create_service(RobotStateCheck,'check_robot_state',self.check_robot_state_callback)

        self.robot_station_update = self.create_service(RobotStationUpdate,'update_robot_station',self.update_station_callback)
        self.robot_station_check = self.create_service(RobotStationCheck,'check_robot_station',self.check_station_callback)

        # Service for update robot status
        self.robot_lift_update = self.create_service(RobotLiftUpdate, 'update_lift_status', self.robot_lift_update_callback)
        self.robot_lift_check = self.create_service(RobotLiftCheck, 'check_lift_status', self.robot_lift_check_callback)

        self.get_logger().info("Robot State Node is running...")

    def update_robot_state_callback(self, request, response):
        """
        Callback function to handle station updates.
        """
        self.current_state = request.state
        self.get_logger().info(f"Robot is state now at: {self.current_state}")
        
        # Return response
        response.success = True
        response.message = f"Updated state to {self.current_state}"
        return response

    def update_station_callback(self, request, response):
        """
        Callback function to handle station updates.
        """
        self.current_station = request.station
        self.get_logger().info(f"Robot is station now at: {self.current_station}")
        
        # Return response
        response.success = True
        response.message = f"Updated state to {self.current_state}"
        return response

    def check_robot_state_callback(self,request, response):
        """
        Callback function to check the robot's current state.
        """
        response.state = self.current_state
        response.success = True
        response.message = f"Robot is currently state at: {self.current_state}"
        self.get_logger().info(f"Robot is currently state at: {self.current_state}")
        return response

    def check_station_callback(self,request, response):
        """
        Callback function to check the robot's current state.
        """
        response.station = self.current_station
        response.success = True
        response.message = f"Robot is currently station at: {self.current_station}"
        self.get_logger().info(f"Robot is currently station at: '{self.current_station}'")
        return response
    
    def robot_lift_update_callback(self, request, response):
        """
        Callback function to update the robot's lift status.
        """
        self.lift_status = request.lift_status
        self.get_logger().info(f"Robot lift status updated to: {self.lift_status}")
        
        # Return response
        response.success = True
        response.message = f"Updated lift status to {self.lift_status}"
        return response
    
    def robot_lift_check_callback(self, request, response):
        """
        Callback function to check the robot's lift status.
        """
        response.lift_status = self.lift_status
        self.get_logger().info(f"Robot lift status is: {self.lift_status}")
        
        # Return response
        response.success = True
        response.message = f"Lift status is {self.lift_status}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
