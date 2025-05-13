#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import RobotLiftUpdate, RobotLiftCheck
from std_srvs.srv import SetBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class LiftController(Node):
    def __init__(self):
        super().__init__('lift_controller')

        # Robot lift status
        self.lift_status = False

        self.lift_service = self.create_service(SetBool, 'lift_on', self.lift_on_callback)

        #Create a service client for checking robot station
        self.check_lift_status_cbg = MutuallyExclusiveCallbackGroup()
        self.update_lift_status_cbg = MutuallyExclusiveCallbackGroup()
        # self.robot_lift_check = self.create_client(RobotLiftCheck, '/check_lift_status',callback_group=self.check_lift_status_cbg)
        self.robot_lift_update = self.create_client(RobotLiftUpdate, '/update_lift_status',callback_group=self.update_lift_status_cbg)

        # lofging
        self.get_logger().info("Lift Controller Node has been started.")

    def lift_on_callback(self, request, response):
        # Update the local lift status using the SetBool request.
        self.lift_status = request.data

        # Create a request for the update service.
        update_request = RobotLiftUpdate.Request()
        update_request.lift_status = self.lift_status

        # Wait for the /update_lift_status service to become available.
        if not self.robot_lift_update.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("RobotLiftUpdate service is not available")
        else:
            # Call the update service.
            future = self.robot_lift_update.call_async(update_request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is None or not future.result().success:
                self.get_logger().error("Failed to update lift status via update service")

        response.success = True
        response.message = f"Lift turned {'on' if self.lift_status else 'off'}."
        return response

    def update_lift_status_callback(self, request, response):
        # Assuming RobotLiftUpdate request has a 'status' field.
        self.lift_status = request.lift_status
        response.success = True
        response.message = f"Lift status updated to {'on' if self.lift_status else 'off'}."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LiftController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()