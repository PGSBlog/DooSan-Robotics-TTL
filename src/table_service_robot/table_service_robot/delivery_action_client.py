#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from table_service_robot.action import Delivery
import sys

class DeliveryActionClient(Node):
    def __init__(self):
        super().__init__('delivery_action_client')
        self._action_client = ActionClient(self, Delivery, 'delivery_service')
        self.get_logger().info('Delivery Action Client has been started')

    def send_goal(self, table_number, command):
        # 서버가 실행될 때까지 대기
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # 목표 생성
        goal_msg = Delivery.Goal()
        goal_msg.table_number = table_number
        goal_msg.command = command

        self.get_logger().info('Sending goal request...')

        # 목표 전송 및 피드백 콜백 설정
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_status}')

def main(args=None):
    rclpy.init(args=args)
    action_client = DeliveryActionClient()

    while True:
        print("\nDelivery Action Client Menu:")
        print("1. Send table number and make_finish command")
        print("2. Send delivery_finish command")
        print("3. Exit")
        
        choice = input("Enter your choice (1-3): ")

        if choice == '1':
            try:
                table_num = int(input("Enter table number (1-9): "))
                if 1 <= table_num <= 9:
                    action_client.send_goal(table_num, 'make_finish')
                    rclpy.spin_once(action_client)
                else:
                    print("Invalid table number!")
            except ValueError:
                print("Please enter a valid number!")

        elif choice == '2':
            action_client.send_goal(0, 'delivery_finish')
            rclpy.spin_once(action_client)

        elif choice == '3':
            break

        else:
            print("Invalid choice! Please try again.")

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
