#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class TableNavigationNode(Node):
    def __init__(self):
        super().__init__('table_navigation_node')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF 리스너 추가
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 메시지 구독자
        self.command_subscriber = self.create_subscription(
            String,
            'delivery_command',
            self.command_callback,
            10)
            
        # 테이블 위치 정의
        self.table_positions = {
            '1': {'x': -2.0, 'y': 2.0, 'theta': 0.0},
            '2': {'x': 0.0, 'y': 2.0, 'theta': 0.0},
            '3': {'x': 2.0, 'y': 2.0, 'theta': 0.0},
            '4': {'x': -2.0, 'y': 0.0, 'theta': 0.0},
            '5': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            '6': {'x': 2.0, 'y': 0.0, 'theta': 0.0},
            '7': {'x': -2.0, 'y': -2.0, 'theta': 0.0},
            '8': {'x': 0.0, 'y': -2.0, 'theta': 0.0},
            '9': {'x': 2.0, 'y': -2.0, 'theta': 0.0}
        }
        
        # 시작 위치 정의
        self.start_position = {'x': 0.0, 'y': -3.0, 'theta': 0.0}
        
        # 피드백 발행자
        self.feedback_publisher = self.create_publisher(String, 'delivery_feedback', 10)
        
        self.get_logger().info('Table Navigation Node has been started')

    def command_callback(self, msg):
        command_parts = msg.data.split(',')
        if len(command_parts) == 2:
            table_num, command = command_parts
            
            if command == "선택":
                self.get_logger().info(f'Selected table {table_num}')
                feedback = String()
                feedback.data = f"테이블 {table_num}이 선택되었습니다."
                self.feedback_publisher.publish(feedback)
            elif command == "조리완료":
                self.get_logger().info(f'Moving to table {table_num}')
                feedback = String()
                feedback.data = f"테이블 {table_num}으로 이동을 시작합니다."
                self.feedback_publisher.publish(feedback)
                # 테이블로 이동
                self.navigate_to_table(table_num)
            elif command == "배달완료":
                self.get_logger().info('Returning to start position')
                feedback = String()
                feedback.data = "시작 위치로 돌아갑니다."
                self.feedback_publisher.publish(feedback)
                self.return_to_start()

    def create_pose_stamped(self, position):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(position['x'])
        pose.pose.position.y = float(position['y'])
        pose.pose.position.z = 0.0
        
        # Quaternion으로 변환
        yaw = float(position['theta'])
        pose.pose.orientation.w = math.cos(yaw / 2)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        
        return pose

    def navigate_to_table(self, table_number):
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(self.table_positions[table_number])

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def return_to_start(self):
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(self.start_position)

        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            feedback = String()
            feedback.data = "이동 요청이 거부되었습니다."
            self.feedback_publisher.publish(feedback)
            return

        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

        # 이동 중 피드백
        feedback = String()
        feedback.data = "로봇이 이동 중입니다."
        self.feedback_publisher.publish(feedback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation completed')
        feedback = String()
        feedback.data = "목표 지점에 도착했습니다."
        self.feedback_publisher.publish(feedback)

def main():
    rclpy.init()
    node = TableNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
