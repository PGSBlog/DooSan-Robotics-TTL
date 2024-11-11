#!/usr/bin/env python3
import sys
import threading
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
from datetime import datetime
from ros_study_msgs.srv import OrderReview

class NavigationGUI(Node):
    def __init__(self):
        super().__init__('navigation_gui')
        
        # GUI 설정
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("Restaurant Control Center")
        self.window.resize(1200, 800)
        
        # 중앙 위젯 설정
        self.central_widget = QWidget()
        self.window.setCentralWidget(self.central_widget)
        self.main_layout = QHBoxLayout(self.central_widget)
        
        # 좌측 패널 (주문 관리)
        self.left_panel = QVBoxLayout()
        self.main_layout.addLayout(self.left_panel)
        
        # 주문 목록
        self.order_group = QGroupBox("Incoming Orders")
        order_layout = QVBoxLayout()
        self.order_list = QListWidget()
        order_layout.addWidget(self.order_list)
        self.order_group.setLayout(order_layout)
        self.left_panel.addWidget(self.order_group)
        
        # 주문 처리 버튼
        button_layout = QHBoxLayout()
        self.accept_button = QPushButton("Accept Order")
        self.accept_button.clicked.connect(self.accept_order)
        self.reject_button = QPushButton("Reject Order")
        self.reject_button.clicked.connect(self.reject_order)
        button_layout.addWidget(self.accept_button)
        button_layout.addWidget(self.reject_button)
        self.left_panel.addLayout(button_layout)
        
        # 우측 패널 (네비게이션 제어)
        self.right_panel = QVBoxLayout()
        self.main_layout.addLayout(self.right_panel)
        
        # 테이블 선택
        self.table_group = QGroupBox("Table Control")
        table_layout = QVBoxLayout()
        self.table_combo = QComboBox()
        self.table_combo.addItems([str(i) for i in range(1, 10)])
        table_layout.addWidget(QLabel("Select Table:"))
        table_layout.addWidget(self.table_combo)
        self.table_group.setLayout(table_layout)
        self.right_panel.addWidget(self.table_group)
        
        # 네비게이션 버튼
        nav_button_layout = QVBoxLayout()
        self.start_nav_button = QPushButton("Start Delivery")
        self.start_nav_button.clicked.connect(self.start_delivery)
        self.return_button = QPushButton("Return to Kitchen")
        self.return_button.clicked.connect(self.return_to_kitchen)
        nav_button_layout.addWidget(self.start_nav_button)
        nav_button_layout.addWidget(self.return_button)
        self.right_panel.addLayout(nav_button_layout)
        
        # 상태 디스플레이
        self.status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        self.status_display = QTextBrowser()
        status_layout.addWidget(self.status_display)
        self.status_group.setLayout(status_layout)
        self.right_panel.addWidget(self.status_group)
        
        # ROS2 설정
        # OrderReview 서비스 서버
        self.srv = self.create_service(OrderReview, 'order_review', self.review_order_callback)
        
        # 네비게이션 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 현재 처리 중인 주문 정보
        self.current_order = None
        self.current_response = None
        self.response_ready = threading.Event()
        
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

    def review_order_callback(self, request, response):
        """주문 리뷰 콜백"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        order_info = f"[{timestamp}] New Order: {request.menu_name}"
        self.order_list.addItem(order_info)
        self.status_display.append(f"New order received: {order_info}")
        
        self.current_order = request
        self.response_ready.clear()
        self.response_ready.wait()
        
        response.is_approved = self.current_response
        response.message = "Order accepted" if self.current_response else "Order rejected"
        return response

    def accept_order(self):
        """주문 승인"""
        if not self.order_list.currentItem():
            self.status_display.append("Please select an order first")
            return
            
        self.current_response = True
        self.response_ready.set()
        self.status_display.append("Order accepted")
        
        # 테이블 번호 추출 및 설정
        order_text = self.order_list.currentItem().text()
        if "Table" in order_text:
            table_num = order_text.split("Table")[1].split()[0]
            self.table_combo.setCurrentText(table_num)
            
        self.order_list.takeItem(self.order_list.currentRow())

    def reject_order(self):
        """주문 거절"""
        if not self.order_list.currentItem():
            self.status_display.append("Please select an order first")
            return
            
        self.current_response = False
        self.response_ready.set()
        self.status_display.append("Order rejected")
        self.order_list.takeItem(self.order_list.currentRow())

    def start_delivery(self):
        """배달 시작"""
        table_num = self.table_combo.currentText()
        self.navigate_to_table(table_num)
        self.status_display.append(f"Starting delivery to table {table_num}")

    def return_to_kitchen(self):
        """주방으로 복귀"""
        self.return_to_start()
        self.status_display.append("Returning to kitchen")

    def create_pose_stamped(self, position):
        """PoseStamped 메시지 생성"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(position['x'])
        pose.pose.position.y = float(position['y'])
        pose.pose.position.z = 0.0
        
        yaw = float(position['theta'])
        pose.pose.orientation.w = math.cos(yaw / 2)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        
        return pose

    def navigate_to_table(self, table_number):
        """테이블로 이동"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.status_display.append("Navigation server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(self.table_positions[table_number])

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def return_to_start(self):
        """시작 위치로 복귀"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.status_display.append("Navigation server not available")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped(self.start_position)

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """네비게이션 목표 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status_display.append('Navigation goal rejected')
            return

        self.status_display.append('Navigation goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """네비게이션 결과 콜백"""
        try:
            result = future.result().result
            self.status_display.append('Navigation completed')
        except Exception as e:
            self.status_display.append(f'Navigation failed: {str(e)}')

    def run(self):
        """GUI 실행"""
        self.ros_thread = threading.Thread(target=lambda: rclpy.spin(self))
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        self.window.show()
        sys.exit(self.app.exec_())

def main():
    rclpy.init()
    gui = NavigationGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
