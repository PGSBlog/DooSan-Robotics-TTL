#!/usr/bin/env python3
import sys
import threading
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NavigationGUI(Node):
    def __init__(self):
        super().__init__('navigation_gui')
        
        # GUI 설정
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("Service Robot Control")
        self.window.resize(400, 300)
        
        # 중앙 위젯 설정
        self.central_widget = QWidget()
        self.window.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)
        
        # 테이블 번호 선택
        self.table_combo = QComboBox()
        self.table_combo.addItems([str(i) for i in range(1, 10)])
        self.layout.addWidget(QLabel("Select Table:"))
        self.layout.addWidget(self.table_combo)
        
        # 버튼들
        self.table_button = QPushButton("테이블 번호 전송")
        self.table_button.clicked.connect(self.send_table_number)
        self.layout.addWidget(self.table_button)
        
        self.cook_button = QPushButton("조리 완료")
        self.cook_button.clicked.connect(self.send_make_finish)
        self.layout.addWidget(self.cook_button)
        
        self.delivery_button = QPushButton("배달 완료")
        self.delivery_button.clicked.connect(self.send_delivery_finish)
        self.layout.addWidget(self.delivery_button)
        
        # 상태 표시 텍스트 브라우저
        self.text_browser = QTextBrowser()
        self.layout.addWidget(self.text_browser)
        
        # ROS2 퍼블리셔 설정
        self.command_publisher = self.create_publisher(String, 'delivery_command', 10)
        
        # 피드백 구독자 설정
        self.feedback_subscriber = self.create_subscription(
            String,
            'delivery_feedback',
            self.feedback_callback,
            10)

    def send_table_number(self):
        msg = String()
        table_num = self.table_combo.currentText()
        msg.data = f"{table_num},선택"
        self.command_publisher.publish(msg)
        self.text_browser.append(f"테이블 번호 전송: {table_num}")

    def send_make_finish(self):
        msg = String()
        table_num = self.table_combo.currentText()
        msg.data = f"{table_num},조리완료"
        self.command_publisher.publish(msg)
        self.text_browser.append("조리 완료 신호 전송")

    def send_delivery_finish(self):
        msg = String()
        table_num = self.table_combo.currentText()
        msg.data = f"{table_num},배달완료"
        self.command_publisher.publish(msg)
        self.text_browser.append("배달 완료 신호 전송")

    def feedback_callback(self, msg):
        self.text_browser.append(f"피드백: {msg.data}")

    def run(self):
        # ROS2 스핀을 별도 쓰레드로 실행
        self.ros_thread = threading.Thread(target=lambda: rclpy.spin(self))
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # GUI 실행
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
