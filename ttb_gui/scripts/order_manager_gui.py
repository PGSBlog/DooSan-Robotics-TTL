#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading
from datetime import datetime

class DeliveryApp(Node):
    def __init__(self):
        super().__init__('delivery_app')
        
        # ROS2 Subscriber 생성
        self.subscription = self.create_subscription(
            String,
            'order_topic',
            self.listener_callback,
            10
        )

        # ROS2 Publisher 생성
        self.delivery_publisher = self.create_publisher(String, 'table_topic', 10)
        self.kitchen_publisher = self.create_publisher(String, 'kitchen', 10)

        # GUI 초기화
        self.root = tk.Tk()
        self.root.title("ROS2 Delivery Manager")
        self.root.geometry("800x400")

        # 메인 프레임 구성
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(1, weight=1)
        self.main_frame.grid_columnconfigure(0, weight=1)

        # 상태 표시 레이블
        self.status_var = tk.StringVar(value="Waiting for messages...")
        self.status_label = ttk.Label(self.main_frame, textvariable=self.status_var)
        self.status_label.grid(row=0, column=0, columnspan=2, pady=(0, 5), sticky="w")

        # 주문 리스트와 완료 리스트 레이블
        ttk.Label(self.main_frame, text="Order List").grid(row=1, column=0, sticky="w")
        ttk.Label(self.main_frame, text="Completed Orders").grid(row=1, column=1, sticky="w")

        # 주문 리스트박스
        self.order_listbox = tk.Listbox(self.main_frame, height=15, width=40)
        self.order_listbox.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # 완료된 주문 리스트박스
        self.completed_listbox = tk.Listbox(self.main_frame, height=15, width=40)
        self.completed_listbox.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))

        # '배달 시작' 버튼
        self.start_button = ttk.Button(self.main_frame, text="Start Delivery", command=self.start_delivery)
        self.start_button.grid(row=3, column=0, pady=(5, 0), sticky="w")

        # '배달 완료' 버튼
        self.complete_button = ttk.Button(self.main_frame, text="Complete Delivery", command=self.complete_delivery)
        self.complete_button.grid(row=3, column=1, pady=(5, 0), sticky="e")

        # 주문 저장용 리스트 및 메시지 카운트 초기화
        self.orders = []
        self.message_count = 0

    def listener_callback(self, msg):
        """주문 수신 콜백 함수"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.message_count += 1
        order = f"[{timestamp}] Order {self.message_count}: {msg.data}"
        
        # 수신된 메시지를 콘솔에 로그
        self.get_logger().info(f'Received: "{msg.data}"')
        
        # 주문을 리스트에 추가하고 GUI를 업데이트
        self.orders.append(order)
        self.root.after(0, self.update_order_list, order)
        self.root.after(0, self.update_status)

    def update_order_list(self, order):
        """주문을 주문 리스트박스에 추가"""
        self.order_listbox.insert(tk.END, order)
        self.order_listbox.see(tk.END)  # 스크롤을 자동으로 하단으로 이동

    def update_status(self):
        """상태 표시 업데이트"""
        self.status_var.set(f"Received {self.message_count} messages")

    def start_delivery(self):
        """선택된 주문의 테이블 번호를 발행하고 주문 완료 리스트로 이동"""
        selected_index = self.order_listbox.curselection()
        if selected_index:
            selected_order = self.orders[selected_index[0]]
            table_info = selected_order.split(": ")[1]  # 'Table 1' 추출
            
            # 주문을 배달로 시작하며 table_topic으로 발행
            msg = String()
            msg.data = table_info
            self.delivery_publisher.publish(msg)
            self.get_logger().info(f"Publishing delivery topic for: {table_info}")

            # 주문 리스트에서 해당 주문 제거하고 완료 리스트로 이동
            self.order_listbox.delete(selected_index)
            self.completed_listbox.insert(tk.END, selected_order)
            self.orders.pop(selected_index[0])  # 내부 orders 리스트에서도 제거
        else:
            self.get_logger().info("No order selected for delivery")

    def complete_delivery(self):
        """'kitchen' 토픽으로 배달 완료 발행"""
        msg = String()
        msg.data = "kitchen"
        self.kitchen_publisher.publish(msg)
        self.get_logger().info("Publishing 'kitchen' topic for order completion")

    def run(self):
        """ROS2 스핀을 별도의 스레드에서 실행"""
        ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        ros_thread.start()
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = DeliveryApp()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
