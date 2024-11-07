#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from ttb_gui.action import Delivery  # Delivery.action 파일이 정의된 액션
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk   #, ImageResampling
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

        # 액션 클라이언트 생성
        self.delivery_action_client = ActionClient(self, Delivery, 'start_delivery')

        # GUI 초기화
        self.root = tk.Tk()
        self.root.title("ROS2 Delivery Manager")
        self.root.geometry("700x800")

        # 메인 프레임 구성
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # 상단 주문 및 완료 리스트 프레임
        list_frame = ttk.Frame(self.main_frame)
        list_frame.grid(row=0, column=0, pady=(0, 10), sticky="ew")
        
        # 주문 리스트
        ttk.Label(list_frame, text="Order List").grid(row=0, column=0, sticky="w")
        self.order_listbox = tk.Listbox(list_frame, height=10, width=40)
        self.order_listbox.grid(row=1, column=0, padx=(0, 10), sticky="ns")

        # 완료 리스트
        ttk.Label(list_frame, text="   Completed Orders").grid(row=0, column=1, sticky="w")
        self.completed_listbox = tk.Listbox(list_frame, height=10, width=40)
        self.completed_listbox.grid(row=1, column=1, padx=(10, 0), sticky="ns")

        # 버튼 프레임
        button_frame = ttk.Frame(self.main_frame)
        button_frame.grid(row=1, column=0, pady=(10, 10), sticky="ew")
        self.start_button = ttk.Button(button_frame, text="Start Delivery", command=self.start_delivery)
        self.start_button.grid(row=0, column=0, padx=5)
        self.complete_button = ttk.Button(button_frame, text="Complete Delivery", command=self.complete_delivery)
        self.complete_button.grid(row=0, column=1, padx=5)

        # 맵 이미지 표시
        map_frame = ttk.Frame(self.main_frame)
        map_frame.grid(row=2, column=0, pady=(10, 10))
        ttk.Label(map_frame, text="Map").pack()
        self.load_map_image(map_frame)

        # 피드백과 상태 표시
        status_frame = ttk.Frame(self.main_frame)
        status_frame.grid(row=3, column=0, pady=(10, 0))
        
        self.status_var = tk.StringVar(value="Waiting for messages...")
        self.status_label = ttk.Label(status_frame, textvariable=self.status_var, font=("Arial", 12, "bold"), foreground="blue")
        self.status_label.pack(pady=(0, 5))
        
        self.feedback_var = tk.StringVar(value="Feedback: None")
        self.feedback_label = ttk.Label(status_frame, textvariable=self.feedback_var, font=("Arial", 12, "bold"), foreground="blue")
        self.feedback_label.pack()

        # 주문 리스트 및 메시지 카운트 초기화
        self.orders = []
        self.message_count = 0

    def load_map_image(self, frame):
        """맵 이미지를 불러와서 표시"""
        image_path = '/home/ye2202/p1_turtlebot_ws/src/ttb_gui/map/map.pgm'  # 경로는 .png로 지정
        image = Image.open(image_path)
        image = image.resize((400, 400), Image.LANCZOS)  # 이미지 크기 조정
        self.map_image = ImageTk.PhotoImage(image)
        self.map_label = tk.Label(frame, image=self.map_image)
        self.map_label.pack()

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
        """선택된 주문의 배달을 액션으로 요청"""
        selected_index = self.order_listbox.curselection()
        if selected_index:
            selected_order = self.orders[selected_index[0]]
            order_info = selected_order.split(": ")[1]  # 주문 정보 추출

            # 배달 목표 설정
            goal_msg = Delivery.Goal()
            goal_msg.destination = order_info  # destination을 액션 목표로 설정

            # 배달 시작
            self.get_logger().info(f"Starting delivery for: {order_info}")
            self.delivery_action_client.wait_for_server()
            send_goal_future = self.delivery_action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """피드백 수신 콜백"""
        feedback_text = f"피드백: {feedback_msg.feedback.status}"
        self.feedback_var.set(feedback_text)

    def goal_response_callback(self, future):
        """배달 시작 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status_var.set("배달 요청이 거부되었습니다.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, result_future):
        """배달 완료 결과 수신 콜백"""
        result = result_future.result().result
        self.status_var.set(f"배달 결과: {result.status}")

        # 완료된 주문을 완료 리스트에 추가
        self.completed_listbox.insert(tk.END, result.status)

        # 주문 리스트에서 완료된 주문 삭제
        selected_index = self.order_listbox.curselection()
        if selected_index:
            self.order_listbox.delete(selected_index[0])
            self.orders.pop(selected_index[0])  # 내부 orders 리스트에서도 제거

    def complete_delivery(self):
        """주방 목적지로 배달 완료 발행"""
        goal_msg = Delivery.Goal()
        goal_msg.destination = "kitchen"  # 'kitchen'을 목적지로 설정

        self.get_logger().info("Starting delivery for kitchen")
        send_goal_future = self.delivery_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

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
