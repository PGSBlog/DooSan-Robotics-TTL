#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ttb_gui.action import Delivery  # Delivery.action 파일이 정의된 액션
import tkinter as tk
from threading import Thread
import time

class DeliveryServer(Node):
    def __init__(self):
        super().__init__('delivery_server')
        
        # Action 서버 생성
        self._action_server = ActionServer(
            self,
            Delivery,
            'start_delivery',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # 요청된 목표 메시지를 수락
        destination = goal_handle.request.destination
        self.get_logger().info(f"Received delivery request for: {destination}")

        # 피드백을 통해 진행상황 전송
        for i in range(1, 6):
            feedback_msg = Delivery.Feedback()
            feedback_msg.status = f"{destination} 배달 중... {i*20}% 완료"
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.status)
            
            # 작업 수행 중 대기
            time.sleep(1)

        # 최종 결과 설정
        result = Delivery.Result()
        result.status = f"{destination}에 배달 완료"
        self.get_logger().info(result.status)

        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init(args=args)
    server = DeliveryServer()

    # Tkinter GUI
    root = tk.Tk()
    root.title("ROS 2 Delivery Server")

    label = tk.Label(root, text="Delivery Server Running...", font=("Arial", 16))
    label.pack(pady=20)

    def quit_callback():
        rclpy.shutdown()
        root.destroy()

    quit_button = tk.Button(root, text="Quit", command=quit_callback)
    quit_button.pack(pady=20)

    server_thread = Thread(target=rclpy.spin, args=(server,), daemon=True)
    server_thread.start()

    root.mainloop()

if __name__ == '__main__':
    main()
