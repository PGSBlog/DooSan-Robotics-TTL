#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from ttb_gui.action import Message
import tkinter as tk
from threading import Thread
import time

class Server(Node):
    def __init__(self):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            Message,
            'exchange_message',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # 목표 메시지를 수락하고 로깅
        self.get_logger().info(f"Received goal: {goal_handle.request.message}")

        # 목표를 수락했다고 알림
        goal_handle.succeed()
        
        # 작업 수행 단계와 피드백 전송
        for i in range(1, 5):
            # 피드백 메시지 생성 및 전송
            feedback_msg = Message.Feedback()
            feedback_msg.feedback = f"Processing step {i} for message '{goal_handle.request.message}'"
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(feedback_msg.feedback)
            
            # 잠시 대기 (작업 수행 흉내)
            time.sleep(1)

        # 작업 완료 후 최종 결과 설정
        result = Message.Result()
        result.message = f"Successfully processed message '{goal_handle.request.message}'"
        self.get_logger().info(result.message)
        
        return result  # 최종 결과 반환

def main(args=None):
    rclpy.init(args=args)
    server = Server()

    # Tkinter GUI
    root = tk.Tk()
    root.title("ROS 2 Action Server")

    label = tk.Label(root, text="Action Server Running...", font=("Arial", 16))
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
