#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ttb_gui.action import Message
import tkinter as tk
from threading import Thread

class Client(Node):
    def __init__(self):
        super().__init__('action_client')
        self._action_client = ActionClient(self, Message, 'exchange_message')

    def send_goal(self, input_message):
        goal_msg = Message.Goal()
        goal_msg.message = input_message
        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.feedback_label.config(text=f"Feedback: {feedback_msg.feedback.feedback}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.result_label.config(text="Goal rejected :(")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, result_future):
        result = result_future.result().result
        self.result_label.config(text=f"Result: {result.message}")

def main(args=None):
    rclpy.init(args=args)
    client = Client()

    # Tkinter GUI
    root = tk.Tk()
    root.title("ROS 2 Action Client")

    input_frame = tk.Frame(root)
    input_frame.pack(pady=20)

    input_label = tk.Label(input_frame, text="Enter Message:", font=("Arial", 12))
    input_label.pack(side=tk.LEFT)

    input_entry = tk.Entry(input_frame, font=("Arial", 12), width=50)
    input_entry.pack(side=tk.LEFT)

    send_button = tk.Button(root, text="Send Goal", command=lambda: client.send_goal(input_entry.get()))
    send_button.pack(pady=10)

    client.feedback_label = tk.Label(root, text="Feedback: None", font=("Arial", 14))
    client.feedback_label.pack(pady=10)

    client.result_label = tk.Label(root, text="Result: None", font=("Arial", 14))
    client.result_label.pack(pady=10)

    def quit_callback():
        rclpy.shutdown()
        root.destroy()

    quit_button = tk.Button(root, text="Quit", command=quit_callback)
    quit_button.pack(pady=20)

    client_thread = Thread(target=rclpy.spin, args=(client,), daemon=True)
    client_thread.start()

    root.mainloop()

if __name__ == '__main__':
    main()
