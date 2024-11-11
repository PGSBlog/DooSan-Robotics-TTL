#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk
import threading

class PublisherGUI(Node):
    def __init__(self):
        super().__init__('publisher_gui')
        self.publisher = self.create_publisher(String, 'order_topic', 10)
        
        # Create the main window
        self.root = tk.Tk()
        self.root.title("ROS2 Publisher GUI")
        
        # Create and configure the main frame
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create input field
        self.message_var = tk.StringVar(value="table1")
        self.message_entry = ttk.Entry(self.main_frame, textvariable=self.message_var, width=40)
        self.message_entry.grid(row=0, column=0, padx=5, pady=5)
        
        # Create send button
        self.send_button = ttk.Button(self.main_frame, text="Send Message", command=self.publish_message)
        self.send_button.grid(row=0, column=1, padx=5, pady=5)
        
        # Status label
        self.status_var = tk.StringVar(value="Ready to send messages")
        self.status_label = ttk.Label(self.main_frame, textvariable=self.status_var)
        self.status_label.grid(row=1, column=0, columnspan=2, pady=5)

    def publish_message(self):
        msg = String()
        msg.data = self.message_var.get()
        self.publisher.publish(msg)
        self.status_var.set(f"Published: {msg.data}")
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def run(self):
        # Create a separate thread for ROS2 spinning
        self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
        self.ros_thread.start()
        
        # Start the Tkinter main loop
        self.root.mainloop()

    def _spin_ros(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherGUI()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()