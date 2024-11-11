#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_study_msgs.srv import OrderReview
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
from datetime import datetime
import re
from order_database import OrderDatabase
import pygame 
import os  

class KitchenServer(Node):
    def __init__(self):
        super().__init__('kitchen_server')
        self.srv = self.create_service(OrderReview, 'order_review', self.review_order_callback)
        
        # Database initialization
        self.db = OrderDatabase()
        pygame.mixer.init()
        # MP3 파일 경로 설정 - 실제 MP3 파일 경로로 수정해주세요
        self.sound_file = '/home/shin/projectsrv_ws/src/projectsrv/projectsrv/sounds/notification.wav'

        
        self.notification_sound = pygame.mixer.Sound(self.sound_file)
        # Main window setup
        self.root = tk.Tk()
        self.root.title("Kitchen Display Server")
        self.root.geometry("800x400")
        
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Create two columns
        self.order_frame = ttk.Frame(self.main_frame)
        self.order_frame.grid(row=0, column=0, padx=10)
        
        self.stats_frame = ttk.Frame(self.main_frame)
        self.stats_frame.grid(row=0, column=1, padx=10)
        
        # Order column
        title_label = ttk.Label(self.order_frame, text="Kitchen Display", font=("Helvetica", 20, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 15))
        
        self.message_display = scrolledtext.ScrolledText(
            self.order_frame, 
            wrap=tk.WORD, 
            width=50, 
            height=12, 
            font=("Helvetica", 12)
        )
        self.message_display.grid(row=1, column=0, columnspan=3, pady=(0, 15))
        
        # Stats column
        stats_label = ttk.Label(self.stats_frame, text="Order Statistics", font=("Helvetica", 16, "bold"))
        stats_label.grid(row=0, column=0, pady=(0, 15))
        
        self.stats_display = scrolledtext.ScrolledText(
            self.stats_frame,
            wrap=tk.WORD,
            width=30,
            height=12,
            font=("Helvetica", 12)
        )
        self.stats_display.grid(row=1, column=0, pady=(0, 15))
        
        # Update stats button
        self.update_stats_button = ttk.Button(
            self.stats_frame,
            text="Reset",
            command=self.reset_statistics
                    )
        self.update_stats_button.grid(row=2, column=0, pady=5)
        
        # Current order info
        self.current_order = None
        self.current_response = None
        self.response_ready = threading.Event()
        self.table_number = None
        
        # Buttons
        button_frame = ttk.Frame(self.order_frame)
        button_frame.grid(row=2, column=0, columnspan=3, pady=(0, 10))
        
        self.accept_button = ttk.Button(
            button_frame, 
            text="Accept", 
            command=lambda: self.process_order(True),
            width=15
        )
        self.accept_button.grid(row=0, column=0, padx=5)

        self.reject_button = ttk.Button(
            button_frame, 
            text="Reject", 
            command=lambda: self.process_order(False),
            width=15
        )
        self.reject_button.grid(row=0, column=1, padx=5)

        self.complete_button = ttk.Button(
            button_frame,
            text="Complete",
            command=self.complete_order,
            state="disabled",
            width=15
        )
        self.complete_button.grid(row=0, column=2, padx=5)
        
        # Initial statistics update
        self.update_statistics()
        
        self.get_logger().info('Kitchen display server is ready')

    def review_order_callback(self, request, response):
        timestamp = datetime.now().strftime("%H:%M:%S")
        order_info = f"[{timestamp}] New Order: {request.menu_name}\n"
        
        try:
            self.notification_sound.play()
        except Exception as e:
            self.get_logger().warn(f'Sound playback failed: {str(e)}')


        self.root.after(0, self.update_display, order_info)
        self.current_order = request
        
        self.table_number = self.extract_table_number(request.menu_name)
        
        self.response_ready.clear()
        
        self.root.after(0, self.update_button_states, True)
        
        self.response_ready.wait()
        
        response.is_approved = self.current_response
        response.message = "Order accepted" if self.current_response else "Order rejected"
        
        status = "accepted" if self.current_response else "rejected"
        self.root.after(0, self.update_display, f"Order has been {status}\n")
        
        return response

    def extract_table_number(self, menu_name):
        match = re.search(r"Table (\d+)", menu_name)
        if match:
            return match.group(1)
        return None

    def update_display(self, message):
        self.message_display.configure(state='normal')
        self.message_display.insert(tk.END, message)
        self.message_display.see(tk.END)
        self.message_display.configure(state='disabled')

    def update_statistics(self):
        """Update the statistics display with current totals"""
        # Get totals and reset if update button is clicked
        totals = self.db.get_menu_item_totals()
        stats_text = "Menu Item Totals:\n\n"
        
        for item, quantity in totals.items():
            stats_text += f"{item}: {quantity}\n"
        
        if not totals:
            stats_text += "No orders yet\n"
        
        self.stats_display.delete('1.0', tk.END)
        self.stats_display.insert(tk.END, stats_text)

    def reset_statistics(self):
        """Reset all statistics to zero"""
        self.db.get_menu_item_totals(reset=True)
        self.update_statistics()
        self.update_display("Statistics have been reset.\n")

    def update_button_states(self, enable_accept_reject):
        if enable_accept_reject:
            self.accept_button.configure(state='normal')
            self.reject_button.configure(state='normal')
            self.complete_button.configure(state='disabled')
        else:
            self.accept_button.configure(state='disabled')
            self.reject_button.configure(state='disabled')

    def process_order(self, approved):
        if self.current_order:
            self.current_response = approved
            
            if approved:
                self.db.add_order(self.current_order.menu_name, "accepted")
                self.update_statistics()
                self.complete_button.configure(state='normal')
            else:
                self.db.add_order(self.current_order.menu_name, "rejected")
                self.complete_button.configure(state='disabled')
            
            self.response_ready.set()
            self.current_order = None
            self.root.after(0, self.update_button_states, False)

    def complete_order(self):
        self.update_display("Order completed.\n")
        
        publisher = self.create_publisher(String, 'order_topic', 10)
        msg = String()
        
        if self.table_number:
            msg.data = f"Completed for Table {self.table_number}"
        else:
            msg.data = "Completed for unknown table"
            
        publisher.publish(msg)
        
        self.update_statistics()
        
        self.root.after(0, self.update_button_states, True)

    def run(self):
        threading.Thread(target=self._spin_ros, daemon=True).start()
        self.root.mainloop()

    def _spin_ros(self):
        rclpy.spin(self)

    def destroy_node(self):
        """Clean up resources"""
        pygame.mixer.quit() 
        self.db.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KitchenServer()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()