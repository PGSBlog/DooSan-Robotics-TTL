# kiosk.py
import rclpy
from rclpy.node import Node
from ros_study_msgs.srv import OrderReview
import tkinter as tk
from tkinter import ttk
import threading

class KioskClient(Node):
    def __init__(self):
        super().__init__('kiosk_client')
        self.cli = self.create_client(OrderReview, 'order_review')
        
        # Main window setup
        self.root = tk.Tk()
        self.root.title("Kiosk Client")
        self.root.geometry("400x600")
        self.root.resizable(False, False)
        
        # Create main container
        self.main_container = ttk.Frame(self.root)
        self.main_container.pack(fill=tk.BOTH, expand=True)
        
        # Upper frame for kiosk
        self.upper_frame = ttk.Frame(self.main_container, padding="5")
        self.upper_frame.pack(fill=tk.X)
        
        # Lower frame for selected menu
        self.lower_frame = ttk.Frame(self.main_container, padding="5")
        self.lower_frame.pack(fill=tk.BOTH, expand=True)
        
        # Title label
        title_label = ttk.Label(self.upper_frame, text="Order Kiosk", font=("Helvetica", 16, "bold"))
        title_label.pack(pady=(5, 10))
        
        # Menu items with quantity controls
        self.menu_quantities = {}  # Store quantities for each menu item
        self.create_menu_buttons()
        
        # Table number frame
        table_frame = ttk.Frame(self.upper_frame)
        table_frame.pack(pady=10)
        
        table_label = ttk.Label(table_frame, text="테이블 번호:", font=("Helvetica", 10))
        table_label.pack(side=tk.LEFT, padx=5)
        
        self.table_number_var = tk.StringVar()
        self.table_number_entry = ttk.Entry(table_frame, textvariable=self.table_number_var, width=8)
        self.table_number_entry.pack(side=tk.LEFT)
        
        # Order button
        self.order_button = ttk.Button(self.upper_frame, text="Order", command=self.send_order_request)
        self.order_button.pack(pady=5)

        # Selected menu display section
        selected_title = ttk.Label(self.lower_frame, text="Selected Menu", font=("Helvetica", 14, "bold"))
        selected_title.pack(pady=(5, 10))
        
        # Create a white background frame for selected items
        self.selected_frame = ttk.Frame(self.lower_frame, style="Selected.TFrame")
        self.selected_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Configure style for white background
        style = ttk.Style()
        style.configure("Selected.TFrame", background="white")
        
        # Selected items display
        self.selected_items_var = tk.StringVar(value="")
        self.selected_items_label = ttk.Label(
            self.selected_frame,
            textvariable=self.selected_items_var,
            justify=tk.LEFT,
            background="white"
        )
        self.selected_items_label.pack(padx=10, pady=5, anchor=tk.NW)
        
        # Status message display
        self.status_var = tk.StringVar()
        self.status_label = ttk.Label(
            self.selected_frame,
            textvariable=self.status_var,
            background="white",
            font=("Helvetica", 11)
        )
        self.status_label.pack(pady=5)

        self.waiting_for_response = False

    def create_menu_buttons(self):
        menu_frame = ttk.Frame(self.upper_frame)
        menu_frame.pack(fill=tk.X, pady=5)
        
        menu_items = ["Pizza", "Pasta", "Salad", "Soup"]
        for item in menu_items:
            item_frame = ttk.Frame(menu_frame)
            item_frame.pack(fill=tk.X, pady=2)
            
            menu_button = ttk.Button(
                item_frame,
                text=item,
                command=lambda i=item: self.add_menu_item(i),
                width=15
            )
            menu_button.pack(side=tk.LEFT, padx=2)
            
            self.menu_quantities[item] = tk.IntVar(value=0)
            quantity_label = ttk.Label(item_frame, textvariable=self.menu_quantities[item], width=3)
            quantity_label.pack(side=tk.LEFT, padx=5)
            
            minus_button = ttk.Button(
                item_frame,
                text="-",
                command=lambda i=item: self.decrease_quantity(i),
                width=3
            )
            minus_button.pack(side=tk.LEFT, padx=1)
            
            plus_button = ttk.Button(
                item_frame,
                text="+",
                command=lambda i=item: self.increase_quantity(i),
                width=3
            )
            plus_button.pack(side=tk.LEFT, padx=1)

    def add_menu_item(self, item):
        if not self.waiting_for_response:
            if self.menu_quantities[item].get() == 0:
                self.increase_quantity(item)

    def increase_quantity(self, item):
        if not self.waiting_for_response:
            current = self.menu_quantities[item].get()
            self.menu_quantities[item].set(current + 1)
            self.update_selected_display()
            self.status_var.set("")

    def decrease_quantity(self, item):
        if not self.waiting_for_response:
            current = self.menu_quantities[item].get()
            if current > 0:
                self.menu_quantities[item].set(current - 1)
                self.update_selected_display()
                self.status_var.set("")

    def update_selected_display(self):
        selected_items = []
        for item, quantity in self.menu_quantities.items():
            if quantity.get() > 0:
                selected_items.append(f"{item} X {quantity.get()}")
        
        if selected_items:
            self.selected_items_var.set("\n".join(selected_items))
        else:
            self.selected_items_var.set("")

    def send_order_request(self):
        if self.waiting_for_response:
            return

        table_number = self.table_number_var.get()
        if not table_number:
            self.status_var.set("테이블 번호를 입력해주세요")
            return

        selected_items = []
        for item, quantity in self.menu_quantities.items():
            if quantity.get() > 0:
                selected_items.append(f"{item} X {quantity.get()}")

        if not selected_items:
            self.status_var.set("메뉴를 선택해주세요")
            return

        self.waiting_for_response = True
        self.status_var.set("주문 처리 중...")
        self.order_button.configure(state='disabled')

        # 각 선택된 메뉴를 하나의 문자열로 만들어서 전송
        request = OrderReview.Request()
        request.menu_name = f"Table {table_number}/ " + ", ".join(selected_items)
        request.quantity = 1  # 모든 항목을 하나의 요청으로 보냄

        future = self.cli.call_async(request)
        future.add_done_callback(self.order_response_callback)

    def order_response_callback(self, future):
        try:
            response = future.result()
            self.root.after(0, self.handle_response, response)
        except Exception as e:
            self.root.after(0, self.handle_error, str(e))

    def handle_response(self, response):
        if response.is_approved:
            self.status_var.set("주문이 접수되었습니다")
        else:
            self.status_var.set("주문이 거절되었습니다")

        self.reset_selections()
        self.waiting_for_response = False
        self.order_button.configure(state='normal')

    def handle_error(self, error_msg):
        self.status_var.set(f"Error: {error_msg}")
        self.waiting_for_response = False
        self.order_button.configure(state='normal')

    def reset_selections(self):
        for quantity in self.menu_quantities.values():
            quantity.set(0)
        self.table_number_var.set("")
        self.selected_items_var.set("")

    def run(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')
            
        threading.Thread(target=self._spin_ros, daemon=True).start()
        self.root.mainloop()

    def _spin_ros(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = KioskClient()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
