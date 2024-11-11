import sqlite3
from datetime import datetime

class OrderDatabase:
    def __init__(self):
        self.conn = sqlite3.connect('restaurant_orders.db')
        self.cursor = self.conn.cursor()
        self.create_tables()
    
    def create_tables(self):
        """Create necessary tables if they don't exist"""
        self.cursor.execute('''
        CREATE TABLE IF NOT EXISTS orders (
            order_id INTEGER PRIMARY KEY AUTOINCREMENT,
            table_number INTEGER,
            order_time TIMESTAMP,
            status TEXT
        )''')
        
        self.cursor.execute('''
        CREATE TABLE IF NOT EXISTS order_items (
            item_id INTEGER PRIMARY KEY AUTOINCREMENT,
            order_id INTEGER,
            menu_item TEXT,
            quantity INTEGER,
            FOREIGN KEY (order_id) REFERENCES orders (order_id)
        )''')
        
        self.conn.commit()
    
    def parse_order_string(self, order_string):
        """Parse order string to extract table number and menu items"""
        # Example input: "Table 3/ Pizza X 2, Pasta X 1"
        parts = order_string.split('/')
        table_str = parts[0].strip()
        table_number = int(table_str.split()[1])
        
        items = {}
        if len(parts) > 1:
            menu_items = parts[1].strip().split(',')
            for item in menu_items:
                item = item.strip()
                menu_name = item.split('X')[0].strip()
                quantity = int(item.split('X')[1].strip())
                items[menu_name] = quantity
                
        return table_number, items
    
    def add_order(self, order_string, status="accepted"):
        """Add a new order to the database"""
        try:
            table_number, items = self.parse_order_string(order_string)
            
            # Insert the main order
            self.cursor.execute('''
            INSERT INTO orders (table_number, order_time, status)
            VALUES (?, ?, ?)
            ''', (table_number, datetime.now(), status))
            
            order_id = self.cursor.lastrowid
            
            # Insert each menu item
            for menu_item, quantity in items.items():
                self.cursor.execute('''
                INSERT INTO order_items (order_id, menu_item, quantity)
                VALUES (?, ?, ?)
                ''', (order_id, menu_item, quantity))
            
            self.conn.commit()
            return True
        except Exception as e:
            print(f"Error adding order to database: {e}")
            self.conn.rollback()
            return False
    
    def update_order_status(self, order_id, new_status):
        """Update the status of an order"""
        try:
            self.cursor.execute('''
            UPDATE orders
            SET status = ?
            WHERE order_id = ?
            ''', (new_status, order_id))
            self.conn.commit()
            return True
        except Exception as e:
            print(f"Error updating order status: {e}")
            self.conn.rollback()
            return False
    
    def get_menu_item_totals(self, reset=False):
        """Get total quantities ordered for each menu item"""
        if reset:
            try:
                # Delete all records from order_items and orders tables
                self.cursor.execute('DELETE FROM order_items')
                self.cursor.execute('DELETE FROM orders')
                # Reset the autoincrement counters
                self.cursor.execute('DELETE FROM sqlite_sequence WHERE name="order_items" OR name="orders"')
                self.conn.commit()
                return {}
            except Exception as e:
                print(f"Error resetting statistics: {e}")
                self.conn.rollback()
                return {}
        
        self.cursor.execute('''
        SELECT menu_item, SUM(quantity) as total
        FROM order_items
        GROUP BY menu_item
        ''')
        return dict(self.cursor.fetchall())
    
    def get_recent_orders(self, limit=10):
        """Get recent orders with their items"""
        self.cursor.execute('''
        SELECT o.order_id, o.table_number, o.order_time, o.status,
               oi.menu_item, oi.quantity
        FROM orders o
        JOIN order_items oi ON o.order_id = oi.order_id
        ORDER BY o.order_time DESC
        LIMIT ?
        ''', (limit,))
        return self.cursor.fetchall()
    
    def close(self):
        """Close the database connection"""
        self.conn.close()