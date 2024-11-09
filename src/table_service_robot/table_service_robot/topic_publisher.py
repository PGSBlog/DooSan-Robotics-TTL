#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class ServiceTopicPublisher(Node):
    def __init__(self):
        super().__init__('service_topic_publisher')
        
        # 퍼블리셔 생성
        self.table_num_pub = self.create_publisher(Int32, 'table_number', 10)
        self.cooking_pub = self.create_publisher(String, 'cooking_status', 10)
        self.delivery_pub = self.create_publisher(String, 'delivery_status', 10)
        
        # 타이머 생성 (사용자 입력 처리용)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Topic Publisher Node has been started')

    def timer_callback(self):
        try:
            # 사용자 입력 받기
            print("\nSelect command:")
            print("1. Send table number")
            print("2. Send cooking complete")
            print("3. Send delivery complete")
            print("4. Exit")
            
            cmd = input("Enter command number: ")
            
            if cmd == '1':
                table_num = int(input("Enter table number (1-9): "))
                if 1 <= table_num <= 9:
                    msg = Int32()
                    msg.data = table_num
                    self.table_num_pub.publish(msg)
                    self.get_logger().info(f'Published table number: {table_num}')
                else:
                    self.get_logger().warning('Invalid table number')
                    
            elif cmd == '2':
                msg = String()
                msg.data = "조리 완료"
                self.cooking_pub.publish(msg)
                self.get_logger().info('Published cooking complete')
                
            elif cmd == '3':
                msg = String()
                msg.data = "배달 완료"
                self.delivery_pub.publish(msg)
                self.get_logger().info('Published delivery complete')
                
            elif cmd == '4':
                raise KeyboardInterrupt
                
        except KeyboardInterrupt:
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    publisher = ServiceTopicPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
