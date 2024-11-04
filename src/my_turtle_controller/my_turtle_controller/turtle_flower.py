import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class TurtleFlowerNode(Node):
    def __init__(self):
        super().__init__('turtle_flower')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        time.sleep(2)  # Tunggu sebentar agar Turtlesim Node siap
        self.draw_flower()

    def draw_flower(self):
        twist = Twist()
        kelopak = 10  # Tentukan berapa banyak kelopak bunga
        sudut_antar_kelopak = 360 / kelopak

        for _ in range(kelopak):
            # Membuat satu kelopak bunga
            for _ in range(2):
                twist.linear.x = 2.0  # Gerak maju untuk membentuk kelopak
                twist.angular.z = math.radians(45)  # Belok dengan sudut melingkar untuk kelopak
                self.publisher_.publish(twist)
                time.sleep(2.0)

                twist.linear.x = 0.0
                twist.angular.z = math.radians(135)  # Belok lebih tajam untuk sisi kelopak
                self.publisher_.publish(twist)
                time.sleep(1.0)

            # Berbelok ke posisi awal untuk mulai menggambar kelopak berikutnya
            twist.linear.x = 0.0
            twist.angular.z = math.radians(sudut_antar_kelopak)
            self.publisher_.publish(twist)
            time.sleep(1.0)

        # Hentikan gerakan setelah selesai menggambar
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_flower_node = TurtleFlowerNode()
    rclpy.spin_once(turtle_flower_node, timeout_sec=2)
    turtle_flower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
