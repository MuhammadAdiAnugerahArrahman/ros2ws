import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import math
import time
import threading

class DualTurtleController(Node):
    def __init__(self):
        super().__init__('dual_turtle_controller')
        self.turtle1_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle2_publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.turtle3_publisher = self.create_publisher(Twist, '/turtle3/cmd_vel', 10)
        self.turtle4_publisher = self.create_publisher(Twist, '/turtle4/cmd_vel', 10)
        self.turtle5_publisher = self.create_publisher(Twist, '/turtle5/cmd_vel', 10)
        self.spawn_client = self.create_client(Spawn, '/spawn')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            pass

        # Spawning turtles in specific positions
        if not self.spawn_turtle2(7.5, 8.0):
            return
        if not self.spawn_turtle3(1.0, 1.0):
            return
        if not self.spawn_turtle4(8.0, 1.0):
            return
        if not self.spawn_turtle5(1.0, 8.0):
            return

        time.sleep(2)

        # Create and start threads for each turtle's task
        thread1 = threading.Thread(target=self.draw_flower)
        thread2 = threading.Thread(target=self.draw_star)
        thread3 = threading.Thread(target=self.draw_square)
        thread4 = threading.Thread(target=self.draw_circle)
        thread5 = threading.Thread(target=self.draw_triangle)

        thread1.start()
        thread2.start()
        thread3.start()
        thread4.start()
        thread5.start()

        # Wait for all threads to complete
        thread1.join()
        thread2.join()
        thread3.join()
        thread4.join()
        thread5.join()

    def spawn_turtle2(self, x, y):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'turtle2'
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def spawn_turtle3(self, x, y):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'turtle3'
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def spawn_turtle4(self, x, y):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'turtle4'
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def spawn_turtle5(self, x, y):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'turtle5'
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() is not None

    def draw_flower(self):
        twist = Twist()
        kelopak = 5
        sudut_antar_kelopak = 360 / kelopak

        for _ in range(kelopak):
            for _ in range(2):
                twist.linear.x = 2.0
                twist.angular.z = math.radians(45)
                self.turtle1_publisher.publish(twist)
                time.sleep(2.0)

                twist.linear.x = 0.0
                twist.angular.z = math.radians(135)
                self.turtle1_publisher.publish(twist)
                time.sleep(1.0)

            twist.linear.x = 0.0
            twist.angular.z = math.radians(sudut_antar_kelopak)
            self.turtle1_publisher.publish(twist)
            time.sleep(1.0)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.turtle1_publisher.publish(twist)

    def draw_star(self):
        twist = Twist()
        side_length = 2.0
        angle = math.radians(108)

        for _ in range(10):
            twist.linear.x = side_length
            twist.angular.z = 0.0
            self.turtle2_publisher.publish(twist)
            time.sleep(1.0)

            twist.linear.x = 0.0
            twist.angular.z = angle
            self.turtle2_publisher.publish(twist)
            time.sleep(1.0)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.turtle2_publisher.publish(twist)

    def draw_square(self):
        twist = Twist()
        side_length = 2.0

        for _ in range(4):
            twist.linear.x = side_length
            twist.angular.z = 0.0
            self.turtle3_publisher.publish(twist)
            time.sleep(5.0)

            twist.linear.x = 0.0
            twist.angular.z = math.radians(90)
            self.turtle3_publisher.publish(twist)
            time.sleep(1.0)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.turtle3_publisher.publish(twist)

    def draw_circle(self):
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = math.radians(60)

        for _ in range(6):
            self.turtle4_publisher.publish(twist)
            time.sleep(5.0)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.turtle4_publisher.publish(twist)

    def draw_triangle(self):
        twist = Twist()
        side_length = 2.0
        angle = math.radians(120)  # Sudut untuk segitiga sama sisi

        for _ in range(3):
            twist.linear.x = side_length
            twist.angular.z = 0.0
            self.turtle5_publisher.publish(twist)
            time.sleep(5.0)

            twist.linear.x = 0.0
            twist.angular.z = angle
            self.turtle5_publisher.publish(twist)
            time.sleep(1.0)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.turtle5_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = DualTurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
