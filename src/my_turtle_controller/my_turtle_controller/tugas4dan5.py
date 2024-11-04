import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill
import time
import math
import threading

class DualTurtleController(Node):
    def __init__(self):
        super().__init__('dual_turtle_controller')

        # Membuat publisher untuk menggerakkan turtle1 dan turtle2
        self.turtle1_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.turtle2_publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Membuat client untuk layanan spawn (menambahkan turtle kedua) dan kill (menghapus turtle)
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')

        # Tunggu sampai layanan siap
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')

        # Spawning turtle2 pada posisi tertentu
        if not self.spawn_turtle2(5.0, 5.0):
            self.get_logger().error('Turtle2 did not spawn properly. Exiting...')
            return

        # Memberi waktu tambahan untuk memastikan turtle2 siap
        time.sleep(3)

        # Membuat thread untuk menggerakkan turtle1 dan turtle2 secara bersamaan
        thread1 = threading.Thread(target=self.draw_flower)
        thread2 = threading.Thread(target=self.write_name, args=("nugi",))

        # Menjalankan kedua thread secara bersamaan
        thread1.start()
        thread2.start()

        # Menunggu kedua thread selesai
        thread1.join()
        thread2.join()

    def spawn_turtle2(self, x, y):
        # Request untuk menambahkan turtle kedua di posisi tertentu
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = 0.0
        request.name = 'turtle2'

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Turtle2 spawned successfully at ({x}, {y})')
            return True
        else:
            self.get_logger().error('Failed to spawn turtle2')
            return False

    def kill_turtle2(self):
        # Request untuk menghapus turtle2
        request = Kill.Request()
        request.name = 'turtle2'

        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Turtle2 killed successfully')
            return True
        else:
            self.get_logger().error('Failed to kill turtle2')
            return False

    def draw_flower(self):
        twist = Twist()
        kelopak = 10  # Jumlah kelopak bunga
        sudut_antar_kelopak = 360 / kelopak

        for _ in range(kelopak):
            # Membuat satu kelopak bunga
            for _ in range(2):
                twist.linear.x = 2.0  # Gerak maju untuk membentuk kelopak
                twist.angular.z = math.radians(45)  # Belok dengan sudut melingkar untuk kelopak
                self.turtle1_publisher.publish(twist)
                self.get_logger().info('Turtle1 drawing flower petal - part 1')
                time.sleep(2.0)

                twist.linear.x = 0.0
                twist.angular.z = math.radians(135)  # Belok lebih tajam untuk sisi kelopak
                self.turtle1_publisher.publish(twist)
                self.get_logger().info('Turtle1 drawing flower petal - part 2')
                time.sleep(1.0)

            # Berbelok ke posisi awal untuk mulai menggambar kelopak berikutnya
            twist.linear.x = 0.0
            twist.angular.z = math.radians(sudut_antar_kelopak)
            self.turtle1_publisher.publish(twist)
            time.sleep(1.0)

        # Hentikan gerakan setelah selesai menggambar
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.turtle1_publisher.publish(twist)

    def write_name(self, name):
        # Menulis nama "nugi" di bawah bunga
        self.get_logger().info('Starting to write name...')
        start_x = 2.0  # Koordinat x awal
        start_y = 2.0  # Koordinat y awal
        letter_spacing = 2.5  # Jarak antar huruf

        for letter in name:
            # Spawn turtle2 di posisi awal untuk menulis huruf baru
            if not self.spawn_turtle2(start_x, start_y):
                self.get_logger().error('Failed to spawn turtle2 for writing letter.')
                return

            self.get_logger().info(f'Writing letter: {letter}')
            if letter == 'n':
                self.draw_n()
            elif letter == 'u':
                self.draw_u()
            elif letter == 'g':
                self.draw_g()
            elif letter == 'i':
                self.draw_i()

            # Menghapus turtle2 setelah selesai menulis satu huruf
            if not self.kill_turtle2():
                self.get_logger().error('Failed to kill turtle2 after writing letter.')
                return

            # Menggeser posisi x untuk huruf berikutnya
            start_x += letter_spacing
            time.sleep(1.0)

    def draw_n(self):
        twist = Twist()

        # Gerak lurus ke atas untuk batang pertama
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 1.5)

        # Miring ke kanan
        twist.linear.x = 1.0
        twist.angular.z = math.radians(45)
        self.publish_turtle2_command(twist, 1.0)

        # Gerak lurus ke bawah
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 1.5)

        # Hentikan gerakan
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 0.5)

    def draw_u(self):
        twist = Twist()

        # Turun untuk batang pertama
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 1.5)

        # Lengkung ke kanan
        twist.linear.x = 1.0
        twist.angular.z = math.radians(90)
        self.publish_turtle2_command(twist, 1.0)

        # Naik untuk batang terakhir
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 1.5)

        # Hentikan gerakan
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 0.5)

    def draw_g(self):
        twist = Twist()

        # Membuat lingkaran bagian atas
        twist.linear.x = 1.0
        twist.angular.z = math.radians(180)
        self.publish_turtle2_command(twist, 1.5)

        # Membuat kaki ke bawah
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 1.0)

        # Hentikan gerakan
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 0.5)

    def draw_i(self):
        twist = Twist()

        # Membuat garis lurus ke atas
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 1.0)

        # Hentikan gerakan
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publish_turtle2_command(twist, 0.5)

    def publish_turtle2_command(self, twist, duration):
        """Publish command to turtle2 with specified duration."""
        self.turtle2_publisher.publish(twist)
        time.sleep(duration)

def main(args=None):
    rclpy.init(args=args)
    node = DualTurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
