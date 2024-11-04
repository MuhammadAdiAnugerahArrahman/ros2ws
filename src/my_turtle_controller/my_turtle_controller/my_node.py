import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_cmd)
        self.get_logger().info('Turtle Controller Node has been started.')

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: Linear X: %f Angular Z: %f' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
