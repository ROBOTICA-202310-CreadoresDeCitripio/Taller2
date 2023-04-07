import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial,time


class RobotController(Node):
    # El constructor de la clase se suscribe al tópico robot_cmdVel en la VM
    def __init__(self):
        super().__init__('robot_controller')
        # Open serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        # Wait for Arduino to reset
        time.sleep(2)

        self.subscription = self.create_subscription(
            Twist,
            'robot_cmdVel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Extract linear and angular velocities from Twist message
        linear = msg.linear.x
        angular = msg.angular.z
        # Pack linear and angular velocities into byte array and send over serial
        data = "{},{}\n".format(linear, angular)
        self.ser.write(data.encode())


def main(args=None):
    # Creación del nodo de ROS2 que controla al robot
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    