from pynput.keyboard import Key, Listener

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class RobotTeleop(Node):

    def __init__(self):
	# Inicializar la superclase Nodo de la cual hereda con el nombre requerido
        super().__init__('robot_teleop')
        # Asignar a los atributos de velocidad los parámetros ingresados
        self.linear = float(input("Por favor ingrese la velocidad lineal (en cm/s - max10): "))
        self.angular = float(input("Por favor ingrese la velocidad angular (en deg/s - max45): "))
        # Publicar en el tópico robot_cmdVel el mensaje tipo Twist
        self.publisher_ = self.create_publisher(Twist,'robot_cmdVel', 10)
        # Definir el Listener de la librería pynput para que detecte tecleo
        with Listener(on_press=self.callback_pressed, on_release=self.callback_released) as listener:
            listener.join()
        # Inicializar el Listener dentro del constructor de la clase
        listener.start()

    # =============== FUNCIONES DE LA LIBRERÍA ===============
    def callback_pressed(self, key):
        # Actualización de velocidades cuando se oprime una tecla
        vel_msg = Twist()
        # Primer movimiento del robot - Traslacional hacia adelante
        if key == Key.up:
            vel_msg.linear.x = self.linear
            self.publisher_.publish(vel_msg)
            self.get_logger().info('Movimiento traslacional hacia Adelante') 
        # Segundo movimiento del robot - Traslacional hacia atrás
        elif key == Key.down:
            vel_msg.linear.x = -1*self.linear
            self.publisher_.publish(vel_msg)
            self.get_logger().info('Movimiento traslacional hacia Atrás')
        # Tercer movimiento del robot - Rotacional hacia la derecha (clockwise)
        elif key == Key.right:
            vel_msg.angular.z = -1*self.angular
            self.publisher_.publish(vel_msg)
            self.get_logger().info('Movimiento rotacional hacia la Derecha')
        # Cuarto movimiento del robot - Rotacional hacia la izquierda (counterclockwise)
        elif key == Key.left:
            vel_msg.angular.z = self.angular
            self.publisher_.publish(vel_msg)
            self.get_logger().info('Movimiento rotacional hacia la Izquierda')
        # Realizar la publicación de los datos actualizados de velocidad
        

    def callback_released(self, key):
        # Actualización a cero de las velocidades cuando se suelta una tecla
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.publisher_.publish(vel_msg)
        self.get_logger().info('No hay teclas presionadas. El robot se está deteniendo.')
        if key == Key.esc:
            # Stop listener when ESC is pressed
            return False   
        

# =============== MÉTODO MAIN PARA EJECUCIÓN ===============
def main(args=None):
    rclpy.init(args=args)

    robot_teleop = RobotTeleop()

    rclpy.spin(robot_teleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

