import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import filedialog
import math
from rclpy.clock import ROSClock
import datetime

class InitialMenu():
    # ==================================================================================================
    # Constructor del menú inicial de la aplicación
    # ==================================================================================================
    def __init__(self, message_1, message_2, option_1, option_2, option_3):
        self.window = tk.Tk()
        self.window.title("Turtlebot Application Start Menu")
        self.window.resizable(True, False)

        # Crear los mensajes que se mostrarán en los botones
        self.label_1 = tk.Label(self.window, text=message_1)
        self.label_1.pack(pady=5)
        self.label_2 = tk.Label(self.window, text=message_2)
        self.label_2.pack(pady=5)

        # Crear los botones que se incorporarán en la ventana
        self.button_1 = tk.Button(
            self.window, text=option_1, command=self.select_draw_trajectory)
        self.button_1.pack(side=tk.LEFT, padx=10)
        self.button_2 = tk.Button(
            self.window, text=option_2, command=self.select_save_trajectory)
        self.button_2.pack(side=tk.LEFT, padx=10)
        self.button_3 = tk.Button(
            self.window, text=option_3, command=self.select_play_trajectory)
        self.button_3.pack(side=tk.LEFT, padx=10)

    # ==================================================================================================
    # Definir los métodos de la clase -> Event Listeners de hacer click a algún botón
    # ==================================================================================================
    def create_file(self):
        # Get the file name and location from the user
        file_path = filedialog.asksaveasfilename(defaultextension='.txt')

        # Check if the user has chosen a file name and location
        if file_path:
            # Create the file
            with open(file_path, 'w') as f:
                # Write a header to the file
                f.write('linear_x, angular_z, timestamp\n')
        return file_path

    def read_file(self):
        file_path = filedialog.askopenfilename()

        # Read the TXT file starting from the second line
        with open(file_path, 'r') as f:
            next(f)
            for line in f:
                print(line.strip())


    def select_draw_trajectory(self):
        # Primera opción: Dibujar la trayectoria seguida por el robot
        self.window.destroy()
        # Inicializar la GUI específica para Dibujo de Trayectoria
        rclpy.init(args=None)
        turtle_bot_interface = TurtleBotInterface()
        rclpy.spin(turtle_bot_interface)

        # Arrancar la GUI de Tkinter
        turtle_bot_interface.root.mainloop()

        turtle_bot_interface.destroy_node()
        rclpy.shutdown()

    def select_save_trajectory(self):
        # Segunda opción: Guardar la trayectoria seguida por el robot
        self.window.destroy()
        file = self.create_file()
        # Inicializar la GUI específica para Guardado de Trayectoria
        rclpy.init(args=None)
        turtle_bot_interface = TurtleBotInterface()
        turtle_bot_interface.file_path = file
        rclpy.spin(turtle_bot_interface)

        # Arrancar la GUI de Tkinter
        turtle_bot_interface.root.mainloop()

        turtle_bot_interface.destroy_node()
        rclpy.shutdown()

    def select_play_trajectory(self):
        # Tercera opción: Reporoducir la trayectoria seguida por el robot
        self.window.destroy()
        file = self.read_file()
        # Inicializar la GUI específica para Reproducción de Trayectoria
        rclpy.init(args=None)
        turtle_bot_interface = TurtleBotInterface()
        rclpy.spin(turtle_bot_interface)

        # Arrancar la GUI de Tkinter
        turtle_bot_interface.root.mainloop()

        turtle_bot_interface.destroy_node()
        rclpy.shutdown()

    def run(self):
        # Correr la ventana con el menú inicial
        self.window.mainloop()


class TurtleBotInterface(Node):
    def __init__(self):
        super().__init__("robot_interface")
        self.file_path = None
        self.subscription = self.create_subscription(Twist, "robot_cmdVel", self.callback, 10)
        self.subscription

        # Crear ventana principal de Tkinter
        self.root = tk.Tk()
        self.root.title("Robot Interface")

        # Crear figura de Matplotlib
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.xdata = [0.0]
        self.ydata = [0.0]
        self.theta = 0.0
        self.line, = self.ax.plot(self.xdata, self.ydata)

        # Crea un botón que llame a la función save_figure cuando se haga clic en él
        save_button = tk.Button(self.root, text="Guardar", command=self.save_figure)
        save_button.pack(side=tk.BOTTOM)

        # Crea un cuadro de texto que actualiza el titulo de la gráfica
        self.text_field = tk.Text(self.root, height=1, width=40, wrap="word")
        self.text_field.pack()

        # Crea un botón que actualiza el titulo de la gráfica
        title_button = tk.Button(self.root, text="Cambiar título", command=self.cambiar_titulo)
        title_button.pack()

        # Crear un widget de Matplotlib que se puede agregar a la ventana principal de Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

    def actualizar_titulo(self, new_title):
        self.fig_title = new_title
        self.ax.set_title(new_title)

    def cambiar_titulo(self):
        new_title = self.text_field.get('1.0','end')
        self.actualizar_titulo(new_title)

    def save_figure(self):
        # Pídele al usuario que seleccione un archivo
        file_path = tk.filedialog.asksaveasfilename(defaultextension=".png")
        # Guarda la figura de Matplotlib en el archivo seleccionado
        if file_path:
            self.fig.savefig(file_path)

    def callback(self, msg):
        x = 0.01*msg.linear.x
        self.theta += math.pi*msg.angular.z/180
        self.xdata.append(self.xdata[-1] + x * math.cos(self.theta))
        self.ydata.append(self.ydata[-1] + x * math.sin(self.theta))
        self.line.set_data(self.xdata, self.ydata)
        plt.draw()
        plt.pause(0.01)
        plt.show(block=False)
        if self.file_path is not None:
            current_time = ROSClock().now().to_msg()

            # Create a string with the data to be saved
            data = f'{msg.linear.x}, {msg.angular.z}, {current_time.sec}.{current_time.nanosec}\n'

            # Open the file in append mode and write the data
            with open(self.file_path, 'a') as f:
                f.write(data)


def main(args=None):
    # Diálogo para funcionamiento inicial
    dialog = InitialMenu("¡Bienvenid@ a la aplicación del TurtleBot 15!", "Seleccione una opción:",
                         "DIBUJAR TRAYECTORIA", "GUARDAR TRAYECTORIA", "REPRODUCIR TRAYECTORIA")
    dialog.run()

if __name__ == '__main__':
    main()