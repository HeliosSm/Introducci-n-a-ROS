import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import re
import os

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.temperatures = []
        # El reto pide que el gráfico se genere cada 5 segundos [cite: 383]
        self.timer = self.create_timer(5.0, self.plot_data)
        self.plot_dir = '/ros2_ws/data' # Directorio compartido
        os.makedirs(self.plot_dir, exist_ok=True)
        self.get_logger().info('Nodo graficador iniciado.')

    def listener_callback(self, msg):
        match = re.search(r'(\d+)', msg.data)
        if match:
            temp = int(match.group(1))
            self.temperatures.append(temp)
            if len(self.temperatures) > 50:
                self.temperatures.pop(0)

    def plot_data(self):
        if not self.temperatures:
            self.get_logger().warn('No hay datos para graficar aún.')
            return

        plt.figure(figsize=(10, 5))
        plt.plot(self.temperatures, marker='o', linestyle='-', color='b')
        plt.title('Historial de Temperatura en Tiempo Real')
        plt.xlabel('Muestras Recientes')
        plt.ylabel('Temperatura (°C)')
        plt.grid(True)
        plt.ylim(18, 32)
        
        # Guardar la imagen en el volumen compartido [cite: 383]
        save_path = os.path.join(self.plot_dir, 'sensor_plot.png')
        plt.savefig(save_path)
        plt.close()
        self.get_logger().info(f'Gráfico actualizado y guardado en: {save_path}')

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()