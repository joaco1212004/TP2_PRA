import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker

def get_yaw_from_quaternion(q):
    """Extrae el ángulo yaw (rotación Z) a partir del cuaternión."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Parámetros del algoritmo
        self.lookahead_distance = 0.0  # [IMPLEMENTAR] Distancia de avance (m)
        self.target_speed = 0.0        # [IMPLEMENTAR] Velocidad lineal deseada (m/s)
        self.goal_tolerance = 0.0      # [IMPLEMENTAR] Radio de parada respecto a la meta (m)

        # Variables de estado
        self.current_path = None
        self.current_pose = None
        self.path_frame_id = 'map'
        self.last_closest_idx = 0      # Índice para evitar retrocesos en la ruta

        # Suscriptores y Publicadores
        self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.create_subscription(Odometry, '/calc_odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)

        # Bucle de control a 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def path_callback(self, msg):
        self.current_path = msg.poses
        self.last_closest_idx = 0  # Reiniciar índice al recibir una nueva ruta
        if msg.header.frame_id:
            self.path_frame_id = msg.header.frame_id

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if not self.current_path or self.current_pose is None:
            return

        # [IMPLEMENTAR] Algoritmo de path following Pure Pursuit

        #### Ejemplo de publicación de lookahead marker
        # target_point = self.get_lookahead_point(x, y)
        # self.publish_lookahead_marker(target_point)


        #### Ejemplo de publicación de comando de velocidad
        # cmd = Twist()
        # cmd.linear.x = self.target_speed
        # cmd.angular.z = angular_velocity
        # self.cmd_pub.publish(cmd)

    def publish_lookahead_marker(self, target_point):
        marker = Marker()
        marker.header.frame_id = self.path_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = target_point.x
        marker.pose.position.y = target_point.y
        marker.pose.position.z = target_point.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.nanosec = 200000000

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()