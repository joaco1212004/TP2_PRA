#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.map = None
        self.odom = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/calc_odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/plan', 10)

        self.get_logger().info('Planner iniciado.')

    def map_callback(self, msg):
        self.map = msg

    def odom_callback(self, msg):
        self.odom = msg

    def goal_callback(self, msg):
        if self.map is None:
            self.get_logger().warn('Todavía no se recibió /map.')
            return
        if self.odom is None:
            self.get_logger().warn('Todavía no se recibió /calc_odom.')
            return

        self.plan(msg)

    def world_to_map(self, x, y):
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        res = self.map.info.resolution

        mx = int((x - ox) / res)
        my = int((y - oy) / res)

        if not (0 <= mx < self.map.info.width and
                0 <= my < self.map.info.height):
            return None

        return (mx, my)

    def map_to_world(self, mx, my):
        ox = self.map.info.origin.position.x
        oy = self.map.info.origin.position.y
        res = self.map.info.resolution

        return (
            ox + (mx + 0.5) * res,
            oy + (my + 0.5) * res
        )

    def plan(self, goal_msg):
        start = self.world_to_map(
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y
        )

        goal = self.world_to_map(
            goal_msg.pose.position.x,
            goal_msg.pose.position.y
        )

        if start is None:
            self.get_logger().warn('La posición actual está fuera del mapa.')
            return

        if goal is None:
            self.get_logger().warn('El objetivo está fuera del mapa.')
            return

        self.get_logger().info(f'Planificando de {start} a {goal}.')

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # [COMPLETAR] Implementar algún algoritmo de planificación de caminos
        # Debe devolver una lista de tuplas (x, y) en coordenadas de mapa, o None si no se encuentra un camino
        path = [(0,0), (1,0), (1,1), (2,1), (2,2)]  # Ejemplo de camino ficticio

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        if path is None:
            self.get_logger().warn('No se encontró un camino.')
            return

        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map.header.frame_id

        for mx, my in path:
            x, y = self.map_to_world(mx, my)

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0

            msg.poses.append(pose)

        self.path_pub.publish(msg)

        self.get_logger().info(
            f'Camino publicado: {len(path)} puntos.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()