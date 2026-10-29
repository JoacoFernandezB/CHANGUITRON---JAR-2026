"""Le da a AMCL la pose inicial del robot: una gaussiana centrada en el origen del mapa.

El robot arranca cerca del origen (0, 0, 0) del mapa publicado, no exactamente ahí (§1). AMCL
reparte sus partículas según la pose inicial y su covarianza; con covarianza cero (lo que hace el
parámetro `set_initial_pose` de AMCL) todas arrancarían en el mismo punto. Por eso la mandamos
nosotros por /initialpose, con una incertidumbre que sale de params/params.yaml.

Se manda una vez por período, después de recibir /map, hasta que AMCL publica la transformación
map -> odom (señal de que ya se localizó). /initialpose es un topic interno: solo lo usan nuestros
nodos.
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from equipo_jar.localizacion import covarianza_pose_inicial, cuaternion_de_yaw

# /map lo publica el juez con esta QoS (docs/QOS.md).
QOS_MAPA = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NodoPoseInicial(Node):
    def __init__(self):
        super().__init__('nodo_pose_inicial')
        self.declare_parameter('sigma_xy_inicial_m', 0.15)
        self.declare_parameter('sigma_yaw_inicial_deg', 10.0)
        self.declare_parameter('periodo_pose_inicial_s', 1.0)
        self.declare_parameter('frame_mapa', 'map')
        self.declare_parameter('frame_odom', 'odom')

        self.mapa_recibido = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub_mapa = self.create_subscription(
            OccupancyGrid, '/map', self.al_recibir_mapa, QOS_MAPA)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(
            self.get_parameter('periodo_pose_inicial_s').value, self.ciclo)

    def al_recibir_mapa(self, _mapa):
        self.mapa_recibido = True

    def ciclo(self):
        if not self.mapa_recibido:
            return  # sin mapa AMCL no puede localizarse
        mapa = self.get_parameter('frame_mapa').value
        odom = self.get_parameter('frame_odom').value
        if self.tf_buffer.can_transform(mapa, odom, Time()):
            self.get_logger().info(f'AMCL publica {mapa} -> {odom}: localizado')
            self.timer.cancel()
            return
        if self.pub_pose.get_subscription_count() == 0:
            return  # AMCL todavía no arrancó: un mensaje ahora se perdería
        self.pub_pose.publish(self.pose_inicial(mapa))
        self.get_logger().info('Pose inicial enviada a AMCL')

    def pose_inicial(self, frame_mapa):
        """El origen del mapa, con la incertidumbre de params/params.yaml."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_mapa
        msg.header.stamp = self.get_clock().now().to_msg()
        q = msg.pose.pose.orientation
        q.x, q.y, q.z, q.w = cuaternion_de_yaw(0.0)
        msg.pose.covariance = covarianza_pose_inicial(
            self.get_parameter('sigma_xy_inicial_m').value,
            math.radians(self.get_parameter('sigma_yaw_inicial_deg').value))
        return msg


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoPoseInicial()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
