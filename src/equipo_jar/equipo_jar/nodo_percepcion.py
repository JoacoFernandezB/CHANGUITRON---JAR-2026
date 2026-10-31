"""Percepción: encuentra las cajas rojas y publica dónde están en el mapa.

Mira la cámara y el lidar, ubica cada caja roja en el marco `map` y, cuando la vio varias veces
seguidas en el mismo lugar, la publica UNA vez en el topic interno /equipo/cajas_confirmadas.
No reporta: el único que publica en /victimas es nodo_equipo (más adelante, el nodo de misión),
que lleva la cuenta de los N reportes (§7.2).

Cómo se ubica una caja (las cuentas están en deteccion.py, sin ROS y con tests):
1. La cámara ve una mancha roja: su columna en la imagen da un ángulo respecto del frente.
2. El lidar, en esa misma dirección, da la distancia a la cara de la caja. Se contrasta con la
   distancia que sale del alto de la caja en la imagen.
3. Se suma medio lado de caja (el juez mide contra el centro, §6.1) y, con la pose del robot en el
   mapa (TF map -> base_footprint, que completa AMCL), se pasa a coordenadas del plano.
4. Se confirma con varias vistas seguidas que coincidan y se publica el promedio. Una caja ya
   confirmada no se vuelve a publicar.

Por ahora solo mira mientras el robot avanza: girando en el lugar, la imagen y la pose dejan de
ser del mismo instante. Es lo mismo que hacía nodo_equipo antes de separar este nodo.
"""
import math

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from equipo_jar.deteccion import (
    angulo_de_columna, buscar_caja_roja, cerca_de_alguno, coinciden, distancia_a_la_caja,
    distancia_por_altura, promedio, punto_en_el_plano, yaw_de_cuaternion)

# /map lo publica el juez con esta QoS (docs/QOS.md).
QOS_MAPA = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Topic interno (no es del reglamento): una caja confirmada por mensaje, en el marco `map`.
TOPIC_CAJAS = '/equipo/cajas_confirmadas'


class NodoPercepcion(Node):
    def __init__(self):
        super().__init__('nodo_percepcion')
        # Los números viven en params/params.yaml (§9.1), sección nodo_percepcion.
        self.declare_parameter('frecuencia_hz', 10.0)
        self.declare_parameter('angulo_frente_deg', 180.0)
        self.declare_parameter('velocidad_lineal_min_deteccion', 0.05)
        self.declare_parameter('saturacion_min', 120)
        self.declare_parameter('valor_min', 80)
        self.declare_parameter('area_minima_px', 400.0)
        self.declare_parameter('margen_borde_px', 5)
        self.declare_parameter('fraccion_central', 0.6)
        self.declare_parameter('distancia_min_deteccion_m', 0.35)
        self.declare_parameter('distancia_max_deteccion_m', 3.0)
        self.declare_parameter('semilado_caja_m', 0.125)
        self.declare_parameter('alto_caja_m', 0.30)
        self.declare_parameter('tolerancia_distancia', 0.25)
        self.declare_parameter('confirmaciones', 5)
        self.declare_parameter('dispersion_max_m', 0.25)
        self.declare_parameter('radio_duplicado_m', 0.60)
        self.declare_parameter('frame_mapa', 'map')
        self.declare_parameter('frame_robot', 'base_footprint')

        self.puente = CvBridge()
        self.mapa_recibido = False
        self.scan = None
        self.imagen = None
        self.info_camara = None
        self.velocidad_lineal = 0.0  # m/s, de /odom: para saber si el robot avanza
        self.observaciones = []      # posiciones seguidas de la caja que se está viendo
        self.confirmadas = []        # cajas ya publicadas: no se vuelven a publicar

        self.sub_mapa = self.create_subscription(
            OccupancyGrid, '/map', self.al_recibir_mapa, QOS_MAPA)
        # Sensores (lidar y cámara): best effort. Con la QoS por defecto no llegaría nada.
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.al_recibir_scan, qos_profile_sensor_data)
        self.sub_imagen = self.create_subscription(
            Image, '/cam_1/color/image_raw', self.al_recibir_imagen, qos_profile_sensor_data)
        self.sub_info = self.create_subscription(
            CameraInfo, '/cam_1/color/camera_info', self.al_recibir_info,
            qos_profile_sensor_data)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.al_recibir_odom, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cajas = self.create_publisher(PointStamped, TOPIC_CAJAS, 10)

        self.create_timer(1.0 / self.get_parameter('frecuencia_hz').value, self.ciclo)
        self.get_logger().info('Percepción lista. Esperando el mapa en /map...')

    # --- Sensores: los callbacks solo guardan el último dato; decide el timer ---------------

    def al_recibir_mapa(self, _mapa):
        self.mapa_recibido = True

    def al_recibir_scan(self, scan):
        self.scan = scan

    def al_recibir_imagen(self, msg):
        self.imagen = self.puente.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def al_recibir_info(self, info):
        self.info_camara = info

    def al_recibir_odom(self, odom):
        self.velocidad_lineal = odom.twist.twist.linear.x

    def pose_en_el_mapa(self):
        """(x, y, yaw) del robot en el mapa según AMCL, o None si todavía no se localizó."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.get_parameter('frame_mapa').value,
                self.get_parameter('frame_robot').value, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_de_cuaternion(q.x, q.y, q.z, q.w)

    # --- Ciclo: se ejecuta a frecuencia_hz --------------------------------------------------

    def ciclo(self):
        if not self.mapa_recibido:
            return  # nada antes del mapa (§5)
        avanzando = abs(self.velocidad_lineal) >= self.get_parameter(
            'velocidad_lineal_min_deteccion').value
        pose = self.pose_en_el_mapa()
        if not avanzando or pose is None:
            self.observaciones.clear()  # girando en el lugar, la imagen y la pose no coinciden
            return

        punto = self.ubicar_caja_roja(pose)
        if punto is None or cerca_de_alguno(
                punto, self.confirmadas, self.get_parameter('radio_duplicado_m').value):
            self.observaciones.clear()
            return

        # Una sola vista puede estar equivocada (ruido, una pared detrás de la caja): se
        # confirma con varias seguidas que coincidan.
        if self.observaciones and not cerca_de_alguno(
                punto, self.observaciones, self.get_parameter('dispersion_max_m').value):
            self.observaciones.clear()
        self.observaciones.append(punto)
        if len(self.observaciones) < self.get_parameter('confirmaciones').value:
            return

        x, y = promedio(self.observaciones)
        self.observaciones.clear()
        self.confirmadas.append((x, y))
        self.publicar_caja(x, y)
        self.get_logger().info(
            f'Caja {len(self.confirmadas)} confirmada en ({x:.2f}, {y:.2f})')

    def ubicar_caja_roja(self, pose):
        """Posición (x, y) de la caja roja que se ve ahora, o None si no hay una confiable."""
        if self.imagen is None or self.info_camara is None or self.scan is None:
            return None
        caja = buscar_caja_roja(
            self.imagen, self.get_parameter('saturacion_min').value,
            self.get_parameter('valor_min').value, self.get_parameter('area_minima_px').value,
            self.get_parameter('margen_borde_px').value)
        if caja is None:
            return None
        columna, ancho_px, alto_px = caja

        # 1. Ángulo de la caja respecto del frente (la cámara mira al frente del robot).
        fx, cx = self.info_camara.k[0], self.info_camara.k[2]
        angulo = angulo_de_columna(columna, fx, cx)

        # 2. Distancia que ve el lidar en esa dirección: la mediana de los rayos del tramo
        #    central de la caja (el tamaño del tramo sale de cuántos píxeles ocupa en la imagen).
        ancho_deg = max(3.0, math.degrees(ancho_px / fx)
                        * self.get_parameter('fraccion_central').value)
        distancia = distancia_a_la_caja(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
            self.get_parameter('angulo_frente_deg').value, math.degrees(angulo), ancho_deg,
            self.scan.range_min, self.scan.range_max)
        if distancia is None or not (
                self.get_parameter('distancia_min_deteccion_m').value <= distancia
                <= self.get_parameter('distancia_max_deteccion_m').value):
            return None

        # 3. Contraste: el tamaño de la caja en la imagen da otra estimación de la distancia. Si el
        #    lidar mide otra cosa (por ejemplo, el borde de una pared que tapa la caja), se descarta.
        por_altura = distancia_por_altura(
            alto_px, self.info_camara.k[4], self.get_parameter('alto_caja_m').value)
        if not coinciden(distancia, por_altura, self.get_parameter('tolerancia_distancia').value):
            return None

        # 4. El lidar ve la cara de la caja; el punto que puntúa es su centro (§6.1).
        distancia += self.get_parameter('semilado_caja_m').value
        x, y, yaw = pose
        return punto_en_el_plano(x, y, yaw, angulo, distancia)

    def publicar_caja(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = self.get_parameter('frame_mapa').value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(x)
        msg.point.y = float(y)
        self.pub_cajas.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoPercepcion()
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
