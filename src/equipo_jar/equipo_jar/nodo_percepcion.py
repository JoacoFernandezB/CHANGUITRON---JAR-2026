"""Percepción: encuentra las cajas rojas y publica dónde están en el mapa.

Mira la cámara y el lidar, ubica cada caja roja en el marco `map` y, cuando la vio varias veces en
el mismo lugar, la publica UNA vez en el topic interno /equipo/cajas_confirmadas. No reporta: el
único que publica en /victimas es nodo_equipo (más adelante, el nodo de misión), que lleva la
cuenta de los N reportes (§7.2).

Cómo se ubica una caja (las cuentas están en deteccion.py, sin ROS y con tests):
1. La cámara ve una mancha roja con forma de caja (descarta otros colores, hilos y manchas
   cortadas o tapadas). Su columna en la imagen da un ángulo respecto del frente.
2. El lidar, en esa misma dirección, da la distancia a la cara de la caja. Se descarta si los rayos
   no son parejos (un borde) o si justo al costado hay algo más cerca (una pared que la tapa).
3. Se contrasta con la cámara: una caja apoyada en el piso a esa profundidad tiene que verse de
   cierto alto y en ciertas filas de la imagen.
4. Se suma medio lado de caja (el juez mide contra el centro, §6.1) y, con la pose del robot, se
   pasa al mapa. El /map descarta lo imposible: adentro de una pared, o detrás de una.
5. Las vistas se juntan por cercanía (agrupador.py) y se publica la mediana del grupo.

La pose sale de supervisor_localizacion.py: AMCL mientras coincida con la odometría, y la última
corrección buena de AMCL + /odom (respaldo) cuando AMCL salta, deriva o deja de publicar. La pose
se toma en el instante de la imagen (TF odom -> base_footprint con el stamp de la imagen), no en
el de ahora: la imagen llega con unas decenas de ms de atraso.

Solo se mira mientras el robot avanza y no gira demasiado rápido. Girando, la imagen y el scan
no son del mismo instante: el robot sigue la pared girando casi todo el tiempo, así que en vez de
no mirar se corrige el sector del lidar por lo que giró el robot entre el rayo y la imagen (TF
odom -> base_footprint en los dos instantes).
"""
import math
from dataclasses import fields

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from equipo_jar.agrupador import AgrupadorCajas
from equipo_jar.deteccion import (
    FiltroColor, FiltroForma, angulo_de_columna, apoyada_en_el_piso, buscar_caja_roja, coinciden,
    distancia_por_altura, hay_algo_delante, mediana_del_tramo, profundidad, tramo_del_lidar,
    yaw_de_cuaternion)
from equipo_jar.evasion import normalizar_angulo
from equipo_jar.geometria import componer, de_robot_a_mapa
from equipo_jar.mapa import GrillaMapa
from equipo_jar.supervisor_localizacion import ConfigSupervisor, SupervisorLocalizacion

# /map lo publica el juez con esta QoS (docs/QOS.md).
QOS_MAPA = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Topic interno (no es del reglamento): una caja confirmada por mensaje, en el marco `map`.
TOPIC_CAJAS = '/equipo/cajas_confirmadas'
# Lo publica nuestro AMCL (es interno): de acá sale su covarianza.
TOPIC_POSE_AMCL = '/amcl_pose'

# Parámetros y su valor por defecto. Los valores de la corrida viven en params/params.yaml (§9.1).
PARAMETROS = {
    'frecuencia_hz': 10.0,
    'angulo_frente_deg': 180.0,
    # Cuándo se mira
    'velocidad_lineal_min_deteccion': 0.05,
    'giro_max_deteccion': 0.7,
    # Color (deteccion.FiltroColor)
    'saturacion_min': 120,
    'valor_min': 80,
    'hue_max_bajo': 10,
    'hue_min_alto': 170,
    'limpieza_px': 3,
    # Forma (deteccion.FiltroForma)
    'area_minima_px': 400.0,
    'margen_borde_px': 5,
    'relleno_min': 0.65,
    'proporcion_min': 0.55,
    'proporcion_max': 1.6,
    # Lidar
    'fraccion_central': 0.6,
    'tramo_min_deg': 3.0,
    'tolerancia_tramo_m': 0.10,
    'fraccion_tramo_min': 0.6,
    'ancho_oclusion_deg': 4.0,
    'salto_oclusion_m': 0.15,
    'distancia_min_deteccion_m': 0.35,
    'distancia_max_deteccion_m': 3.0,
    # Caja y cámara
    'semilado_caja_m': 0.125,
    'alto_caja_m': 0.30,
    'tolerancia_distancia': 0.25,
    'tolerancia_piso': 0.25,
    # Mapa
    'usar_mapa': True,
    'radio_libre_m': 0.15,
    'margen_vista_inicio_m': 0.15,
    'margen_vista_fin_m': 0.20,
    # Confirmación (agrupador.AgrupadorCajas)
    'confirmaciones': 5,
    'radio_grupo_m': 0.30,
    'dispersion_max_m': 0.15,
    'olvido_s': 3.0,
    'radio_duplicado_m': 0.60,
    # Frames y log
    'frame_mapa': 'map',
    'frame_odom': 'odom',
    'frame_robot': 'base_footprint',
    'log_descartes': True,
}


class NodoPercepcion(Node):
    def __init__(self):
        super().__init__('nodo_percepcion')
        for nombre, valor in PARAMETROS.items():
            self.declare_parameter(nombre, valor)
        # Los del supervisor: uno por campo de ConfigSupervisor, con el prefijo loc_.
        for campo in fields(ConfigSupervisor):
            self.declare_parameter('loc_' + campo.name, float(campo.default))

        self.color = FiltroColor(*(int(self.p(n)) for n in (
            'saturacion_min', 'valor_min', 'hue_max_bajo', 'hue_min_alto', 'limpieza_px')))
        self.forma = FiltroForma(
            float(self.p('area_minima_px')), int(self.p('margen_borde_px')),
            float(self.p('relleno_min')), float(self.p('proporcion_min')),
            float(self.p('proporcion_max')))
        self.supervisor = SupervisorLocalizacion(ConfigSupervisor(**{
            c.name: float(self.p('loc_' + c.name)) for c in fields(ConfigSupervisor)}))
        self.agrupador = AgrupadorCajas(
            int(self.p('confirmaciones')), self.p('radio_grupo_m'), self.p('dispersion_max_m'),
            self.p('olvido_s'), self.p('radio_duplicado_m'))

        self.puente = CvBridge()
        self.mapa = None              # GrillaMapa, cuando llega /map
        self.scan = None
        self.imagen = None
        self.stamp_imagen = None
        self.stamp_procesado = None   # no procesar dos veces la misma imagen
        self.info_camara = None
        self.velocidad = (0.0, 0.0)   # (lineal en m/s, giro en rad/s), de /odom
        self.sigmas_amcl = None       # (sigma_xy m, sigma_yaw rad) de la covarianza de AMCL
        self.estado_anterior = None
        self.giro_scan_deg = 0.0      # cuánto giró el robot entre el rayo del lidar y la imagen
        self.lidar_en_robot = None    # (x, y) del lidar en base_footprint, de TF
        self.altura_camara = None     # altura (m) de la cámara sobre el piso, de TF

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
        # AMCL lo publica reliable; reliable + volatile se conecta con cualquier durability.
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, TOPIC_POSE_AMCL, self.al_recibir_pose_amcl, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_cajas = self.create_publisher(PointStamped, TOPIC_CAJAS, 10)

        self.create_timer(1.0 / self.p('frecuencia_hz'), self.ciclo)
        self.get_logger().info('Percepción lista. Esperando el mapa en /map...')

    def p(self, nombre):
        return self.get_parameter(nombre).value

    # --- Sensores: los callbacks solo guardan el último dato; decide el timer ---------------

    def al_recibir_mapa(self, msg):
        self.mapa = GrillaMapa.desde_mensaje(msg)

    def al_recibir_scan(self, scan):
        self.scan = scan

    def al_recibir_imagen(self, msg):
        self.imagen = self.puente.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.stamp_imagen = msg.header.stamp

    def al_recibir_info(self, info):
        self.info_camara = info

    def al_recibir_odom(self, odom):
        t = odom.twist.twist
        self.velocidad = (math.hypot(t.linear.x, t.linear.y), t.angular.z)

    def al_recibir_pose_amcl(self, msg):
        cov = msg.pose.covariance
        self.sigmas_amcl = (math.sqrt(max(cov[0], cov[7], 0.0)), math.sqrt(max(cov[35], 0.0)))

    # --- TF ----------------------------------------------------------------------------------

    def transformada(self, destino, origen, tiempo=None):
        """TF origen -> destino en el instante `tiempo` (rclpy Time; None: la última), o None."""
        try:
            return self.tf_buffer.lookup_transform(
                destino, origen, Time() if tiempo is None else tiempo)
        except TransformException:
            return None

    def pose_tf(self, destino, origen, tiempo=None):
        """(x, y, yaw) de `origen` en `destino`, o None."""
        tf = self.transformada(destino, origen, tiempo)
        if tf is None:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_de_cuaternion(q.x, q.y, q.z, q.w)

    def correccion_amcl(self):
        """(map -> odom como (x, y, yaw), su stamp en s), o (None, None) si AMCL no publicó."""
        tf = self.transformada(self.p('frame_mapa'), self.p('frame_odom'))
        if tf is None:
            return None, None
        t, q = tf.transform.translation, tf.transform.rotation
        stamp = Time.from_msg(tf.header.stamp).nanoseconds * 1e-9
        return (t.x, t.y, yaw_de_cuaternion(q.x, q.y, q.z, q.w)), stamp

    def geometria_del_robot(self):
        """Dónde están montados el lidar y la cámara (TF estática del URDF). Una sola vez."""
        robot = self.p('frame_robot')
        if self.lidar_en_robot is None and self.scan is not None:
            tf = self.transformada(robot, self.scan.header.frame_id)
            if tf is not None:
                self.lidar_en_robot = (tf.transform.translation.x, tf.transform.translation.y)
        if self.altura_camara is None and self.info_camara is not None:
            tf = self.transformada(robot, self.info_camara.header.frame_id)
            if tf is not None:
                self.altura_camara = tf.transform.translation.z
                self.get_logger().info(
                    f'Cámara a {self.altura_camara:.3f} m del piso; lidar en '
                    f'{self.lidar_en_robot} respecto de {robot}')

    # --- Ciclo: se ejecuta a frecuencia_hz --------------------------------------------------

    def ciclo(self):
        if self.mapa is None:
            return  # nada antes del mapa (§5)
        self.geometria_del_robot()
        ahora = self.get_clock().now().nanoseconds * 1e-9
        odom = self.pose_tf(self.p('frame_odom'), self.p('frame_robot'))
        correccion, t_correccion = self.correccion_amcl()
        estado = self.supervisor.actualizar(
            ahora, odom, correccion, t_correccion, self.sigmas_amcl)
        self.informar(estado)
        if not estado.confiable:
            return

        lineal, giro = self.velocidad
        if (lineal < self.p('velocidad_lineal_min_deteccion')
                or abs(giro) > self.p('giro_max_deteccion')):
            return  # quieto o girando: la imagen y la pose no son del mismo instante
        if self.imagen is None or self.stamp_imagen == self.stamp_procesado:
            return  # la misma imagen del ciclo anterior no es una vista nueva
        self.stamp_procesado = self.stamp_imagen

        # La pose en el instante de la imagen. Si TF todavía no tiene ese instante, la última.
        odom_imagen = self.pose_tf(
            self.p('frame_odom'), self.p('frame_robot'), Time.from_msg(self.stamp_imagen)) or odom
        pose = componer(estado.correccion, odom_imagen)
        punto, motivo = self.ubicar_caja_roja(pose, odom_imagen[2])
        if punto is None:
            if self.p('log_descartes') and motivo != 'no hay rojo':
                self.get_logger().info(f'Descarto: {motivo}', throttle_duration_sec=1.0)
            return

        centro = self.agrupador.agregar(punto, ahora)
        if centro is not None:
            self.publicar_caja(*centro)
            self.get_logger().info(
                f'Caja {len(self.agrupador.confirmadas)} confirmada en '
                f'({centro[0]:.2f}, {centro[1]:.2f}) con pose de {estado.fuente}')

    def informar(self, estado):
        """Loguea los cambios de fuente de la pose. Al cambiar, las vistas sin confirmar se
        tiran: venían de otra pose y mezclarlas daría un promedio de dos lugares."""
        anterior = self.estado_anterior
        self.estado_anterior = estado
        if anterior is not None and (anterior.fuente, anterior.confiable) == (
                estado.fuente, estado.confiable):
            return
        self.agrupador.olvidar_pendientes()
        texto = f'Pose: {estado.fuente} ({estado.motivo})'
        if estado.confiable:
            self.get_logger().info(texto)
        else:
            self.get_logger().warn(texto + ': no ubico cajas')

    def tramo(self, desde_deg, hasta_deg):
        """Rayos del lidar entre dos ángulos vistos desde la imagen (corregidos por el giro)."""
        s = self.scan
        return tramo_del_lidar(s.ranges, s.angle_min, s.angle_increment,
                               self.p('angulo_frente_deg'), desde_deg + self.giro_scan_deg,
                               hasta_deg + self.giro_scan_deg, s.range_min, s.range_max)

    def giro_desde_el_scan(self, angulo_deg, yaw_odom_imagen):
        """Grados que giró el robot desde que el lidar midió en la dirección `angulo_deg` (desde
        el frente) hasta que se tomó la imagen.

        Un objeto quieto que en la imagen está a b grados, cuando lo midió el lidar estaba a
        b + (yaw en la imagen - yaw en el rayo). Cada rayo tiene su instante: stamp del scan +
        índice * time_increment (si el driver lo informa; si no, el stamp).
        """
        s = self.scan
        tiempo = Time.from_msg(s.header.stamp)
        if s.time_increment > 0.0 and s.angle_increment > 0.0:
            desde_min = math.radians(self.p('angulo_frente_deg') + angulo_deg) - s.angle_min
            i = int(round((desde_min % (2.0 * math.pi)) / s.angle_increment)) % len(s.ranges)
            tiempo = Time(nanoseconds=tiempo.nanoseconds + int(i * s.time_increment * 1e9))
        odom_rayo = self.pose_tf(self.p('frame_odom'), self.p('frame_robot'), tiempo)
        if odom_rayo is None:
            return 0.0
        return math.degrees(normalizar_angulo(yaw_odom_imagen - odom_rayo[2]))

    def ubicar_caja_roja(self, pose, yaw_odom_imagen):
        """(Posición (x, y) del centro de la caja roja que se ve ahora, None), o (None, motivo)."""
        if self.info_camara is None or self.scan is None:
            return None, 'faltan datos de la cámara o del lidar'
        mancha, motivo = buscar_caja_roja(self.imagen, self.color, self.forma)
        if mancha is None:
            return None, motivo
        k = self.info_camara.k
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]

        # 1. Ángulo de la caja respecto del frente (la cámara mira al frente del robot).
        angulo = angulo_de_columna(mancha.columna, fx, cx)
        izquierda = math.degrees(angulo_de_columna(mancha.x0, fx, cx))
        derecha = math.degrees(angulo_de_columna(mancha.x1, fx, cx))
        self.giro_scan_deg = self.giro_desde_el_scan(math.degrees(angulo), yaw_odom_imagen)

        # 2. Distancia que ve el lidar en esa dirección: la mediana del tramo central de la caja.
        mitad = max(self.p('tramo_min_deg'),
                    (izquierda - derecha) * self.p('fraccion_central')) / 2.0
        centro_deg = math.degrees(angulo)
        distancia, parejos = mediana_del_tramo(
            self.tramo(centro_deg - mitad, centro_deg + mitad), self.p('tolerancia_tramo_m'))
        if distancia is None:
            return None, 'el lidar no ve nada en esa dirección'
        if parejos < self.p('fraccion_tramo_min'):
            return None, f'rayos dispares ({parejos:.0%} parejos): es un borde'
        if not (self.p('distancia_min_deteccion_m') <= distancia
                <= self.p('distancia_max_deteccion_m')):
            return None, f'a {distancia:.2f} m: fuera del rango útil'

        # 3. ¿Algo delante, justo al costado de la mancha? Entonces la caja está tapada a medias.
        ancho = self.p('ancho_oclusion_deg')
        salto = self.p('salto_oclusion_m')
        if (hay_algo_delante(self.tramo(izquierda, izquierda + ancho), distancia, salto)
                or hay_algo_delante(self.tramo(derecha - ancho, derecha), distancia, salto)):
            return None, 'tapada en parte por algo más cerca'

        # 4. Contraste con la cámara: alto y posición de una caja en el piso a esa profundidad.
        prof = profundidad(distancia, angulo)
        alto_caja = self.p('alto_caja_m')
        if mancha.entera_en_alto and not coinciden(
                prof, distancia_por_altura(mancha.alto, fy, alto_caja),
                self.p('tolerancia_distancia')):
            return None, 'el alto en la imagen no coincide con la distancia del lidar'
        if self.altura_camara is not None and not apoyada_en_el_piso(
                mancha, prof, fy, cy, self.altura_camara, alto_caja, self.p('tolerancia_piso')):
            return None, 'no está apoyada en el piso a esa distancia'

        # 5. El lidar ve la cara de la caja; el punto que puntúa es su centro (§6.1).
        lx, ly = self.lidar_en_robot or (0.0, 0.0)
        hasta_cara = distancia
        hasta_centro = distancia + self.p('semilado_caja_m')
        cara = de_robot_a_mapa(pose, lx + hasta_cara * math.cos(angulo),
                               ly + hasta_cara * math.sin(angulo))
        centro = de_robot_a_mapa(pose, lx + hasta_centro * math.cos(angulo),
                                 ly + hasta_centro * math.sin(angulo))

        # 6. El mapa: una caja no puede estar adentro de una pared ni verse a través de una.
        if self.p('usar_mapa'):
            if not self.mapa.hay_libre_cerca(*centro, self.p('radio_libre_m')):
                return None, 'caería adentro de una pared o afuera del mapa'
            if not self.mapa.linea_de_vista(pose[:2], cara, self.p('margen_vista_inicio_m'),
                                            self.p('margen_vista_fin_m')):
                return None, 'en el mapa hay una pared entre el robot y la caja'
        return centro, None

    def publicar_caja(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = self.p('frame_mapa')
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
