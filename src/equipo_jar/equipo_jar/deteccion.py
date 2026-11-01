"""Detectar una caja roja con la cámara y ubicarla con el lidar. Funciones puras, sin ROS.

El camino de una imagen a un punto en el mapa, con un filtro en cada paso:

1. Color (`mascara_rojo`): rojo en HSV, con los dos rangos de H. Después se limpia la máscara
   (apertura y cierre) para borrar los píxeles sueltos que aparecen en los bordes de otros
   colores: en una cámara real, el borde entre una caja amarilla y una pared oscura puede dar
   un hilo naranja-rojizo.
2. Forma (`buscar_caja_roja`): de las manchas rojas, una que parezca una caja vista de frente o
   de costado: bastante llena (no un hilo ni una L), con proporción ancho/alto de caja y sin
   tocar los costados de la imagen (si está cortada de costado, su centro está mal). Cortada
   arriba o abajo sí sirve: pasa siempre que el robot pasa cerca (a menos de 0,43 m la tapa queda
   afuera) y la columna sigue bien; solo no se usa el borde que falta.
3. Lidar (`tramo_del_lidar`, `mediana_del_tramo`): la distancia a la cara, con la mediana de los
   rayos del centro de la mancha. Si los rayos no se ponen de acuerdo (mitad pared, mitad caja)
   no se usa.
4. Oclusión (`hay_algo_delante`): si justo afuera de un costado de la mancha hay algo más cerca
   que la caja, una pared la tapa en parte y el centro de la mancha no es el de la caja.
5. Geometría de la cámara (`coinciden`, `apoyada_en_el_piso`): una caja de alto conocido, apoyada
   en el piso, a la profundidad que dice el lidar, tiene que verse de cierto alto y en ciertas
   filas de la imagen. Un rojo que no cumple (un cartel, algo detrás de una pared baja, una
   mancha en el piso) no es una caja.

Es la idea del workshop de la semana 04, simplificada: en vez de proyectar todo el scan sobre la
imagen, se mira el scan solo en la dirección donde la cámara vio el rojo.
https://airclub-udesa.github.io/jar_site/workshops/semana-04-deteccion-color/
"""
import math
from dataclasses import dataclass

import cv2
import numpy as np

from equipo_jar.evasion import normalizar_angulo


# --- 1. Color --------------------------------------------------------------------------------

@dataclass(frozen=True)
class FiltroColor:
    """Qué cuenta como rojo. Los valores salen de params/params.yaml."""

    saturacion_min: int = 120  # S mínima (0-255): descarta grises y rosas pálidos
    valor_min: int = 80        # V mínimo (0-255): descarta zonas muy oscuras
    hue_max_bajo: int = 10     # rojo pegado a 0: H en [0, hue_max_bajo]. Más arriba es naranja
    hue_min_alto: int = 170    # rojo pegado a 180: H en [hue_min_alto, 180]. Más abajo, magenta
    limpieza_px: int = 3       # lado del cuadrado para abrir y cerrar la máscara (0: no limpiar)


def mascara_rojo(imagen_bgr, filtro):
    """Máscara binaria (255 = rojo) de una imagen BGR.

    En HSV el color es un ángulo (H, de 0 a 179 en OpenCV) y el rojo queda partido en los dos
    extremos del círculo: por eso hacen falta dos rangos de H. Los distractores de otro color
    caen lejos: amarillo ~30, verde ~60, azul ~120. Los que pueden colarse son el naranja
    (H 10-25) y el magenta (H 140-170): `hue_max_bajo` y `hue_min_alto` son el límite.

    La limpieza: la *apertura* borra lo que es más fino que `limpieza_px` (puntos y bordes
    sueltos), y el *cierre* tapa agujeros chicos adentro de la caja (un reflejo).
    """
    hsv = cv2.cvtColor(imagen_bgr, cv2.COLOR_BGR2HSV)
    s, v = filtro.saturacion_min, filtro.valor_min
    cerca_de_cero = cv2.inRange(hsv, (0, s, v), (filtro.hue_max_bajo, 255, 255))
    cerca_de_180 = cv2.inRange(hsv, (filtro.hue_min_alto, s, v), (180, 255, 255))
    mascara = cv2.bitwise_or(cerca_de_cero, cerca_de_180)
    if filtro.limpieza_px > 1:
        nucleo = np.ones((filtro.limpieza_px, filtro.limpieza_px), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, nucleo)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, nucleo)
    return mascara


# --- 2. Forma --------------------------------------------------------------------------------

@dataclass(frozen=True)
class FiltroForma:
    """Cómo tiene que verse una caja en la imagen. Los valores salen de params/params.yaml."""

    area_minima_px: float = 400.0  # más chica: ruido, o tan lejos que no se puede ubicar
    margen_borde_px: int = 5       # a menos de esto de un borde, la caja está cortada ahí
    relleno_min: float = 0.65      # área de la mancha / área de su rectángulo: una caja ~0,9
    proporcion_min: float = 0.55   # ancho / alto. Caja de 0,25 x 0,30: de 0,83 (de frente)
    proporcion_max: float = 1.6    # a 1,18 (de costado, 45°). Tapada por una pared: menos


@dataclass(frozen=True)
class Mancha:
    """Una mancha roja en la imagen: su rectángulo (en píxeles) y su área."""

    x0: int
    y0: int
    ancho: int
    alto: int
    area: float
    cortada_arriba: bool = False   # la tapa queda fuera de la imagen: el alto no sirve
    cortada_abajo: bool = False    # la base queda fuera de la imagen

    @property
    def x1(self):
        return self.x0 + self.ancho

    @property
    def y1(self):
        return self.y0 + self.alto

    @property
    def columna(self):
        """Columna del centro."""
        return self.x0 + self.ancho / 2.0

    @property
    def relleno(self):
        return self.area / float(self.ancho * self.alto)

    @property
    def proporcion(self):
        return self.ancho / float(self.alto)

    @property
    def entera_en_alto(self):
        return not (self.cortada_arriba or self.cortada_abajo)


def motivo_de_descarte(mancha, ancho_imagen, alto_imagen, forma):
    """Por qué esta mancha no es una caja, o None si puede serlo. El texto sirve para el log."""
    if mancha.area < forma.area_minima_px:
        return 'chica'
    m = forma.margen_borde_px
    if mancha.x0 < m or mancha.x1 > ancho_imagen - m:
        return 'cortada por un costado de la imagen'
    if mancha.relleno < forma.relleno_min:
        return f'poco llena ({mancha.relleno:.2f})'
    # Angosta: tapada en parte de costado. Ancha: solo si está entera en alto (si le falta la
    # tapa, se ve más ancha que alta y está bien).
    if mancha.proporcion < forma.proporcion_min or (
            mancha.entera_en_alto and mancha.proporcion > forma.proporcion_max):
        return f'proporción {mancha.proporcion:.2f}'
    return None


def manchas_rojas(imagen_bgr, color, margen_borde_px=5):
    """Todas las manchas rojas de la imagen, de la más grande a la más chica."""
    alto_img = imagen_bgr.shape[0]
    contornos, _ = cv2.findContours(
        mascara_rojo(imagen_bgr, color), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    manchas = []
    for c in contornos:
        x, y, ancho, alto = cv2.boundingRect(c)
        # El área de los píxeles, no la del polígono del contorno (que para una mancha de N px
        # de lado da (N-1)^2 y castiga a las chicas).
        area = float(max(cv2.contourArea(c), 0.0)) + 0.5 * cv2.arcLength(c, True) + 1.0
        manchas.append(Mancha(x, y, ancho, alto, min(area, float(ancho * alto)),
                              cortada_arriba=y < margen_borde_px,
                              cortada_abajo=y + alto > alto_img - margen_borde_px))
    return sorted(manchas, key=lambda mancha: mancha.area, reverse=True)


def buscar_caja_roja(imagen_bgr, color, forma):
    """(Mancha, None) con la caja roja más grande que pasa los filtros de forma, o
    (None, motivo) con el motivo por el que se descartó la mancha roja más grande."""
    alto_img, ancho_img = imagen_bgr.shape[:2]
    primer_motivo = 'no hay rojo'
    for i, mancha in enumerate(manchas_rojas(imagen_bgr, color, forma.margen_borde_px)):
        motivo = motivo_de_descarte(mancha, ancho_img, alto_img, forma)
        if motivo is None:
            return mancha, None
        if i == 0:
            primer_motivo = motivo
    return None, primer_motivo


# --- 3. Lidar --------------------------------------------------------------------------------

def angulo_de_columna(columna, fx, cx):
    """Ángulo (rad) de una columna de la imagen respecto del eje de la cámara.

    Positivo hacia la izquierda del robot, que es el sentido de giro de ROS. `fx` y `cx` salen
    del topic de la cámara `camera_info` (matriz K: fx = k[0], cx = k[2]).
    """
    return math.atan2(cx - columna, fx)


def tramo_del_lidar(rangos, angulo_min, incremento, frente_deg, desde_deg, hasta_deg,
                    rango_min=0.0, rango_max=math.inf):
    """Rangos válidos del lidar entre dos ángulos medidos desde el frente (positivo, izquierda).

    frente_deg: ángulo del /scan que corresponde al frente (ver evasion.distancia_en_sector).
    """
    frente = math.radians(frente_deg)
    desde, hasta = sorted((math.radians(desde_deg), math.radians(hasta_deg)))
    centro, mitad = (desde + hasta) / 2.0, (hasta - desde) / 2.0
    dentro = []
    for i, r in enumerate(rangos):
        if not math.isfinite(r) or r < rango_min or r > rango_max:
            continue
        desde_el_frente = normalizar_angulo(angulo_min + i * incremento - frente)
        if abs(normalizar_angulo(desde_el_frente - centro)) <= mitad:
            dentro.append(r)
    return dentro


def mediana_del_tramo(rangos, tolerancia_m):
    """(mediana, fracción de rayos a menos de `tolerancia_m` de ella), o (None, 0) si no hay.

    Una cara de caja da rayos parejos: casi todos cerca de la mediana. Si el tramo agarra mitad
    caja y mitad pared (un borde), la fracción baja y la mediana no es de nadie.
    """
    if not rangos:
        return None, 0.0
    mediana = float(np.median(rangos))
    cerca = sum(1 for r in rangos if abs(r - mediana) <= tolerancia_m)
    return mediana, cerca / len(rangos)


def distancia_a_la_caja(rangos, angulo_min, incremento, frente_deg, centro_deg, ancho_deg,
                        rango_min=0.0, rango_max=math.inf):
    """Mediana de los rayos de un sector (centro y ancho en grados), o None si no hay rayos.

    La **mediana**, no el más corto: el borde de una pared que asoma en el sector, o un rayo
    suelto, darían una distancia mucho menor que la de la caja.
    """
    tramo = tramo_del_lidar(rangos, angulo_min, incremento, frente_deg,
                            centro_deg - ancho_deg / 2.0, centro_deg + ancho_deg / 2.0,
                            rango_min, rango_max)
    return mediana_del_tramo(tramo, math.inf)[0]


# --- 4. Oclusión -----------------------------------------------------------------------------

def hay_algo_delante(rangos_al_costado, distancia_caja, salto_m):
    """¿Algún rayo justo al costado de la caja ve algo más cerca que ella?

    Si la caja asoma detrás de una pared, el rayo que pasa justo por afuera del borde de la mancha
    pega en la pared, que está más cerca. Entonces la mancha es solo un pedazo de la caja: su
    centro no es el de la caja y el punto saldría corrido hasta medio lado.
    Del otro lado (la pared de atrás, el pasillo) los rayos dan más lejos y no molestan.
    """
    return any(r < distancia_caja - salto_m for r in rangos_al_costado)


# --- 5. Geometría de la cámara ---------------------------------------------------------------

def profundidad(distancia, angulo):
    """Profundidad (eje óptico) de un punto a `distancia` y `angulo` del eje de la cámara.

    El pinhole da profundidad, no distancia: una caja a 1 m pero 30° al costado está a 0,87 m de
    profundidad, y así se ve de alto en la imagen.
    """
    return distancia * math.cos(angulo)


def distancia_por_altura(alto_px, fy, alto_caja_m):
    """Profundidad de la caja (m) según cuántos píxeles de alto ocupa (modelo pinhole).

    Sirve para contrastar con el lidar: el alto no lo tapa una pared que asoma de costado, que es
    justo lo que engaña al lidar cuando la caja se ve doblando una esquina.
    """
    return fy * alto_caja_m / alto_px


def coinciden(distancia_lidar, distancia_imagen, tolerancia_relativa):
    """True si las dos estimaciones difieren menos que `tolerancia_relativa` de la de la imagen."""
    return abs(distancia_lidar - distancia_imagen) <= tolerancia_relativa * distancia_imagen


def filas_esperadas(prof, fy, cy, altura_camara_m, alto_caja_m):
    """(fila de arriba, fila de abajo) donde se vería una caja apoyada en el piso a `prof` m.

    La cámara mira horizontal (sin inclinación), a `altura_camara_m` del piso. El piso a
    profundidad z cae en la fila cy + fy * h / z; el borde de arriba de la caja, en
    cy - fy * (alto - h) / z.
    """
    abajo = cy + fy * altura_camara_m / prof
    arriba = cy - fy * (alto_caja_m - altura_camara_m) / prof
    return arriba, abajo


def apoyada_en_el_piso(mancha, prof, fy, cy, altura_camara_m, alto_caja_m, tolerancia_relativa):
    """¿La mancha está donde estaría una caja apoyada en el piso a la profundidad del lidar?

    Compara las filas de arriba y de abajo de la mancha con las esperadas (las que se ven: si la
    mancha está cortada arriba, solo la de abajo). La tolerancia es una fracción del alto
    esperado. Descarta rojos a la altura equivocada: un cartel en una pared, algo que se ve por
    encima de una pared baja, una marca en el piso que el lidar no toca.
    """
    arriba, abajo = filas_esperadas(prof, fy, cy, altura_camara_m, alto_caja_m)
    tolerancia = tolerancia_relativa * (abajo - arriba)
    arriba_bien = mancha.cortada_arriba or abs(mancha.y0 - arriba) <= tolerancia
    abajo_bien = mancha.cortada_abajo or abs(mancha.y1 - abajo) <= tolerancia
    return arriba_bien and abajo_bien


# --- Utilidades ------------------------------------------------------------------------------

def punto_en_el_plano(x_robot, y_robot, yaw_robot, angulo, distancia):
    """Posición (x, y) de un punto que está a `distancia` en la dirección `angulo` del robot."""
    direccion = yaw_robot + angulo
    return (x_robot + distancia * math.cos(direccion),
            y_robot + distancia * math.sin(direccion))


def yaw_de_cuaternion(x, y, z, w):
    """Ángulo de giro (yaw) en radianes de un cuaternión de orientación."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def cerca_de_alguno(punto, puntos, radio_m):
    """True si `punto` está a menos de `radio_m` de alguno de `puntos`."""
    return any(math.dist(punto, otro) < radio_m for otro in puntos)


def promedio(puntos):
    """Centro de una lista de puntos (x, y)."""
    return (float(np.mean([p[0] for p in puntos])), float(np.mean([p[1] for p in puntos])))
