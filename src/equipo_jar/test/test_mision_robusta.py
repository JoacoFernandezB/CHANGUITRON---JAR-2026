import math

from equipo_jar.nodo_mision import NodoMision
from geometry_msgs.msg import PointStamped
import pytest
import rclpy


@pytest.fixture
def nodo():
    rclpy.init()
    n = NodoMision()
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def test_un_error_en_el_ciclo_no_mata_el_nodo_y_frena(nodo):
    frenos = []
    nodo.pub_cmd.publish = frenos.append

    def falla():
        raise TypeError('map -> odom no disponible')
    nodo.paso = falla
    nodo.ciclo()  # no tiene que lanzar la excepción
    assert len(frenos) == 1
    assert frenos[0].linear.x == 0.0 and frenos[0].angular.z == 0.0


def test_el_ciclo_llama_a_paso(nodo):
    llamadas = []
    nodo.paso = lambda: llamadas.append(1)
    nodo.ciclo()
    assert llamadas == [1]


@pytest.mark.parametrize('x, y', [(math.nan, 1.0), (1.0, math.inf), (-math.inf, math.nan)])
def test_una_caja_no_finita_no_gasta_un_reporte(nodo, x, y):
    publicados = []
    nodo.pub_victimas.publish = publicados.append
    nodo.t_inicio = nodo.ahora()
    nodo.registrar_caja(x, y)
    assert publicados == []
    assert nodo.reportes == 0
    assert nodo.confirmadas == []


def test_una_caja_valida_se_reporta_una_sola_vez(nodo):
    publicados = []
    nodo.pub_victimas.publish = publicados.append
    nodo.t_inicio = nodo.ahora()
    nodo.registrar_caja(1.0, 2.0)
    nodo.registrar_caja(1.05, 2.0)  # la misma caja
    assert len(publicados) == 1
    assert isinstance(publicados[0], PointStamped)
    assert nodo.reportes == 1
