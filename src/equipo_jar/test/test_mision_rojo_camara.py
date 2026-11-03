"""nodo_mision también va a mirar de cerca donde la cámara vio rojo y no hay caja todavía."""
import numpy as np
import pytest
import rclpy

from equipo_jar.cobertura import Cobertura
from equipo_jar.memoria_camara import ROJO
from equipo_jar.nodo_mision import NodoMision

RES = 0.05


def sala():
    grilla = np.zeros((int(3.0 / RES), int(4.0 / RES)), dtype=np.int16)
    grilla[0, :] = grilla[-1, :] = 100
    grilla[:, 0] = grilla[:, -1] = 100
    return grilla


@pytest.fixture
def nodo():
    rclpy.init()
    n = NodoMision()
    n.cobertura = Cobertura(sala(), RES, 0.0, 0.0, (1.0, 1.0), 0.17, 0.30)
    n.t_inicio = n.ahora()
    n.pose = (1.0, 1.0, 0.0)
    n.pub_cmd.publish = lambda cmd: None
    yield n
    n.destroy_node()
    rclpy.try_shutdown()


def con_rojo(nodo, *puntos):
    c = nodo.cobertura
    camara = np.zeros((c.alto, c.ancho), dtype=np.int16)
    for x, y in puntos:
        camara[c.celda(x, y)] = ROJO
    nodo.vistas_camara = camara


def test_el_rojo_de_la_camara_queda_para_inspeccionar(nodo):
    con_rojo(nodo, (3.0, 2.0), (3.05, 2.0))
    nodo.anotar_objetos(nodo.zonas_rojas(), nodo.ahora(), 'rojo de la cámara')
    assert len(nodo.pendientes) == 1
    assert nodo.pendientes[0] == pytest.approx((3.025, 2.0), abs=0.05)


def test_el_rojo_de_una_caja_ya_confirmada_no_se_vuelve_a_mirar(nodo):
    nodo.confirmadas.append((3.1, 2.0))
    con_rojo(nodo, (3.0, 2.0), (3.05, 2.0))
    nodo.anotar_objetos(nodo.zonas_rojas(), nodo.ahora(), 'rojo de la cámara')
    assert nodo.pendientes == []


def test_apagado_no_agrega_nada(nodo):
    from rclpy.parameter import Parameter
    nodo.set_parameters([Parameter('inspeccionar_rojo_camara', value=False)])
    con_rojo(nodo, (3.0, 2.0), (3.05, 2.0))
    assert nodo.zonas_rojas() == []
