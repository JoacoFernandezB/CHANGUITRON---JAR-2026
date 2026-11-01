import math

from equipo_jar.geometria import componer, diferencia, inversa
from equipo_jar.supervisor_localizacion import (
    AMCL, CONVERGIENDO, RESPALDO, SIN_POSE, SOLO_ODOM, ConfigSupervisor, SupervisorLocalizacion)

CONVERGIDA = (0.05, math.radians(2.0))   # sigmas de un AMCL ya convergido
CORRECCION = (0.2, -0.15, 0.3)           # map -> odom real: el robot arrancó corrido del origen


def recta(t):
    """Robot a 0,3 m/s en línea recta por el eje x de odom."""
    return (0.3 * t, 0.0, 0.0)


def correr(supervisor, desde, hasta, correccion_amcl, odom=recta, dt=0.1, sigmas=CONVERGIDA):
    """Corre el supervisor de `desde` a `hasta` (s); `correccion_amcl(t)` es lo que publica AMCL."""
    estados = []
    t = desde
    while t < hasta - 1e-9:
        c = correccion_amcl(t)
        estados.append((t, supervisor.actualizar(t, odom(t), c, t if c else None, sigmas)))
        t += dt
    return estados


def error(estado, t, odom=recta):
    """Error de la pose que da el supervisor contra la pose real del robot."""
    real = componer(CORRECCION, odom(t))
    return diferencia(componer(estado.correccion, odom(t)), real)


def test_sin_amcl_espera_y_despues_usa_solo_odometria():
    sup = SupervisorLocalizacion(ConfigSupervisor(espera_sin_amcl_s=5.0))
    estados = correr(sup, 0.0, 6.0, lambda t: None)
    assert estados[0][1].fuente == SIN_POSE and not estados[0][1].confiable
    assert estados[-1][1].fuente == SOLO_ODOM and estados[-1][1].correccion == (0.0, 0.0, 0.0)


def test_no_confia_en_amcl_hasta_que_converge():
    sup = SupervisorLocalizacion(ConfigSupervisor())
    estado = sup.actualizar(0.0, recta(0.0), CORRECCION, 0.0, (0.15, math.radians(10.0)))
    assert estado.fuente == CONVERGIENDO and not estado.confiable
    estado = sup.actualizar(0.1, recta(0.1), CORRECCION, 0.1, CONVERGIDA)
    assert estado.fuente == AMCL and estado.confiable


def test_sin_covarianza_acepta_amcl_tras_avanzar():
    sup = SupervisorLocalizacion(ConfigSupervisor(avance_convergencia_m=1.0))
    estados = correr(sup, 0.0, 3.5, lambda t: CORRECCION, sigmas=None)  # 1,05 m
    assert estados[0][1].fuente == CONVERGIENDO
    assert estados[-1][1].fuente == AMCL


def test_quieto_amcl_no_converge_y_no_se_acepta_aunque_pase_el_tiempo():
    # Con el robot quieto AMCL no se actualiza: esperar más tiempo no sirve de nada.
    sup = SupervisorLocalizacion(ConfigSupervisor(avance_convergencia_m=1.0))
    estados = correr(sup, 0.0, 60.0, lambda t: CORRECCION, odom=lambda t: (0.0, 0.0, 0.0),
                     sigmas=None)
    assert estados[-1][1].fuente == CONVERGIENDO and not estados[-1][1].confiable


def test_amcl_que_anda_bien_no_dispara_el_respaldo():
    # AMCL con ruido chico (3 cm, 2°) en la pose del robot, que es lo que estima AMCL.
    sup = SupervisorLocalizacion(ConfigSupervisor())

    def ruidosa(t):
        x, y, yaw = componer(CORRECCION, recta(t))
        pose_amcl = (x + 0.03 * math.sin(t), y + 0.03 * math.cos(3 * t),
                     yaw + math.radians(2.0) * math.sin(2 * t))
        return componer(pose_amcl, inversa(recta(t)))

    estados = correr(sup, 0.0, 60.0, ruidosa)
    assert all(e.fuente == AMCL for _, e in estados)


def test_un_salto_de_amcl_pasa_al_respaldo_y_la_pose_sigue_bien():
    # Como en la corrida del 2026-10-30 01:19: AMCL salta 3 m y 120° y vuelve a los 9 s.
    sup = SupervisorLocalizacion(ConfigSupervisor())
    perdida = componer((1.0, -3.0, 2.1), CORRECCION)

    def amcl(t):
        return perdida if 20.0 <= t < 29.0 else CORRECCION

    estados = correr(sup, 0.0, 40.0, amcl)
    durante = [e for t, e in estados if 20.0 <= t < 29.0]
    assert all(e.fuente == RESPALDO and e.confiable for e in durante)
    for t, e in estados:
        d_m, d_rad = error(e, t)
        assert d_m < 0.05 and d_rad < math.radians(1.0)
    assert estados[-1][1].fuente == AMCL  # volvió a coincidir


def test_una_deriva_lenta_de_amcl_tambien_se_detecta():
    # Como en la corrida del 2026-10-30 01:57: AMCL se va de a 0,2 m/s, de a pasitos.
    sup = SupervisorLocalizacion(ConfigSupervisor())

    def amcl(t):
        corrimiento = max(0.0, t - 20.0) * 0.2
        return (CORRECCION[0], CORRECCION[1] + corrimiento, CORRECCION[2])

    estados = correr(sup, 0.0, 30.0, amcl)
    assert estados[-1][1].fuente == RESPALDO
    for t, e in estados:
        assert error(e, t)[0] < 0.20  # la tolerancia, sin que la deriva se cuele de a poco


def test_una_deriva_de_amcl_girando_en_el_lugar_se_detecta():
    # Como en maze_4 (2026-10-31): el robot gira en el lugar y AMCL se corre 0,07 m/s.
    # Girar no hace derivar la posición de /odom, así que no puede agrandar esa tolerancia.
    sup = SupervisorLocalizacion(ConfigSupervisor())
    correr(sup, 0.0, 10.0, lambda t: CORRECCION)
    quieto = recta(10.0)

    def girando(t):
        return (quieto[0], quieto[1], 1.0 * (t - 10.0))

    def amcl(t):
        return (CORRECCION[0] + 0.07 * (t - 10.0), CORRECCION[1], CORRECCION[2])

    estados = correr(sup, 10.0, 25.0, amcl, odom=girando)
    assert estados[-1][1].fuente == RESPALDO
    assert all(error(e, t, odom=girando)[0] < 0.15 for t, e in estados)


def test_la_deriva_normal_de_la_odometria_no_es_un_salto():
    # La odometría se equivoca 3% en la distancia: AMCL lo va corrigiendo y eso es normal.
    sup = SupervisorLocalizacion(ConfigSupervisor())

    def odom_mala(t):
        return (0.3 * t * 1.03, 0.0, 0.0)

    def amcl(t):  # la corrección que dejaría a la pose en el lugar real
        return componer(componer(CORRECCION, recta(t)), inversa(odom_mala(t)))

    estados = correr(sup, 0.0, 60.0, amcl, odom=odom_mala)
    assert all(e.fuente == AMCL for _, e in estados)


def test_si_amcl_deja_de_publicar_usa_el_respaldo():
    sup = SupervisorLocalizacion(ConfigSupervisor(max_antiguedad_s=1.0))
    correr(sup, 0.0, 5.0, lambda t: CORRECCION)
    # map -> odom quedó con el stamp de t = 5 s y no se actualiza más.
    estado = sup.actualizar(7.0, recta(7.0), CORRECCION, 5.0, CONVERGIDA)
    assert estado.fuente == RESPALDO
    assert estado.correccion == CORRECCION


def test_mucho_recorrido_en_respaldo_ya_no_es_confiable():
    sup = SupervisorLocalizacion(ConfigSupervisor(avance_max_respaldo_m=2.0))
    correr(sup, 0.0, 5.0, lambda t: CORRECCION)
    estados = correr(sup, 5.0, 20.0, lambda t: None)  # AMCL muerto: 4,5 m más
    assert estados[0][1].confiable
    assert not estados[-1][1].confiable
    assert estados[-1][1].avance_respaldo_m >= 4.5


def test_amcl_firme_en_otro_lugar_con_poco_recorrido_no_se_acepta():
    # Quieto o casi quieto, un AMCL equivocado puede quedarse "firme": no alcanza.
    sup = SupervisorLocalizacion(ConfigSupervisor(estable_para_reaceptar_s=5.0))
    correr(sup, 0.0, 5.0, lambda t: CORRECCION)
    nueva = (CORRECCION[0] + 1.0, CORRECCION[1], CORRECCION[2])
    estados = correr(sup, 5.0, 16.0, lambda t: nueva, odom=lambda t: recta(min(t, 8.0)))
    assert estados[-1][1].fuente == RESPALDO


def test_si_amcl_se_relocaliza_de_verdad_se_acepta():
    # La referencia quedó mal (la odometría patinó feo) y AMCL se queda firme en otro lugar.
    cfg = ConfigSupervisor(estable_para_reaceptar_s=5.0)
    sup = SupervisorLocalizacion(cfg)
    correr(sup, 0.0, 5.0, lambda t: CORRECCION)
    nueva = (CORRECCION[0] + 1.0, CORRECCION[1], CORRECCION[2])
    estados = correr(sup, 5.0, 16.0, lambda t: nueva)  # 11 s y 3,3 m: firme y andando
    assert estados[0][1].fuente == RESPALDO
    assert estados[-1][1].fuente == AMCL and estados[-1][1].correccion == nueva
