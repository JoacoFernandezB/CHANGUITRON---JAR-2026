"""Publicador del leaderboard con artefactos sintéticos (sin ROS ni red).

Los artefactos imitan los del CI (`simulador-<mapa>-<run>-<intento>/<mapa>/...`,
con `equipo: null`) y algunos casos de un fork que arma artefactos a mano.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from publicar_ranking import main as publicar  # noqa: E402
from validar_resultado import validar  # noqa: E402

PNG = b'\x89PNG\r\n\x1a\n' + b'0' * 32
SHA = 'a' * 40


def resultado(total=0, tiempo=60.1, puntos=(0, 0), mapa_ok=True):
    return {
        'termino_por': 'inmovil', 'mapa_publicado': mapa_ok, 'reportes': [],
        'reportes_mal_formados': [], 'antes_del_mapa': {'cmd_vel_no_nulo': 0, 'reportes': 0},
        'violaciones': [], 'resumen': f'total {total}', 'victimas': [[0, 1], [2, -2]],
        'eventos': [], 'equipo': None, 'duracion_s': tiempo, 'tiempo_total_s': tiempo,
        'tiempo_ultimo_reporte_que_puntua_s': None, 'movimiento_despues_de_done': 0,
        'medidas_con_ground_truth': True,
        'puntaje': {'por_victima': [{'indice': i, 'indice_reporte': None, 'distancia': None,
                                     'puntos': p} for i, p in enumerate(puntos)],
                    'reportes_contados': 0, 'reportes_ignorados': 0, 'puntos_victimas': total,
                    'bono_rapido': False, 'bono_regreso': False, 'penalizacion_colisiones': 0,
                    'penalizacion_salida': 0, 'total': total, 'anulada': False,
                    'tiempo_ultimo_reporte_que_puntua': None},
    }


def artefacto(base, run, mapa, contenido, png=PNG):
    d = base / f'simulador-{mapa}-{run}-1' / mapa
    d.mkdir(parents=True)
    (d / 'resultado.json').write_text(json.dumps(contenido))
    if png is not None:
        (d / 'grafico.png').write_bytes(png)


def correr(tmp, art, run='1', pr='7', owner='osos', nombre='Los Osos', verde='si', toca='no'):
    yaml = tmp / f'equipo-{owner}.yaml'
    yaml.write_text(f'nombre_equipo: {nombre}\ncodigo_inscripcion: X1\n')
    sys.argv = ['publicar_ranking.py', '--artefactos', str(art), '--sitio', str(tmp / 'sitio'),
                '--comentario', str(tmp / 'c.md'), '--run-id', run, '--pr', pr, '--sha', SHA,
                '--repo', 'O/R', '--base-url', 'https://p', '--equipo-id', owner,
                '--equipo-yaml', str(yaml), '--verde', verde, '--toca-infra', toca]
    assert publicar() == 0
    return (tmp / 'c.md').read_text()


def historial(tmp):
    return (tmp / 'sitio' / 'data' / 'historial.jsonl').read_text().splitlines()


def test_corrida_real_publica_con_el_equipo_del_fork(tmp_path):
    art = tmp_path / 'art'
    artefacto(art, '1', 'maze_1_6x5', resultado())
    artefacto(art, '1', 'maze_3_6x6', resultado(puntos=(0, 0, 0)))
    com = correr(tmp_path, art)
    hist = [json.loads(linea) for linea in historial(tmp_path)]
    assert {c['equipo'] for c in hist} == {'osos'}  # el dueño del fork, no el null del resultado
    index = (tmp_path / 'sitio' / 'index.html').read_text()
    assert index.count('<table>') == 2 and 'Los Osos' in index  # una tabla por mapa
    assert (tmp_path / 'sitio' / 'runs' / '1-maze_1_6x5' / 'grafico.png').exists()
    assert (tmp_path / 'sitio' / '.nojekyll').exists()
    assert com.startswith('<!-- jar-ranking -->')
    assert 'https://p/runs/1-maze_1_6x5/grafico.png' in com and 'Entra a la' in com


def test_ci_en_rojo_comenta_pero_no_entra_a_la_tabla(tmp_path):
    art = tmp_path / 'art'
    artefacto(art, '1', 'maze_1_6x5', resultado(total=100, puntos=(100, 0)))
    com = correr(tmp_path, art, verde='no')
    assert len(historial(tmp_path)) == 1
    index = (tmp_path / 'sitio' / 'index.html').read_text()
    assert '<table>' not in index and '(CI en rojo)' in index
    assert (tmp_path / 'sitio' / 'corridas' / '1-maze_1_6x5.html').exists()
    assert 'no entra a la [tabla de práctica]' in com and 'grafico.png' in com


def test_artefactos_armados_a_mano_no_se_publican(tmp_path):
    art = tmp_path / 'art'
    malo = resultado(total=99999)
    malo['puntaje']['por_victima'][0]['puntos'] = '<img src=x onerror=alert(1)>'
    artefacto(art, '1', 'maze_1_6x5', malo)
    artefacto(art, '1', 'm"><img src=x onerror=alert(2)>', resultado(total=500))
    artefacto(art, '1', 'maze_3_6x6', resultado(), png=b'<svg onload=alert(3)>')
    com = correr(tmp_path, art, nombre='<b>osos</b>')
    hist = [json.loads(linea) for linea in historial(tmp_path)]
    assert [c['mapa'] for c in hist] == ['maze_3_6x6']  # el único válido
    assert not hist[0]['grafico']  # no era un PNG
    sitio = tmp_path / 'sitio'
    todo = ''.join(p.read_text() for p in sitio.rglob('*.html'))
    assert 'onerror' not in todo and '<b>osos</b>' not in todo and '&lt;b&gt;osos' in todo
    assert '99999' not in todo and 'onerror' not in com and 'nombre de mapa inválido' in com


def test_pr_que_toca_tools_no_publica(tmp_path):
    art = tmp_path / 'art'
    artefacto(art, '1', 'maze_1_6x5', resultado(total=400))
    com = correr(tmp_path, art, toca='si')
    assert not (tmp_path / 'sitio').exists()
    assert 'no se publican' in com


def test_republicar_no_duplica_y_la_tabla_ordena_por_mapa(tmp_path):
    art1 = tmp_path / 'art1'
    artefacto(art1, '1', 'maze_1_6x5', resultado(total=100, tiempo=150, puntos=(100, 0)))
    correr(tmp_path, art1, owner='osos', nombre='Osos')
    com = correr(tmp_path, art1, owner='osos', nombre='Osos')  # misma corrida otra vez
    assert len(historial(tmp_path)) == 1 and 'grafico.png' in com
    art2 = tmp_path / 'art2'
    artefacto(art2, '2', 'maze_1_6x5', resultado(total=100, tiempo=100, puntos=(100, 0)))
    correr(tmp_path, art2, run='2', pr='8', owner='lobos', nombre='Lobos')
    art3 = tmp_path / 'art3'  # peor corrida de osos: no cambia su mejor
    artefacto(art3, '3', 'maze_1_6x5', resultado(total=0))
    correr(tmp_path, art3, run='3', owner='osos', nombre='Osos')
    assert len(historial(tmp_path)) == 3
    index = (tmp_path / 'sitio' / 'index.html').read_text()
    tabla = index.split('<h2>Todas las corridas')[0]
    assert tabla.index('>Lobos<') < tabla.index('>Osos<')  # mismo puntaje, menos tiempo
    assert 'corridas/1-maze_1_6x5.html' in tabla  # el detalle de osos es su mejor corrida


def test_validador_chequea_tipos(tmp_path):
    ruta = tmp_path / 'r.json'
    ruta.write_text(json.dumps(resultado()))
    assert validar(ruta) == []
    for cambio in (lambda d: d.update(tiempo_total_s='<script>'),
                   lambda d: d['puntaje'].update(total='400'),
                   lambda d: d['puntaje']['por_victima'][0].update(distancia='x'),
                   lambda d: d.update(reportes=5),
                   lambda d: d.update(mapa_publicado='si')):
        d = resultado()
        cambio(d)
        ruta.write_text(json.dumps(d))
        assert validar(ruta), cambio
