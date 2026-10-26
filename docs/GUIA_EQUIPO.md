# Guía del equipo CHANGUITRON

Cómo trabajamos en este repo. El [README](../README.md) es el del template oficial
(`AIRclub-UdeSA/jar_challenge`) y el [reglamento](https://airclub-udesa.github.io/jar_site/reglamento/)
es la fuente de verdad: si algo de acá los contradice, ganan ellos.

**Fechas:** clasificación el miércoles 4 de noviembre, final el jueves 5 (Rosario).
La organización toma el commit vigente del PR al cierre de entregas.

---

## 1. Setup (una vez por persona)

Necesitás ROS 2 Humble + Gazebo Fortress + el simulador, ya sea en Ubuntu 22.04 nativo o con el
contenedor Docker del simulador ([guía de setup](https://airclub-udesa.github.io/jar_site/setup/simulador/)).

Este repo va **dentro de `src/` del workspace**, al lado de `yahboom_rosmaster`. Con Docker es
obligatorio: el contenedor solo ve la carpeta del workspace (`~/rosmaster_ws` →
`/home/alumno/rosmaster_ws`).

```text
rosmaster_ws/
└── src/
    ├── yahboom_rosmaster/          simulador oficial, en el tag jar2026-v1
    └── CHANGUITRON---JAR-2026/     este repo
        └── src/equipo_jar/         nuestro paquete ROS 2
```

```bash
cd ~/rosmaster_ws/src
git clone git@github.com:JoacoFernandezB/CHANGUITRON---JAR-2026.git
cd CHANGUITRON---JAR-2026
git remote add upstream https://github.com/AIRclub-UdeSA/jar_challenge.git
```

El CI valida contra el simulador `jar2026-v1`: conviene que `yahboom_rosmaster` esté en ese tag o
apenas después (`git -C ~/rosmaster_ws/src/yahboom_rosmaster describe --tags`).

## 2. El día a día

Todos los comandos de ROS van **dentro del contenedor** (`./container.sh enter`) o en una terminal
con Humble.

**Compilar** (desde la raíz del workspace, como en los workshops):

```bash
cd ~/rosmaster_ws
colcon build --symlink-install --packages-select equipo_jar
source install/setup.bash
```

Con `--symlink-install` los cambios a `.py`, launch y `params/` se ven sin recompilar. Hay que
volver a compilar si agregás un ejecutable nuevo en `setup.py` o un archivo nuevo en `launch/` o
`params/`.

**Probar como en la corrida** (tres terminales):

```bash
# 1. Simulador (spawn_x/y/yaw: los jueces nos ponen cerca del origen, no exacto)
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)/worlds/maze_1_6x5_victimas.world \
  spawn_x:=0.15 spawn_y:=-0.1 spawn_yaw:=0.2

# 2. Juez de práctica: publica /map a los 5 s, puntúa en vivo y dice cuántas víctimas hay (N)
cd ~/rosmaster_ws/src/CHANGUITRON---JAR-2026
SIM=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)
python3 tools/juez.py --mundo maze_1_6x5 --mapa $SIM/maps/maze_1_6x5.yaml --ground-truth

# 3. Nuestro software, con el N que dijo el juez
ros2 launch equipo_jar competencia.launch.py use_sim_time:=true victimas:=N
```

Mapas de práctica: `laberinto_simple`, `laberinto_1`, `maze_1_6x5`, `maze_2_6x5`, `maze_3_6x6`,
`maze_4_metal_6x6`. Probá en **todos**: en Rosario el laberinto es otro.

**Antes de abrir un PR**, los mismos controles que el CI:

```bash
cd ~/rosmaster_ws/src/CHANGUITRON---JAR-2026
tools/verificar.sh --template                  # rápido, sin simulador (~1 min)
tools/verificar.sh --template --sim maze_1_6x5 # además, una corrida headless puntuada
```

`--template` saltea el chequeo de `equipo.yaml`, que queda con `codigo_inscripcion: COMPLETAR`
hasta que lo carguemos. `verificar.sh` compila adentro del repo (`build/`, `install/`, `log/`, todo
ignorado por git); no molesta al build del workspace.

## 3. Reglas que no se negocian

Romper cualquiera de estas cuesta puntos o la entrega entera:

- **Interfaz fija** (§7.1). Solo `/map`, `/victimas`, `/done`, `/cmd_vel`, `/scan`, `/odom`,
  `/imu/data`, `/cam_1/...`, `/joint_states`, `/tf`, `/tf_static`. Entre nuestros nodos, topics libres.
  Nada de ground truth del simulador.
- **QoS** ([docs/QOS.md](QOS.md)). `/scan` y cámaras: `qos_profile_sensor_data`. `/map`: reliable +
  transient local. Lo que publicamos: la QoS por defecto (`create_publisher(msg, topic, 10)`).
  Una QoS mal puesta no tira error: simplemente no llega nada.
- **Nada antes de `/map`.** No mover el robot ni reportar hasta recibir el mapa.
- **Exactamente N reportes** (§7.2). Cada mensaje a `/victimas` gasta uno, puntúe o no. Siempre
  con `armar_reporte` de `reportes.py` (`frame_id: map`, sin NaN). Que **un solo nodo** publique en
  `/victimas` y lleve la cuenta.
- **Nada hardcodeado** (§10): ni mapas, ni coordenadas, ni waypoints. En `params/` solo umbrales,
  ganancias, velocidades, rangos de color. N llega por `victimas:=N`, no va en `params/`.
- **`use_sim_time`** (§7.3): todos los nodos lo reciben del launch. El robot real no publica `/clock`.
- **Watchdog de `/cmd_vel`**: el robot se frena si pasan 0,5 s sin comando. Tras `/done`, frenar y
  no volver a mover.
- **Robot quieto 60 s = fin de corrida** (§11.3). Si un nodo se cae, no se puede reiniciar (§11.4).
- **Todo adentro de la imagen.** Corre sin internet: dependencias de ROS en `package.xml`
  (`<exec_depend>`), de Python en `requirements.txt` con versión fija.
- **No tocar `tools/` ni `.github/`.** La organización usa su propia copia, y un PR que los modifica
  no entra al leaderboard de práctica.

## 4. Git: cómo trabajamos

- `main` siempre compila y pasa `tools/verificar.sh`. Nadie pushea directo a `main`.
- Una branch por tarea: `feature/localizacion`, `feature/deteccion-rojo`, `fix/qos-scan`, etc.
- PR contra `main` de este repo, con al menos una revisión de otra persona. El workflow
  `Validar entrega` del template corre solo en cada PR.
- Commits chicos y con mensaje claro. No commitear `build/`, `install/`, `log/` ni rosbags.

**Traer cambios del template oficial** (la organización lo sigue actualizando):

```bash
git fetch upstream
git checkout main && git merge upstream/main
```

## 5. La entrega (§9.1)

La entrega es un PR desde **un fork de `AIRclub-UdeSA/jar_challenge`** hacia ese repo. Este repo no
es un fork en GitHub, pero tiene la historia del template, así que se puede pushear tal cual:

1. Alguien del equipo hace *Fork* de `AIRclub-UdeSA/jar_challenge` en GitHub.
2. `git remote add entrega git@github.com:<usuario>/jar_challenge.git`
3. `git push entrega main`
4. Abrir el PR hacia `AIRclub-UdeSA/jar_challenge`, título `[Equipo] CHANGUITRON`, con el checklist
   del template completo.
5. Desde ahí, cada vez que `main` tenga algo mejor: `git push entrega main`. El PR no se mergea:
   la organización toma el último commit al cierre.

Conviene abrirlo temprano: cada push al PR corre el CI completo en los mundos de práctica y
publica el puntaje en el [leaderboard de práctica](https://airclub-udesa.github.io/jar_challenge/).
Antes, completar `codigo_inscripcion` en `equipo.yaml` (el CI del PR lo exige). El PR es público:
nada de mails ni datos personales.
