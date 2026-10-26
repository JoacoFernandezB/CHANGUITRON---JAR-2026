# Arquitectura (propuesta, a decidir en equipo)

El template trae un solo nodo esqueleto (`nodo_equipo`) que espera `/map` y no hace nada más.
Este documento propone cómo partir el problema para trabajar en paralelo. **Nada de esto está
decidido todavía**: la primera tarea del equipo es elegir la opción de navegación (sección 2).

## 1. Módulos

La misión (§1 del reglamento) se parte en cuatro problemas casi independientes:

| Módulo | Qué resuelve | Entra | Sale | Base de los workshops |
| --- | --- | --- | --- | --- |
| **Localización** | Dónde estoy en `/map`, arrancando *cerca* del origen | `/map`, `/scan`, `/odom` | TF `map → odom` | 06 (filtro de partículas) o AMCL (09) |
| **Percepción** | Ver cajas rojas y estimar su centro en `map` | cámara, `/scan`, TF | candidatos de víctima (topic interno) | 04 (`detector`, `detector_scan`) |
| **Navegación** | Ir a un punto sin chocar, incluidos obstáculos que no están en el mapa | `/map`, `/scan`, pose | `/cmd_vel` | 07 + 08 (A*) o Nav2 (09) |
| **Misión** | Qué recorrer, cuándo reportar, cuándo volver y publicar `/done` | todo lo anterior | `/victimas`, `/done`, metas | 08 (`explorador`, cobertura) |

Contratos a respetar entre módulos:

- **Un solo nodo publica en `/victimas`** (el de misión) y lleva la cuenta de los N reportes.
  Percepción publica *candidatos* en un topic interno (por ejemplo `/equipo/candidatos`); misión
  los agrupa y decide cuándo una víctima está confirmada.
- **Un solo dueño de `/cmd_vel`.** Si dos nodos lo publican, el robot tiembla.
- Todos los nodos se levantan desde `competencia.launch.py`, con `parameters=[params, ...]` y
  `use_sim_time`.

## 2. La decisión grande: ¿Nav2 o navegación propia?

| | **A. Nav2** (workshop 09) | **B. Propia** (workshops 06–08) |
| --- | --- | --- |
| Localización | AMCL, probado y robusto | nuestro filtro de partículas (ya anda en sim) |
| Obstáculos no mapeados | los ve el costmap local solo | hay que detectarlos y replanificar a mano (07) |
| Esfuerzo hasta "anda" | bajo: configurar YAML | alto: planificador + control + recuperación |
| Entender qué falla | difícil: muchas piezas y parámetros | fácil: es nuestro código |
| Imagen Docker | +`nav2_bringup` en `package.xml`, imagen más pesada | sin dependencias nuevas |
| Riesgo en el robot real | pasillos angostos: hay que afinar el radio del robot y los costmaps | depende de cuánto lo probemos |

Con ~10 días hasta la clasificación, **A** suele ser lo más seguro para "moverse sin chocar", y el
trabajo propio se concentra en percepción y misión, que es donde están los puntos. **B** da más
control, pero se come el tiempo en reinventar el planificador y las recuperaciones.

Si vamos con Nav2: agregar `<exec_depend>nav2_bringup</exec_depend>` (y lo que haga falta) a
`package.xml` para que rosdep lo meta en la imagen, y sus parámetros en `params/` (sin poses
iniciales fijas: la pose inicial sale de "cerca del origen" o de una localización global).

## 3. Puntos de diseño que valen puntos

- **Precisión del reporte:** ≤ 0,25 m da 100, hasta 0,50 m da 50. Se mide contra el **centro** de la
  caja (0,25 × 0,25 m), no contra la cara que ve el LiDAR: sumar medio lado en la dirección del rayo.
- **No gastar reportes:** filtrar falsos positivos (bordes, paralaje) acumulando varias detecciones
  antes de confirmar. Distractores azules, amarillos y verdes: el umbral de rojo tiene que ser
  estricto.
- **Bono de velocidad (+200):** las N víctimas antes de los 5 minutos.
- **Bono de regreso (+100):** volver a ≤ 0,25 m de la largada y publicar `/done`. Guardar la pose
  inicial estimada al recibir `/map`.
- **Colisiones (−50 cada una):** márgenes de seguridad generosos antes que velocidad.

## 4. Reparto sugerido

Una persona (o pareja) por módulo, cada uno en su branch y su nodo, integrando en `main` por PR.
Lo primero en llegar a `main` debería ser el esqueleto de los cuatro nodos conectados por los topics
internos acordados, aunque no hagan nada: así cada uno puede probar el suyo dentro del launch
completo.
