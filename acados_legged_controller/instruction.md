# Comandi utili

In ogni nuovo terminale:

```bash
source ~/colcon_ws/install/setup.bash
```

## Compilare

```bash
cd ~/colcon_ws

# Controller e obstacle_simulator
colcon build --symlink-install --packages-select acados_legged_controller --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Se CMake non vede file generati o modifiche al CMakeLists
colcon build --symlink-install --packages-select acados_legged_controller --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Plugin del robot reale e dipendenze
colcon build --symlink-install --packages-up-to legged_unitree_hw --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source install/setup.bash
```

Dopo ogni compilazione riavvia il controller (in simulazione: riavvia `sim_full.launch.py`).

## Rigenerare il codice acados

Serve quando cambi `lipm_model.py`, oppure pesi, limiti, `N_horizon`, `N_OBS_MAX` o `max_iter` in `config.yaml`.

```bash
cd ~/colcon_ws/src/legged_control/acados_legged_controller/scripts
LD_LIBRARY_PATH=$HOME/acados/lib PYTHONPATH= ~/opti_pessi_acados/acados_env/bin/python model_gen.py
```

Poi ricompila. Ostacoli, goal e parametri della sezione `controller` non richiedono `model_gen.py`.

## Simulazione

```bash
# Gazebo, broadcaster, controller (si attiva da solo), odom_to_tf e RViz
ros2 launch acados_legged_controller sim_full.launch.py

# Con un altro mondo di legged_gazebo/worlds
ros2 launch acados_legged_controller sim_full.launch.py world:=warehouse_world.world

# Salva il log per cercare errori
ros2 launch acados_legged_controller sim_full.launch.py 2>&1 | tee ~/sim_log.txt
grep -nE "ERROR|WARN|Failed|died|exception|terminate" ~/sim_log.txt | head -50

# Chiude i processi rimasti aperti
pkill -f "gz sim"
pkill -f rviz2
```

## Robot reale (laboratorio)

### Rete (una volta sola)

Portatile collegato via cavo (adattatore USB-C → Ethernet) a una porta libera del robot. Nessun altro programma deve inviare comandi low-level a `192.168.123.10`.

```bash
# Nome dell'interfaccia dell'adattatore (di solito enx...)
ip -br link

# IP statico sulla rete del robot, resta salvato
sudo nmcli con add type ethernet ifname <enx...> con-name unitree ipv4.method manual ipv4.addresses 192.168.123.162/24
sudo nmcli con up unitree
```

Priorità real-time (`thread_priority: 95` in `legged_unitree_hw/config/aliengo.yaml`): aggiungi `elia - rtprio 98` in `/etc/security/limits.conf`, poi logout e login.

### Prima di accendere

```bash
# In ogni terminale: ROS 2 solo in locale, così non si mescola con altri ROS in laboratorio
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Il PC deve vedere il robot
ping 192.168.123.10

# Deve essere 98, non 0
ulimit -r
```

### Avvio

**Terminale 1: hardware.** Avvia `ros2_control_node` con il plugin UnitreeHW e i controller di `ros2_controllers.yaml`.

```bash
ros2 launch legged_unitree_hw legged_unitree_hw_launch.xml robot_type:=aliengo \
  controllers_yaml:=$(ros2 pkg prefix --share acados_legged_controller)/config/ros2_controllers.yaml

# Opzioni: SDK 3.8.0, soglia contatti, limite di potenza (1-10), senza sensori nell'URDF
#   unitree_sdk:=3_8_0 contact_threshold:=40 power_limit:=10 camera:=false lidar:=false
```

Nel log controlla: `UnitreeHW` senza errori UDP e *Receiving the robot state*.

**Terminale 2: controller.** Avvia broadcaster, controller, `odom_to_tf`, il relay dei goal e RViz.

> Il controller si **attiva da solo** appena caricato (`on_init` dura circa un minuto) e parte subito con STAND_UP. Prima di lanciarlo, metti il robot nella posizione da cui deve alzarsi.

```bash
ros2 launch acados_legged_controller load_controller_launch.xml use_sim_time:=false

# Con gli ostacoli finti di obstacle_simulator (vedi sotto)
ros2 launch acados_legged_controller load_controller_launch.xml use_sim_time:=false \
  params_file:=$(ros2 pkg prefix --share acados_legged_controller)/config/obstacles.yaml
```

- `explore.yaml` (default): goal dei planner, il robot guarda solo nella direzione di marcia (`useGoalYaw: false`).
- `obstacles.yaml`: raggiunge anche l'orientamento di `/goal_pose`.

Nel log controlla: le due masse (`config.yaml` e URDF) simili, `Solver initialized`, poi `STANCE`.

**Terminale 3: registrazione.**

```bash
mkdir -p ~/colcon_ws/bags && cd ~/colcon_ws/bags
ros2 bag record -a -o lab_$(date +%H%M)
```

**Terminale 4: grafici dal vivo.**

```bash
ros2 run plotjuggler plotjuggler -l ~/colcon_ws/src/legged_control/acados_legged_controller/config/monitor.xml
```

Se non parte da solo: **Streaming** → **ROS2 Topic Subscriber** → **Start** → `/monitor`.

**Terminale 5: controlli e comandi.**

```bash
# Sensori: piega una gamba a mano e guarda giunto e segno
ros2 topic echo /joint_states --once

# In STANCE la posizione stimata deve restare ferma
ros2 topic echo /odom --field pose.pose.position

# Primi passi: goal nel frame odom, l'origine è dove è partito lo stimatore
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 0.3, y: 0.0}}}"
```

### Fermare

```bash
# Ferma il controller (MPC e stimatore si fermano)
ros2 control switch_controllers --deactivate acados_legged_controller

# Riattiva: STAND_UP, poi STANCE
ros2 control switch_controllers --activate acados_legged_controller

ros2 control list_controllers
```

In questo branch **non ci sono** i servizi `stop` e `damping`, né il controllo di inclinazione. Tieni il robot assicurato e lo stop di emergenza a portata di mano.

Alla fine: `Ctrl+C` nel terminale 3 per chiudere la bag, poi nel terminale 2 e nel terminale 1.

## Goal

- **RViz:** tool `2D Goal Pose`, clicca sul piano (la freccia dà anche il yaw, se `useGoalYaw` è attivo).
- **Terminale:**

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.5}}}"

# Da STANCE verso c_target di config.yaml
ros2 service call /acados_legged_controller/start_walking std_srvs/srv/Trigger
```

I planner (TARE, FAR) pubblicano su `/opti_pessi/goal`, e `opti_pessi_goal_relay.py` lo inoltra a `/goal_pose`.

## Ostacoli

Il controller legge gli ostacoli dal topic `/opti_pessi/obstacles` (`obstaclesFromTopic: true` in `config/ros2_controllers.yaml`): gli ostacoli di `config.yaml` non vengono simulati dentro il controller.
`obstacle_simulator` muove gli ostacoli di `obstacles.instances` di `config.yaml` e li pubblica su quel topic, nel frame `odom`.

```bash
# Robot reale
ros2 run acados_legged_controller obstacle_simulator

# Simulazione
ros2 run acados_legged_controller obstacle_simulator --ros-args -p use_sim_time:=true

# Partono fermi: avvia, metti in pausa
ros2 service call /obstacle_simulator/set_moving std_srvs/srv/SetBool "{data: true}"
ros2 service call /obstacle_simulator/set_moving std_srvs/srv/SetBool "{data: false}"

# Controlli
ros2 topic hz /opti_pessi/obstacles
ros2 topic echo /opti_pessi/obstacles --once
```

Parametri del nodo: `rate` (50 Hz), `autostart` (false), `obstacle_topic`, `frame_id`, `lipmConfigFile`.
Il raggio usato dal controller è quello della classe (`obstacle_types` di `config.yaml`: `human`, `car`), non `r_obs` dell'istanza.
In RViz: `/mpc_obstacles` (quelli visti dall'MPC, con la zona di sicurezza) e `/obstacle_simulator/markers`.

Senza ostacoli: non lanciare `obstacle_simulator`. Il controller avvisa che non arriva nessun messaggio e cammina senza ostacoli.

## Topic principali

| Topic | Contenuto |
|---|---|
| `/goal_pose` | Goal del robot (frame `odom`). |
| `/opti_pessi/obstacles` | Ostacoli per l'MPC (`legged_controllers/ObstacleArray`). |
| `/odom` | Stima dello stato. Si ferma se il controller è inattivo. |
| `/joint_states` | Posizione, velocità e sforzo dei giunti. |
| `/monitor` | Coppie comandate, velocità, limiti e tracking del WBC (PlotJuggler). |
| `/mpc_predictions` | Traiettorie: ottimistica piena, pessimistica trasparente. |
| `/mpc_obstacles` | Ostacoli e zona di sicurezza. |

```bash
ros2 topic list
ros2 topic hz /monitor
ros2 topic info /odom -v
```

## Bag e log

```bash
ros2 bag info <cartella>
ros2 bag play <cartella>
ls -t ~/.ros/log | head
```

Nel log cerca `solver failed` per i fallimenti dell'MPC, `solve_avg` per il tempo di calcolo e `Safety check failed` per uno stop del controller.

## Diagnostica

```bash
# Il binario deve essere più recente del sorgente
ls -laL --time-style=long-iso ~/colcon_ws/install/acados_legged_controller/lib/libacados_legged_controller.so ~/colcon_ws/src/legged_control/acados_legged_controller/src/AcadosLeggedController.cpp

# Simulazione: posizione reale del robot e real time factor
gz model -m aliengo -p
gz topic -e -t /stats -n 1
```

## Parametri da ricordare

| File | Parametro | Uso |
|---|---|---|
| `config.yaml` | `obstacles.instances` | Ostacoli di `obstacle_simulator`. |
| `config.yaml` | `obstacle_types` | Raggio e velocità massima per classe. |
| `config.yaml` | `simulation.goal_tolerance`, `heading.yaw_tolerance` | Distanza e yaw dal goal per tornare in STANCE. |
| `config.yaml` | `simulation.feet_tolerance`, `max_settle_steps` | Piedi sotto le anche prima di fermarsi. |
| `config.yaml` | `controller.*` | Guadagni, durate di STAND_UP e STANCE, altezza del passo. |
| `ros2_controllers.yaml` | `update_rate` | Frequenza in simulazione (il reale usa `legged_unitree_hw/config/aliengo.yaml`). |
| `ros2_controllers.yaml` | `obstaclesFromTopic`, `obstaclesTimeout`, `obstaclesMaxAge` | Ostacoli dal topic e gonfiaggio con l'età del messaggio. |

Il numero di ostacoli non può superare `N_OBS_MAX`. Modificare `N_OBS_MAX` richiede `model_gen.py`.
