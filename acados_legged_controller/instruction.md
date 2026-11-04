# Comandi utili

In ogni nuovo terminale:

```bash
source ~/colcon_ws/install/setup.bash
```

## Compilare

```bash
# Compila il controller
cd ~/colcon_ws
colcon build --symlink-install --packages-select acados_legged_controller --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash

# Ricompila forzando CMake (se file generati o CMakeLists non vengono rilevati)
colcon build --symlink-install --packages-select acados_legged_controller --cmake-force-configure --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Compila il plugin del robot reale e le sue dipendenze
colcon build --symlink-install --packages-up-to legged_unitree_hw --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Dopo ogni compilazione riavvia la simulazione: il controller viene caricato all'avvio di Gazebo.

## Rigenerare il codice acados

Serve quando cambi `lipm_model.py`, oppure pesi, limiti o dimensioni dell'OCP in `config.yaml`.

```bash
cd ~/colcon_ws/src/legged_control/acados_legged_controller/scripts
LD_LIBRARY_PATH=$HOME/acados/lib PYTHONPATH= ~/opti_pessi_acados/acados_env/bin/python model_gen.py
```

Poi ricompila. Le posizioni degli ostacoli e i parametri del controller non richiedono `model_gen.py`.

## Simulazione

```bash
# Avvia Gazebo, broadcaster, controller, odom_to_tf e RViz
ros2 launch acados_legged_controller sim_full.launch.py

# Chiude i processi rimasti aperti
pkill -f "gz sim"
pkill -f rviz2
```

## Robot reale (laboratorio)

File usati: `launch/real_hw.launch.py`, `launch/real_controller.launch.py`, `config/ros2_controllers_real.yaml`, `config/config.yaml` (lo stesso della simulazione).
I limiti di velocità e di durata del passo sono nel codice generato: cambiarli in `config.yaml` non ha effetto senza `model_gen.py`.

### Rete

Portatile collegato via cavo (adattatore USB-C → Ethernet) a una porta libera del robot. Nessun altro programma deve inviare comandi low-level a `192.168.123.10`.

```bash
# Nome dell'interfaccia dell'adattatore (di solito enx...)
ip -br link

# IP statico sulla rete del robot (una volta sola, poi resta salvato)
sudo nmcli con add type ethernet ifname <enx...> con-name unitree ipv4.method manual ipv4.addresses 192.168.123.162/24
sudo nmcli con up unitree

# In ogni terminale: ROS 2 solo in locale, così non si mescola con altri ROS in laboratorio
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### Prima di accendere

```bash
# Il PC deve stare sulla rete del robot (IP 192.168.123.x)
ping 192.168.123.10

# Priorità real-time massima: se è 0, thread_priority: 95 non viene applicato (solo un warning)
ulimit -r
```

Per abilitarla: aggiungi `elia - rtprio 98` in `/etc/security/limits.conf`, poi logout e login.

### Procedura

Ogni terminale: `source ~/colcon_ws/install/setup.bash`.

**Terminale 1, robot appeso:** hardware, sensori e RViz, senza controller. I motori sono passivi (Kp=Kd=tau=0).

```bash
ros2 launch acados_legged_controller real_hw.launch.py

# Opzioni: SDK 3.8.0, soglia contatti, limite di potenza, senza RViz
ros2 launch acados_legged_controller real_hw.launch.py unitree_sdk:=3_8_0 contact_threshold:=30 power_limit:=7 rviz:=false
```

Nel log controlla: `UnitreeHW` senza errori UDP, *Receiving the robot state*, i due broadcaster attivi.

**Terminale 1b:** carica il controller **inattivo**, il robot non si muove.

```bash
ros2 launch acados_legged_controller real_controller.launch.py
```

Nel log controlla: lo spawner che dice *Loaded*, le due masse (`config.yaml` e URDF) simili. `ros2 control list_controllers` deve mostrarlo `inactive`.

**Terminale 2:** emergenza. Scrivilo e tienilo pronto **senza premere invio**.

```bash
ros2 service call /acados_legged_controller/damping std_srvs/srv/Trigger
```

**Terminale 3:** registra tutto per l'analisi a casa.

```bash
mkdir -p ~/colcon_ws/bags && cd ~/colcon_ws/bags
ros2 bag record -a -o lab_$(date +%H%M)
```

**Terminale 4:** grafici dal vivo, con lo stesso layout della simulazione (`/monitor` lo pubblica il controller).

```bash
ros2 run plotjuggler plotjuggler -l ~/colcon_ws/src/legged_control/acados_legged_controller/config/monitor.xml
```

Se non parte da solo: **Streaming** → **ROS2 Topic Subscriber** → **Start** → `/monitor`. I grafici restano vuoti finché il controller non è attivo.

**Terminale 5:** controlli e comandi, passo per passo.

```bash
# 1. Sensori, robot appeso: piega una gamba a mano e guarda giunto e segno
ros2 topic echo /joint_states --once

# 2. Forze ai piedi: appeso sotto contact_threshold (40), appoggiato sopra
ros2 topic echo /contact

# 3. Attivazione, ancora appeso: RECOVER, STAND_UP, STANCE
ros2 control switch_controllers --activate acados_legged_controller

# 4. A terra, in STANCE: la posizione stimata deve restare ferma
ros2 topic echo /odom --field pose.pose.position

# 5. Primi passi: goal nel frame odom, l'origine è dove è partito lo stimatore
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 0.3, y: 0.0}}}"

# Oppure verso c_target di config.yaml
ros2 service call /acados_legged_controller/start_walking std_srvs/srv/Trigger

# Stop controllato
ros2 service call /acados_legged_controller/stop std_srvs/srv/Trigger
```

Alla fine: `Ctrl+C` nel terminale 3 per chiudere la bag, poi nel terminale 1.

## Muovere il robot

- **RViz:** usa il tool `2D Goal Pose` e clicca sul piano.
- **Terminale:** il comando qui sotto invia lo stesso goal nel frame `odom`.

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.5}}}"
```

## Servizi del controller

| Comando | Cosa fa |
|---|---|
| `ros2 service call /acados_legged_controller/start_walking std_srvs/srv/Trigger` | Da STANCE parte verso il goal corrente, inizialmente `c_target`. |
| `ros2 service call /acados_legged_controller/stop std_srvs/srv/Trigger` | Stop controllato: frena al capture point e torna in STANCE. |
| `ros2 service call /acados_legged_controller/damping std_srvs/srv/Trigger` | Stop di emergenza: smorzamento immediato, il controller resta attivo. Per ripartire serve il ciclo qui sotto. |

## Attivare e disattivare il controller

```bash
# Stato dei controller
ros2 control list_controllers

# Disattiva: smorzamento, stimatore e MPC si fermano, quindi RViz resta fermo
ros2 control switch_controllers --deactivate acados_legged_controller

# Attiva: RECOVER, poi STAND_UP e STANCE
ros2 control switch_controllers --activate acados_legged_controller
```

Se il robot è inclinato oltre `max_tilt`, l'attivazione passa in `DAMPING`: va raddrizzato prima.

## Spinte in Gazebo

```bash
cd ~/colcon_ws/src/legged_control/acados_legged_controller

# Forza di 30 N lungo x per 0,3 s
bash push.sh x 30 0.3

# Forza di 30 N lungo -y per 0,2 s
bash push.sh y -30 0.2
```

Parametri: asse, forza in N, durata in s.

## Grafici con PlotJuggler

```bash
# Con il layout salvato
ros2 run plotjuggler plotjuggler -l ~/colcon_ws/src/legged_control/acados_legged_controller/config/monitor.xml

# Senza layout
ros2 run plotjuggler plotjuggler
```

Serve `sudo apt install ros-jazzy-plotjuggler-ros`. Per i dati dal vivo: **Streaming** → **ROS2 Topic Subscriber** → **Start** → `/monitor`.

## Topic principali

| Topic | Contenuto |
|---|---|
| `/goal_pose` | Goal del robot. |
| `/odom` | Stima dello stato; twist nel frame della base. Si ferma se il controller è inattivo. |
| `/joint_states` | Posizione, velocità e sforzo dei giunti. |
| `/monitor` | Coppie comandate, velocità locali e limiti. |
| `/mpc_predictions` | Traiettorie: blu ottimistica, arancione pessimistica. |
| `/mpc_obstacles` | Ostacoli e zona di sicurezza. |

```bash
ros2 topic list                  # elenca i topic
ros2 topic hz /monitor           # misura la frequenza
ros2 topic echo /odom --once     # stampa un messaggio
ros2 topic info /odom -v         # mostra publisher, subscriber e QoS
```

## Log e registrazioni

```bash
# Registra una prova in ~/colcon_ws/bags
mkdir -p ~/colcon_ws/bags
cd ~/colcon_ws/bags
ros2 bag record /monitor /odom /mpc_predictions /mpc_obstacles

# Mostra il contenuto e riproduce una registrazione
ros2 bag info <cartella>
ros2 bag play <cartella>

# Mostra i log più recenti
ls -t ~/.ros/log | head
```

## Diagnostica

```bash
# Controlla che il binario sia più recente del sorgente
ls -laL --time-style=long-iso ~/colcon_ws/install/acados_legged_controller/lib/libacados_legged_controller.so ~/colcon_ws/src/legged_control/acados_legged_controller/src/AcadosLeggedController.cpp

# Posizione reale del robot in Gazebo
gz model -m aliengo -p

# Real time factor di Gazebo
gz topic -e -t /stats -n 1
```

Nel log cerca `[Acados MPC]: solver failed` per i fallimenti del solver e `[Safety]: DAMPING` per il motivo di uno stop.

## Parametri da ricordare

File: `config/config.yaml`.

| Parametro | Uso |
|---|---|
| `obstacles.instances` | Ostacoli `static`, `dynamic`, `adversarial`. Con `[]` non ce ne sono. |
| `obstacles.random.count` | Numero di ostacoli random. Con `0` sono spenti. |
| `obstacles.random.seed` | Stesso seed, stessi ostacoli. |
| `simulation.goal_tolerance` | Distanza dal goal per entrare in STANCE. |
| `simulation.goal_speed_tolerance` | Velocità massima per entrare in STANCE. |
| `controller.damping_kd` | Smorzamento dei giunti in `DAMPING`. |
| `controller.max_tilt` | Inclinazione che attiva `DAMPING`. |

`instances` più `random.count` non possono superare `N_OBS_MAX`. Modificare `N_OBS_MAX` richiede `model_gen.py`.
