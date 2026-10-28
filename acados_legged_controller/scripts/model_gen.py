import os
import yaml
import numpy as np

from lipm_model import create_lipm_ocp
from gait_planner import GaitPlanner


#Ho implementato un sistema funzionante in python in cui ci sono più oggetti che si muovono casualmente, il solver trova soluzioni solide e anche in tempi relativamente brevi, che crescono all’aumentare del numero di oggetti massimi accettabili dal robot. 
#Per ovviare a questo in caso di tanti oggetti si potrebbe creare un controllo che prenda in considerazione solo gli n ostacoli più vicini. 
#Per ottenere questo risultato ho aggiunto un local_target che guarda in direzione del target vero (che nel mio esempio è posto a 10 m) e fa sì che il solver esploda molto meno, perché prima vedeva un target lontanissimo e il costo era altissimo. Adesso è molto più solido e mi ha permesso di abbassare il numero di iterazioni massime. Con 1 oggetto solo giro sui 120Hz.

#Adesso devo implementare la stessa logica in cpp, per farlo però devo rendere automatica la generazione del c_generated_code, perché così è insostenibile, ogni volta devo andare avanti e indietro, cambiare config del python, rieseguire quello, tornare di là, cambiare config del python….macello.

#Potrei provare ad aggiungere lipm_model.py nel ws cpp, fare un piccolo script in python che generi il codice necessario e usare quello. Questo dovrà prendere le info da config.yaml, così è tutto unito.



 # Extract values from config.yaml
with open("/home/elia/colcon_ws/src/legged_control/acados_legged_controller/config/config.yaml", "r") as file:
    config = yaml.safe_load(file)



# --- CREATE OBJECT FOR THE CONFIGURATION ---
class DictToObject:
    """Convert the dictionary in an object that can be called 'cfg.robot.h_com'"""
    def __init__(self, dictionary):
        for key, value in dictionary.items():
            if isinstance(value, dict):
                setattr(self, key, DictToObject(value))
            else:
                setattr(self, key, value)


# Create object with all the stuff cfg.robot.h_com
cfg = DictToObject(config)




# Instantiate the gait planner
gait_planner = GaitPlanner(off_x=cfg.robot.off_x, off_y=cfg.robot.off_y, steps_per_phase=cfg.simulation.step_per_phase)

# Initialization of the state
X_sim = np.zeros(11)
X_sim[0:2] = cfg.robot.x_init
X_sim[2] = cfg.robot.theta_init
# Initialization of the gait
init_gait = gait_planner.get_gait_horizon(0, cfg.simulation.N_horizon)[0]
initial_hips = gait_planner.compute_hip_positions(X_sim[0:3], init_gait)
# Initial position of the selected feet exactly underneath the hips
X_sim[6:8] = initial_hips[0:2]
X_sim[8:10] = initial_hips[2:4]
X_aug = np.concatenate([X_sim, X_sim])


print("----- Generating the OCP -----")

# ---- Create the OCP ----
solver = create_lipm_ocp(
    cfg=cfg,
    N=cfg.simulation.N_horizon, 
    c_target=cfg.simulation.c_target, 
    x_init=X_aug
)


print("-----    -----")