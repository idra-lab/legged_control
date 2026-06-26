#!/usr/bin/env python3
import os
# Forza l'uso di XWayland (x11) e del software rendering per evitare i bug di scaling/copy mode di WSLg
os.environ['SDL_VIDEODRIVER'] = 'x11'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry  # <-- IMPORTANTE: Per leggere la vera odometria
from ocs2_msgs.msg import MpcFlattenedController
from std_msgs.msg import Bool, String
import pygame
import sys
import threading
import math

# Configurazione Finestra (1 metro = 100 pixel)
SCALE = 100
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = 100, 300

def to_pixels(x, y):
    try:
        x_f, y_f = float(x), float(y)
        if not math.isfinite(x_f) or not math.isfinite(y_f):
            return (OFFSET_X, OFFSET_Y)
        return int(OFFSET_X + x_f * SCALE), int(OFFSET_Y - y_f * SCALE)
    except Exception:
        return (OFFSET_X, OFFSET_Y)

def to_meters(px, py):
    try:
        return (float(px) - OFFSET_X) / SCALE, (OFFSET_Y - float(py)) / SCALE
    except Exception:
        return 0.0, 0.0

def val_to_x(val, max_val, min_val=0.0):
    ratio = (val - min_val) / (max_val - min_val)
    return int(580 + ratio * 180)

def x_to_val(x, max_val, min_val=0.0):
    ratio = (x - 580) / 180
    ratio = max(0.0, min(1.0, ratio))
    return min_val + ratio * (max_val - min_val)

def clean_trajectory(points):
    cleaned = []
    for p in points:
        try:
            x, y = float(p[0]), float(p[1])
            if math.isfinite(x) and math.isfinite(y):
                cleaned.append((x, y))
        except Exception:
            continue
    return cleaned

def draw_trajectory(screen, color, points_world, width, node):
    if len(points_world) < 2:
        return
    points_pix = []
    for wx, wy in points_world:
        px, py = to_pixels(wx, wy)
        if isinstance(px, int) and isinstance(py, int):
            points_pix.append((px, py))
    if len(points_pix) >= 2:
        try:
            pygame.draw.lines(screen, color, False, points_pix, width)
        except Exception as e:
            pass

class MpcInteractorGUI(Node):
    def __init__(self):
        super().__init__('mpc_interactor_gui')
        
        self.obs_pub = self.create_publisher(Point, '/obstacle_pose', 10)
        self.goal_pub = self.create_publisher(Point, '/goal_pose', 10)
        
        self.mpc_sub = self.create_subscription(MpcFlattenedController, '/legged_robot_mpc_policy', self.mpc_callback, 10)
        
        # --- SOTTOSCRIZIONE AL FEEDBACK REALE DI ODOMETRIA ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # self.is_running_wbc_pub = self.create_publisher(Bool, '/is_running_wbc', 10)
        self.robot_mode_pub = self.create_publisher(String, '/robot_mode', 10)
        
        # Coordinate reali misurate dal simulatore
        self.true_physics_x = 0.0
        self.true_physics_y = 0.0
        
        # Variabili di visualizzazione interna
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.goal_x = 5.0
        self.goal_y = 0.0
        self.obs_x = 2.5
        self.obs_y = 0.3
        self.obs_radius = 0.4
        self.obs_max_vel = 0.5
        
        self.traj_optimistic = []
        self.traj_pessimistic = []
        self.dragging_obstacle = False
        self.is_running = True
        self.robot_mode = 'lie'  # 'lie', 'stand', 'walk'
        
        # Real-time joint gains
        self.kp_kd_pub = self.create_publisher(Point, '/joint_kp_kd', 10)
        self.kp = 0.0
        self.kd = 0.0
        self.dragging_kp = False
        self.dragging_kd = False
        
        self.create_timer(0.02, self.update_loop) # Sincronizzato a 50Hz con l'MPC

    def odom_callback(self, msg: Odometry):
        """ Riceve la posizione fisica reale del robot da Isaac Sim """
        self.true_physics_x = msg.pose.pose.position.x
        self.true_physics_y = msg.pose.pose.position.y
        
        # Aggiorna le coordinate locali per il disegno Pygame
        self.robot_x = self.true_physics_x
        self.robot_y = self.true_physics_y

    def mpc_callback(self, msg: MpcFlattenedController):
        """ Memorizza le traiettorie predette dall'MPC unicamente per disegnarle """
        self.traj_optimistic = []
        for s in msg.state_trajectory:
            if len(s.value) >= 11:
                self.traj_optimistic.append((s.value[9], s.value[10]))
        self.traj_pessimistic = []
        
        # --- COMENTATO E RIMOSSO IL BUG DI TELEPORTAZIONE CINEMATICA ---
        # Il robot Python non salta più in avanti anticipando la fisica reale!

    def update_loop(self):
        # Pubblica sempre il robot_mode corrente
        mode_msg = String()
        mode_msg.data = self.robot_mode
        self.robot_mode_pub.publish(mode_msg)

        # Pubblica sempre i valori Kp e Kd correnti
        kp_kd_msg = Point()
        kp_kd_msg.x = float(self.kp)
        kp_kd_msg.y = float(self.kd)
        kp_kd_msg.z = 0.0
        self.kp_kd_pub.publish(kp_kd_msg)

        if self.is_running:
            obs_msg = Point()
            obs_msg.x = self.obs_x
            obs_msg.y = self.obs_y
            obs_msg.z = self.obs_max_vel
            self.obs_pub.publish(obs_msg)

            goal_msg = Point()
            goal_msg.x = self.goal_x
            goal_msg.y = self.goal_y
            goal_msg.z = 0.0
            self.goal_pub.publish(goal_msg)

# Definizione pulsanti modalità robot
BUTTON_DEFS = [
    {'label': 'LIE',   'mode': 'lie',   'color': (180, 80,  80),  'hover': (220, 110, 110)},
    {'label': 'STAND', 'mode': 'stand', 'color': (80,  130, 200), 'hover': (110, 160, 230)},
    {'label': 'WALK',  'mode': 'walk',  'color': (60,  170, 90),  'hover': (90,  200, 120)},
]
BTN_W, BTN_H = 110, 38
BTN_Y = HEIGHT - 70
BTN_MARGIN = 14
BTN_START_X = WIDTH - (BTN_W * 3 + BTN_MARGIN * 2) - 16

def get_button_rect(idx):
    x = BTN_START_X + idx * (BTN_W + BTN_MARGIN)
    return pygame.Rect(x, BTN_Y, BTN_W, BTN_H)

def draw_buttons(screen, font_big, current_mode, mouse_pos):
    for idx, btn in enumerate(BUTTON_DEFS):
        rect = get_button_rect(idx)
        is_active = (current_mode == btn['mode'])
        is_hover  = rect.collidepoint(mouse_pos)
        color = btn['hover'] if is_hover else btn['color']
        border_color = (255, 255, 255) if is_active else (50, 50, 50)
        border_w = 3 if is_active else 1
        pygame.draw.rect(screen, color, rect, border_radius=8)
        pygame.draw.rect(screen, border_color, rect, border_w, border_radius=8)
        label = font_big.render(btn['label'], True, (255, 255, 255))
        lx = rect.x + (BTN_W - label.get_width()) // 2
        ly = rect.y + (BTN_H - label.get_height()) // 2
        screen.blit(label, (lx, ly))

def draw_sliders(screen, font, node, mouse_pos):
    # Draw background panel card (Slate Dark Theme)
    panel_rect = pygame.Rect(530, 15, 250, 115)
    pygame.draw.rect(screen, (30, 41, 59), panel_rect, border_radius=10)
    pygame.draw.rect(screen, (71, 85, 105), panel_rect, 2, border_radius=10)

    # Title
    title = font.render("PARAMETRI PID GIUNTI", True, (241, 245, 249))
    screen.blit(title, (545, 22))

    # Kp Slider (Y = 62)
    kp_lbl = font.render("Kp", True, (148, 163, 184))
    screen.blit(kp_lbl, (545, 53))
    
    # Track
    pygame.draw.line(screen, (71, 85, 105), (580, 62), (760, 62), 4)
    kp_x = val_to_x(node.kp, 80.0)
    pygame.draw.line(screen, (13, 148, 136), (580, 62), (kp_x, 62), 4)
    
    # Knob
    kp_knob_rect = pygame.Rect(kp_x - 8, 62 - 8, 16, 16)
    is_hover = kp_knob_rect.collidepoint(mouse_pos) or node.dragging_kp
    knob_color = (20, 184, 166) if is_hover else (241, 245, 249)
    pygame.draw.circle(screen, knob_color, (kp_x, 62), 8)
    
    # Value text
    kp_val_text = font.render(f"{node.kp:.1f}", True, (241, 245, 249))
    screen.blit(kp_val_text, (768, 53))

    # Kd Slider (Y = 97)
    kd_lbl = font.render("Kd", True, (148, 163, 184))
    screen.blit(kd_lbl, (545, 88))
    
    # Track
    pygame.draw.line(screen, (71, 85, 105), (580, 97), (760, 97), 4)
    kd_x = val_to_x(node.kd, 10.0)
    pygame.draw.line(screen, (13, 148, 136), (580, 97), (kd_x, 97), 4)
    
    # Knob
    kd_knob_rect = pygame.Rect(kd_x - 8, 97 - 8, 16, 16)
    is_hover = kd_knob_rect.collidepoint(mouse_pos) or node.dragging_kd
    knob_color = (20, 184, 166) if is_hover else (241, 245, 249)
    pygame.draw.circle(screen, knob_color, (kd_x, 97), 8)
    
    # Value text
    kd_val_text = font.render(f"{node.kd:.1f}", True, (241, 245, 249))
    screen.blit(kd_val_text, (768, 88))

def pygame_loop(node):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Opti-Pessi MPC Dashboard Fisica Reale")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)
    font_big = pygame.font.SysFont(None, 28)

    running = True
    while rclpy.ok() and running:
        mouse_pos = pygame.mouse.get_pos()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = pygame.mouse.get_pos()
                handled = False
                
                # Check sliders click first
                if 530 <= mx <= 780 and 45 <= my <= 75:
                    node.dragging_kp = True
                    node.kp = x_to_val(mx, 80.0)
                    handled = True
                elif 530 <= mx <= 780 and 80 <= my <= 110:
                    node.dragging_kd = True
                    node.kd = x_to_val(mx, 10.0)
                    handled = True
                
                if not handled:
                    # Controlla click sui pulsanti modalità
                    for idx, btn in enumerate(BUTTON_DEFS):
                        if get_button_rect(idx).collidepoint(mx, my):
                            node.robot_mode = btn['mode']
                            handled = True
                            break
                if not handled:
                    gx, gy = to_meters(mx, my)
                    if math.hypot(gx - node.obs_x, gy - node.obs_y) < node.obs_radius:
                        node.dragging_obstacle = True
                    else:
                        node.goal_x = gx
                        node.goal_y = gy
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                mx, my = pygame.mouse.get_pos()
                node.obs_x, node.obs_y = to_meters(mx, my)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                node.dragging_obstacle = False
                node.dragging_kp = False
                node.dragging_kd = False
            elif event.type == pygame.MOUSEMOTION:
                mx, my = pygame.mouse.get_pos()
                if node.dragging_obstacle:
                    node.obs_x, node.obs_y = to_meters(mx, my)
                elif node.dragging_kp:
                    node.kp = x_to_val(mx, 80.0)
                elif node.dragging_kd:
                    node.kd = x_to_val(mx, 10.0)

        # msg = Bool()
        # msg.data = node.is_running
        # node.is_running_wbc_pub.publish(msg)

        screen.fill((240, 240, 240))
        pygame.draw.line(screen, (200,200,200), (0, OFFSET_Y), (WIDTH, OFFSET_Y), 1)
        pygame.draw.line(screen, (200,200,200), (OFFSET_X, 0), (OFFSET_X, HEIGHT), 1)

        clean_opt = clean_trajectory(node.traj_optimistic)
        clean_pess = clean_trajectory(node.traj_pessimistic)

        draw_trajectory(screen, (255,100,100), clean_pess, 4, node)
        draw_trajectory(screen, (100,205,100), clean_opt, 3, node)

        ox, oy = to_pixels(node.obs_x, node.obs_y)
        r_pix = int(node.obs_radius * SCALE)
        pygame.draw.circle(screen, (100,100,100), (ox, oy), r_pix)
        pygame.draw.circle(screen, (50,50,50), (ox, oy), r_pix, 2)

        gx_px, gy_px = to_pixels(node.goal_x, node.goal_y)
        pygame.draw.circle(screen, (0,0,255), (gx_px, gy_px), 8)

        rx, ry = to_pixels(node.robot_x, node.robot_y)
        pygame.draw.circle(screen, (255,215,0), (rx, ry), 12)
        pygame.draw.circle(screen, (0,0,0), (rx, ry), 12, 2)

        screen.blit(font.render("Fisica Reale Attiva", True, (50, 50, 50)), (20, 20))
        screen.blit(font.render(f"Robot Reale (Odom): ({node.robot_x:.2f},{node.robot_y:.2f}) m", True, (0,0,0)), (20, HEIGHT-40))
        screen.blit(font.render(f"Modalità: {node.robot_mode.upper()}", True, (80,80,80)), (20, HEIGHT-65))

        draw_buttons(screen, font_big, node.robot_mode, mouse_pos)
        draw_sliders(screen, font, node, mouse_pos)

        pygame.display.flip()
        clock.tick(30)
    pygame.quit()

def main(args=None):
    rclpy.init(args=args)
    node = MpcInteractorGUI()
    
    # Esegui lo spin di ROS 2 in un thread in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # Il loop di Pygame DEVE girare sul thread principale per gestire correttamente la finestra e gli eventi grafici
        pygame_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()