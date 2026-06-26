#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from legged_opti_pessi_mpc.msg import MpcOutput
import pygame
import sys
import threading
import math

# Configurazione Finestra (1 metro = 100 pixel)
SCALE = 100
WIDTH, HEIGHT = 800, 600
OFFSET_X, OFFSET_Y = 100, 300

def to_pixels(x, y):
    """Converte coordinate (m) in pixel. Ritorna (0,0) se input non valido."""
    try:
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return (OFFSET_X, OFFSET_Y)
        if not math.isfinite(x) or not math.isfinite(y):
            return (OFFSET_X, OFFSET_Y)
        return int(OFFSET_X + x * SCALE), int(OFFSET_Y - y * SCALE)
    except Exception:
        return (OFFSET_X, OFFSET_Y)

def to_meters(px, py):
    return (px - OFFSET_X) / SCALE, (OFFSET_Y - py) / SCALE

def clean_trajectory(points):
    """Restituisce solo i punti con coordinate finite e numeriche."""
    cleaned = []
    for p in points:
        try:
            if isinstance(p, (tuple, list)) and len(p) >= 2:
                x, y = p[0], p[1]
                if isinstance(x, (int, float)) and isinstance(y, (int, float)) and math.isfinite(x) and math.isfinite(y):
                    cleaned.append((float(x), float(y)))
        except Exception:
            continue
    return cleaned

def draw_trajectory(screen, color, points_world, width, node):
    """Disegna in modo sicuro una traiettoria, gestendo errori."""
    if len(points_world) < 2:
        return
    points_pix = []
    for wx, wy in points_world:
        px, py = to_pixels(wx, wy)
        # Assicuriamoci che siano due interi validi
        if isinstance(px, int) and isinstance(py, int):
            points_pix.append((px, py))
        else:
            node.get_logger().warn(f"Punto ignorato: ({wx},{wy}) -> ({px},{py})")
    if len(points_pix) >= 2:
        try:
            pygame.draw.lines(screen, color, False, points_pix, width)
        except Exception as e:
            node.get_logger().error(f"Errore disegno traiettoria: {e}")

class MpcInteractorGUI(Node):
    def __init__(self):
        super().__init__('mpc_interactor_gui')
        
        self.state_pub = self.create_publisher(Odometry, '/odom', 10)
        self.obs_pub = self.create_publisher(Point, '/obstacle_pose', 10)
        self.goal_pub = self.create_publisher(Point, '/goal_pose', 10)
        self.mpc_sub = self.create_subscription(MpcOutput, '/mpc/output', self.mpc_callback, 10)
        
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
        self.is_running = False
        
        self.create_timer(0.04, self.update_loop)

    def mpc_callback(self, msg):
        # Salva le traiettorie grezze (verranno filtrate prima del disegno)
        self.traj_optimistic = [(p.x, p.y) for p in msg.com_trajectory_optimistic]
        self.traj_pessimistic = [(p.x, p.y) for p in msg.com_trajectory_pessimistic]
        
        if self.is_running and len(self.traj_optimistic) > 1:
            x1, y1 = self.traj_optimistic[1]
            if math.isfinite(x1) and math.isfinite(y1):
                self.robot_x = x1
                self.robot_y = y1

    def update_loop(self):
        state_msg = Odometry()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'odom'
        state_msg.pose.pose.position.x = self.robot_x
        state_msg.pose.pose.position.y = self.robot_y
        state_msg.pose.pose.position.z = 0.0
        state_msg.pose.pose.orientation.w = 1.0
        self.state_pub.publish(state_msg)
        
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

def pygame_loop(node):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Opti-Pessi MPC Dashboard Interattiva")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont(None, 24)

    while rclpy.ok():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                node.is_running = not node.is_running
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                gx, gy = to_meters(mx, my)
                if event.button == 1:
                    if math.hypot(gx - node.obs_x, gy - node.obs_y) < node.obs_radius:
                        node.dragging_obstacle = True
                    else:
                        node.goal_x = gx
                        node.goal_y = gy
                elif event.button == 3:
                    node.obs_x, node.obs_y = gx, gy
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                node.dragging_obstacle = False
            elif event.type == pygame.MOUSEMOTION and node.dragging_obstacle:
                mx, my = pygame.mouse.get_pos()
                node.obs_x, node.obs_y = to_meters(mx, my)

        # --- Disegno ---
        screen.fill((240, 240, 240))
        pygame.draw.line(screen, (200,200,200), (0, OFFSET_Y), (WIDTH, OFFSET_Y), 1)
        pygame.draw.line(screen, (200,200,200), (OFFSET_X, 0), (OFFSET_X, HEIGHT), 1)

        # Traiettorie filtrate
        clean_opt = clean_trajectory(node.traj_optimistic)
        clean_pess = clean_trajectory(node.traj_pessimistic)

        draw_trajectory(screen, (255,100,100), clean_pess, 4, node)   # Rossa
        draw_trajectory(screen, (100,205,100), clean_opt, 3, node)   # Verde

        # Ostacolo
        ox, oy = to_pixels(node.obs_x, node.obs_y)
        r_pix = int(node.obs_radius * SCALE)
        pygame.draw.circle(screen, (100,100,100), (ox, oy), r_pix)
        pygame.draw.circle(screen, (50,50,50), (ox, oy), r_pix, 2)

        # Goal
        gx, gy = to_pixels(node.goal_x, node.goal_y)
        pygame.draw.circle(screen, (0,0,255), (gx, gy), 8)

        # Robot
        rx, ry = to_pixels(node.robot_x, node.robot_y)
        pygame.draw.circle(screen, (255,215,0), (rx, ry), 12)
        pygame.draw.circle(screen, (0,0,0), (rx, ry), 12, 2)

        status = "IN ESECUZIONE" if node.is_running else "IN PAUSA"
        screen.blit(font.render(f"Spazio: Play/Pausa [{status}]", True, (50,50,50)), (20,20))
        screen.blit(font.render(f"Robot: ({node.robot_x:.2f},{node.robot_y:.2f}) m", True, (0,0,0)), (20, HEIGHT-40))

        pygame.display.flip()
        clock.tick(30)

def main(args=None):
    rclpy.init(args=args)
    node = MpcInteractorGUI()
    threading.Thread(target=pygame_loop, args=(node,), daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()