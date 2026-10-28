import casadi as cs
import numpy as np
from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver



# I HAVE AN OCP WITH 44 (11+11+11+11) * N VARIABLES but acados uses multiple shooting, thus it divides the problem into temporal nodes and creates a blocks structure for the matrices.
# This means having in each node of the horizon (up to the last) an Hessian with dimensions 44x44 and in the last node of 22x22
# 

def create_lipm_ocp(
        cfg,
        N=8,
        c_target=np.array([0.0, 0.0]),
        theta_target=0.0,
        x_init=np.zeros(22)
    ) -> AcadosOcpSolver:
 
    #### CONFIGURATION ####
    # ---- Hip offsets ---
    off_x = cfg.robot.off_x
    off_y = cfg.robot.off_y

    # ---- Weights ----
    W_diag_op = [
        cfg.ocp_weights.tracking_xy, cfg.ocp_weights.tracking_xy,       # 0,1: Tracking x, y
        cfg.ocp_weights.vel_alignment,                          # 2: Velocity alignment - penalize lateral walk
        cfg.ocp_weights.vel_xy, cfg.ocp_weights.vel_xy,                 # 3,4: velocity x, y
        cfg.ocp_weights.yaw_rate,                               # 5: Yaw rate (penalization on fast rotations)
        cfg.ocp_weights.time_weight,                            # 6: Time
        cfg.ocp_weights.anti_skating, cfg.ocp_weights.anti_skating,     # 7-8: Anti-Skating foot 0
        cfg.ocp_weights.anti_skating, cfg.ocp_weights.anti_skating,     # 9-10: Anti-Skating foot 1
        cfg.ocp_weights.posture, cfg.ocp_weights.posture,               # 11-12: Posture (keep hip above foot 0)
        cfg.ocp_weights.posture, cfg.ocp_weights.posture,               # 13-14: Posture (keep hip above foot 1)
        cfg.ocp_weights.alpha_weight,                           # 15: Alpha
        cfg.ocp_weights.dt_weight,                              # 16: dt
    ]
    # Regularization weight for the pessimistic branch to avoid null-space explosion
    W_diag_pe = [w * 1e-3 for w in W_diag_op] 
    # cost_y_expr_e_op = cs.vertcat(c_op, c_dot_op, theta_dot_op, x_op[10])
    W_end_op = [
        cfg.ocp_weights.tracking_xy, cfg.ocp_weights.tracking_xy,
        cfg.ocp_weights.vel_xy, cfg.ocp_weights.vel_xy,
        cfg.ocp_weights.yaw_rate, 
        cfg.ocp_weights.time_weight
    ]
    W_end_pe = [w * 1e-3 for w in W_end_op] 

    # dt_nominal for the reference
    dt_nominal = (cfg.limits.dt_min + cfg.limits.dt_max) / (2*cfg.simulation.step_per_phase) # We must fix a nominal value, otherwise the solver would sent it to zero to avoid errors

    # ---- Limits ----
    max_ext_sq = cfg.robot.max_ext_sq
    # (h_dist0_curr, h_dist1_curr, h_dist0_next, h_dist1_next, fric0, fric1) 
    lh_robot = [0.0, 0.0, 0.0, 0.0, -1e6, -1e6]
    uh_robot = [max_ext_sq, max_ext_sq, max_ext_sq, max_ext_sq, 0.0, 0.0]

    # (eq_norm_a, eq_s[0], eq_s[1], eq_s[2], eq_s[3], eq_p[0], eq_p[1], eq_obs) 
    # eq_s and eq_p ensure hips and feet to be in the negative semiplane (<= 0)
    lh_obs_single = [0.5, -1e6, -1e6, -1e6, -1e6, -1e6, -1e6, 0.0] 
    uh_obs_single  = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6]

    lh_obs = lh_obs_single * cfg.obstacles.N_OBS_MAX
    uh_obs = uh_obs_single * cfg.obstacles.N_OBS_MAX

    # (v_loc_x, v_loc_y) limits local velocities
    lh_vel = [-cfg.limits.v_max_x, -cfg.limits.v_max_y] 
    uh_vel = [cfg.limits.v_max_x, cfg.limits.v_max_y] 

    # ---- Constraints on state ----
    # x = [cx, cy, th, cx_dot, cy_dot, th_dot, p0x, p0y, p1x, p1y, time]
    # we only need to limit theta_dot and total time (indeces [5, 10, 16, 21])
    bnd_x = [-cfg.limits.theta_dot_max, 0.0]
    BND_X = [cfg.limits.theta_dot_max, 100.0]

    # ---- Constraints on controls ----
    # u_op = [p0_nx, p0_ny, p1_nx, p1_ny, alpha, beta, gamma, dt_var, ax, ay, b]
    dt_min = cfg.limits.dt_min / cfg.simulation.step_per_phase
    dt_max = cfg.limits.dt_max / cfg.simulation.step_per_phase

    # Limits on 4 controls (alpha, beta, gamma, dt_var)
    bnd_u_min_robot = [cfg.limits.alpha_min, 0.0, 0.0, dt_min]
    bnd_u_max_robot = [cfg.limits.alpha_max, 1.0, 1.0, dt_max]

    # Limits for single semiplane
    bnd_u_min_single_obs = [-1.0, -1.0, -100.0]
    bnd_u_max_single_obs = [1.0, 1.0, 100.0]
    
    # [alpha, beta, gamma, dt_var, (ax, ay, b)*3]
    bnd_u_min = bnd_u_min_robot + (bnd_u_min_single_obs * cfg.obstacles.N_OBS_MAX)
    bnd_u_max = bnd_u_max_robot + (bnd_u_max_single_obs * cfg.obstacles.N_OBS_MAX)

    w = np.sqrt(cfg.robot.g / cfg.robot.h_com) # Frequency constant

    #### MODEL AND OCP ####

    # Variables and parameters
    x_op = cs.SX.sym('x_op', 11) # cx, cy, theta, cdotx, cdoty, thetadot, p0x, p0y, p1x, p1y, time
    x_pe = cs.SX.sym('x_pe', 11)
    x = cs.vertcat(x_op, x_pe)

    n_u = 8 + (3 * cfg.obstacles.N_OBS_MAX)
    u_op = cs.SX.sym('u_op' ,n_u) # p0next_x, p0next_y, p1next_x, p1next_y, alpha, beta, gamma, dt, ax, ay, b
    u_pe = cs.SX.sym('u_pe', n_u)
    u = cs.vertcat(u_op, u_pe)

    n_p = 4 + (4 * cfg.obstacles.N_OBS_MAX) # 4 hips_off, (x, y, r, y_dot_max) for each obs
    p_sym = cs.SX.sym('p', n_p) # hip0_x, hip0_y, hip1_x, hip1_y, obs_x, obs_y, r_obs, y_dot_max

    hip_offset0, hip_offset1 = p_sym[0:2], p_sym[2:4]
    

    # --- HELPER FUNCTION FOR DYNAMIC ---
    def compute_dynamic(x_curr, u_curr):
        c, theta = x_curr[0:2], x_curr[2]
        c_dot, theta_dot = x_curr[3:5], x_curr[5]
        p0, p1 = x_curr[6:8], x_curr[8:10]
        t = x_curr[10]

        p0_next, p1_next = u_curr[0:2], u_curr[2:4]
        alpha = u_curr[4]
        beta = u_curr[5]
        gamma = u_curr[6]
        dt_var = u_curr[7]

        ch, sh = cs.cosh(w * dt_var), cs.sinh(w * dt_var)
        cop = p0 + alpha * (p1 - p0) # Convex combination of the feet position
        c_ddot = (w**2) * (c - cop) # CoM acceleration

        # Divide the force into tangential (F_tot) and rotationsl (f_diff)
                    # beta and gamma could only make the robot rotate only if it was already going ahead, otherwise it couldn't
        F_tot = cfg.robot.m * c_ddot
        f0_x = beta * F_tot[0]
        f1_x = (1.0 - beta) * F_tot[0]
        
        f0_y = gamma * F_tot[1]
        f1_y = (1.0 - gamma) * F_tot[1]
        
        f0 = cs.vertcat(f0_x, f0_y)
        f1 = cs.vertcat(f1_x, f1_y)

        tau_z0 = (p0[0] - c[0]) * f0[1] - (p0[1] - c[1]) * f0[0]
        tau_z1 = (p1[0] - c[0]) * f1[1] - (p1[1] - c[1]) * f1[0]
        tau_z = tau_z0 + tau_z1

        # Numerical simulation (integration)
        c_next = ch * c + (sh / w) * c_dot + (1 - ch) * cop
        c_dot_next = w * sh * c + ch * c_dot - w * sh * cop
        theta_next = theta + theta_dot * dt_var
        theta_dot_next = theta_dot + (dt_var / cfg.robot.Iz) * tau_z

        # Comulative time
        t_next = t + dt_var        

        x_next = cs.vertcat(c_next[0], c_next[1], theta_next, 
                        c_dot_next[0], c_dot_next[1], theta_dot_next, 
                        p0_next, p1_next, t_next)

        return x_next, c_next, theta_next, f0, f1

    # Compute the evolution of both worlds
    x_next_op, c_next_op, theta_next_op, f0_op, f1_op = compute_dynamic(x_op, u_op)
    x_next_pe, c_next_pe, theta_next_pe, f0_pe, f1_pe = compute_dynamic(x_pe, u_pe)

    x_next = cs.vertcat(x_next_op, x_next_pe)

    # Create the Acados Model
    model = AcadosModel()
    model.x, model.u, model.p, model.disc_dyn_expr = x, u, p_sym, x_next
    model.name = "lipm_opti_pessi" 

    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = N * 0.05 

    # --- COST FUNCTION FOR OPTIMISTIC BRANCH (Main Objective) ---
    c_op, theta_op = x_op[0:2], x_op[2]
    c_dot_op, theta_dot_op = x_op[3:5], x_op[5]
    p0_op, p1_op = x_op[6:8], x_op[8:10]
    p0_next_op, p1_next_op = u_op[0:2], u_op[2:4]

    # Rotation matrix
    R_theta_op = cs.vertcat(
        cs.horzcat(cs.cos(theta_next_op), -cs.sin(theta_next_op)),
        cs.horzcat(cs.sin(theta_next_op), cs.cos(theta_next_op))
    )

    # Hip next position in the global frame
    hip0_expr_next_op = c_next_op + R_theta_op @ hip_offset0
    hip1_expr_next_op = c_next_op + R_theta_op @ hip_offset1

    # --- Vel heading ---
    # ((vx^2+vy^2)cos(theta)^2-vx^2)^2 + ((vx^2+vy^2)sin(theta)^2-vy^2)^2
    vx_op, vy_op = c_dot_op[0], c_dot_op[1]
    # v_sq_op = vx_op**2 + vy_op**2
    # vel_err_op = (v_sq_op*(cs.cos(theta_op))**2-vx_op**2)**2+(v_sq_op*(cs.sin(theta_op))**2-vy_op**2)**2
    vel_err_op = -vx_op * cs.sin(theta_op) + vy_op * cs.cos(theta_op)


    # COSTS
    delta_p0_move_op = p0_next_op - p0_op  # anti-skating
    delta_p1_move_op = p1_next_op - p1_op
    hip_err_0_op = p0_next_op - hip0_expr_next_op # Penalize distance from the shoulder
    hip_err_1_op = p1_next_op - hip1_expr_next_op
    

    cost_y_expr_op = cs.vertcat(
        c_op, vel_err_op, c_dot_op, theta_dot_op, x_op[10],
        delta_p0_move_op, delta_p1_move_op, 
        hip_err_0_op, hip_err_1_op, 
        u_op[4] - 0.5, u_op[7] - dt_nominal
    )
    cost_y_expr_e_op = cs.vertcat(c_op, c_dot_op, theta_dot_op, x_op[10])

    # --- COST FUNCTION FOR PESSIMISTIC BRANCH (Regularization) ---
    c_pe, theta_pe = x_pe[0:2], x_pe[2]
    c_dot_pe, theta_dot_pe = x_pe[3:5], x_pe[5]
    p0_pe, p1_pe = x_pe[6:8], x_pe[8:10]
    p0_next_pe, p1_next_pe = u_pe[0:2], u_pe[2:4]

    R_theta_pe = cs.vertcat(
        cs.horzcat(cs.cos(theta_next_pe), -cs.sin(theta_next_pe)),
        cs.horzcat(cs.sin(theta_next_pe), cs.cos(theta_next_pe))
    )
    hip0_expr_next_pe = c_next_pe + R_theta_pe @ hip_offset0
    hip1_expr_next_pe = c_next_pe + R_theta_pe @ hip_offset1

    vx_pe, vy_pe = c_dot_pe[0], c_dot_pe[1]
    # v_sq_pe = vx_pe**2 + vy_pe**2
    # vel_err_pe = (v_sq_pe*(cs.cos(theta_pe))**2-vx_pe**2)**2+(v_sq_pe*(cs.sin(theta_pe))**2-vy_pe**2)**2
    vel_err_pe = -vx_pe * cs.sin(theta_pe) + vy_pe * cs.cos(theta_pe)

    delta_p0_move_pe = p0_next_pe - p0_pe  
    delta_p1_move_pe = p1_next_pe - p1_pe
    hip_err_0_pe = p0_next_pe - hip0_expr_next_pe 
    hip_err_1_pe = p1_next_pe - hip1_expr_next_pe
    

    cost_y_expr_pe = cs.vertcat(
        c_pe, vel_err_pe, c_dot_pe, theta_dot_pe, x_pe[10],
        delta_p0_move_pe, delta_p1_move_pe, 
        hip_err_0_pe, hip_err_1_pe, 
        u_pe[4] - 0.5, u_pe[7] - dt_nominal
    )
    cost_y_expr_e_pe = cs.vertcat(c_pe, c_dot_pe, theta_dot_pe, x_pe[10])

    # Combine Expressions (40 elements for intermediate, 14 for terminal)
    # Define cost function: (y-y_ref).T W (y-y_ref)
    model.cost_y_expr = cs.vertcat(cost_y_expr_op, cost_y_expr_pe)
    model.cost_y_expr_e = cs.vertcat(cost_y_expr_e_op, cost_y_expr_e_pe)

    
    
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    ocp.cost.W = np.diag(W_diag_op + W_diag_pe)
    ocp.cost.W_e = np.diag(W_end_op + W_end_pe)

    # Reference (Updated lengths)
    y_ref = np.zeros(34)
    y_ref[:2] = c_target
    y_ref[17:19] = c_target
    
    y_ref_e = np.zeros(12)
    y_ref_e[:2] = c_target
    y_ref_e[6:8] = c_target

    ocp.cost.yref, ocp.cost.yref_e = y_ref, y_ref_e

    # --- HELPER FUNCTION FOR CONSTRAINTS ---
    def compute_constraints(x_curr, u_curr, c_next, theta_next, f0, f1, p_sym_local, is_pessimistic=False):

        # Extract current states
        theta = x_curr[2]
        c_dot = x_curr[3:5]

        p0, p1 = x_curr[6:8], x_curr[8:10]
        p0_next, p1_next = u_curr[0:2], u_curr[2:4]
        a, b = u_curr[8:10], u_curr[10]

        R_th = cs.vertcat(cs.horzcat(cs.cos(theta_next), -cs.sin(theta_next)), cs.horzcat(cs.sin(theta_next), cs.cos(theta_next)))
        # Hip position of next step correspondent to next foot
        h0_next, h1_next = c_next + R_th @ hip_offset0, c_next + R_th @ hip_offset1
        # Hip position of next step correspondent to prev foot (invert y)
        hip_offset0_prec = hip_offset0
        hip_offset0_prec[1] = -hip_offset0_prec[1]
        hip_offset1_prec = hip_offset1
        hip_offset1_prec[1] = -hip_offset1_prec[1]
        h0_next_prec, h1_next_prec = c_next + R_th @ hip_offset0_prec, c_next + R_th @ hip_offset1_prec

        # Hip distance from current step
        h_dist0_curr, h_dist1_curr = cs.sumsqr(p0 - h0_next_prec), cs.sumsqr(p1 - h1_next_prec)
        # Hip distance from future step
        h_dist0_next, h_dist1_next = cs.sumsqr(p0_next - h0_next), cs.sumsqr(p1_next - h1_next)

        # Friction constraints
        fric0 = f0[0]**2 + f0[1]**2 - (cfg.robot.mu * (1.0 - u_curr[4]) * cfg.robot.m * cfg.robot.g)**2 + 1e-3
        fric1 = f1[0]**2 + f1[1]**2 - (cfg.robot.mu * u_curr[4] * cfg.robot.m * cfg.robot.g)**2 + 1e-3

        # Initialize lists for obstacle constraints
        eq_obs_all = []

        # 4 angles of the robot must stay in the negative semiplane(we  don't want the robot to be inside the obstacle)
        corners = [cs.vertcat(off_x, off_y), cs.vertcat(-off_x, off_y), cs.vertcat(-off_x, -off_y), cs.vertcat(off_x, -off_y)]

        # Separating plane Constraints
        for i in range(cfg.obstacles.N_OBS_MAX):
            # Extract a and b for obstacle i
            idx_u = 8 + (i*3)
            a = u_curr[idx_u : idx_u+2]
            b = u_curr[idx_u+2]

            # Extract parameters of obstacle i
            idx_p = 4 + (i * 4)
            y0_obs = p_sym_local[idx_p : idx_p+2]
            r_obs = p_sym_local[idx_p+2]
            y_dot_max = p_sym_local[idx_p+3]

            if is_pessimistic:
                r_dynamic = r_obs + y_dot_max * x_curr[10]
            else:
                r_dynamic = r_obs

            # 1) Norma a == 1
            eq_obs_all.append(a[0]**2 + a[1]**2)

            # 2) Robot corners in negative semiplane
            for loc in corners:
                eq_obs_all.append(cs.dot(a, c_next + R_th @ loc) + b)

            # 3) Feet in negative semiplane
            eq_obs_all.append(cs.dot(a, p0_next) + b)
            eq_obs_all.append(cs.dot(a, p1_next) + b)

            # 4) Obstacle in positive semiplane
            eq_obs_all.append(cs.dot(a, y0_obs) + b - (r_dynamic * (1 + 0.8)))

            v_loc_x = c_dot[0] * cs.cos(theta) + c_dot[1] * cs.sin(theta)
            v_loc_y = -c_dot[0] * cs.sin(theta) + c_dot[1] * cs.cos(theta)

        
        return cs.vertcat(h_dist0_curr, h_dist1_curr, h_dist0_next, h_dist1_next, fric0, fric1, *eq_obs_all, v_loc_x, v_loc_y)

    # 1. Optimistic uses fixed nominal radius
    con_op = compute_constraints(x_op, u_op, c_next_op, theta_next_op, f0_op, f1_op, p_sym)

    # 2. Pessimistic uses dynamic growing radius
    con_pe = compute_constraints(x_pe, u_pe, c_next_pe, theta_next_pe, f0_pe, f1_pe, p_sym, is_pessimistic=True)

    model.con_h_expr = cs.vertcat(con_op, con_pe)
    
    # Decouple separating planes: Only the physical 8 controls are forced equal at node 0
    u_diff = u_op[0:8] - u_pe[0:8] # 8 equations
    model.con_h_expr_0 = cs.vertcat(u_diff, con_op, con_pe)

    # Limits
    lh_branch = lh_robot + lh_obs + lh_vel
    ocp.constraints.lh = np.array(lh_branch + lh_branch)
    
    uh_branch = uh_robot + uh_obs + uh_vel
    ocp.constraints.uh = np.array(uh_branch + uh_branch)

    # Initial constraints (36 equations: 8 for u_diff == 0 + 28)
    ocp.constraints.lh_0 = np.array([0.0]*8 + lh_branch + lh_branch)
    ocp.constraints.uh_0 = np.array([0.0]*8 + uh_branch + uh_branch)

    # Constraints on States
    ocp.constraints.x0 = x_init
    ocp.constraints.lbx = np.array(bnd_x + bnd_x) 
    ocp.constraints.ubx = np.array(BND_X + BND_X)
    # theta_dot (5, 16) and time (10, 21)
    ocp.constraints.idxbx = np.array([5, 10, 16, 21]) 

    # ---- Soft Constraints (Slack Variables) -----
    # acados manage the soft constraints as SLACK VARIABLES
    # We have 16 equations to be constrained in h_con, some of them must be softened to simplify the life for the solver
    # We have the soft constraint on the velocities (3) and on the distance from current and next step to hip

    # For collision of corners (eq_s) and feet collision (eq_p)
    # 1) Hip-foot distance
    idxsh_branch = [0, 1, 2, 3]

    # 2) For each obstacle compute its index
    for i in range(cfg.obstacles.N_OBS_MAX):
        # First 6 values are lh_robot, each obs adds 8 eqns
        base_idx = 6 + (8 * i)

        # Make eq_s soft (4 indeces: base+1, base+2, base+3, base+4)
        idxsh_branch.extend([base_idx+1, base_idx+2, base_idx+3, base_idx+4])
        # Make sodt eq_p (2 indeces: base+5, base+6)
        idxsh_branch.extend([base_idx+5, base_idx+6])
        # eq_norm_a (base_idx) and eq_obs (base_idx+7) remain HARD

    # 3) Local velocities
    # After the 6 robot eqns and (8*N_max) obs eqns
    vel_idx = 6 + (8 * cfg.obstacles.N_OBS_MAX)
    idxsh_branch.extend([vel_idx, vel_idx+1])

    # Unify opt and pes
    n_h_branc = 8 + (8 * cfg.obstacles.N_OBS_MAX) # Total constraint number for each branch
    idxsh_total = idxsh_branch + [i + n_h_branc for i in idxsh_branch]
    ocp.constraints.idxsh = np.array(idxsh_total)


    # Soft constraints on non linear constraints
    # Z: quadratic penalty - z: linear penalty
    # [4x Kinematic] + [7x Collision] + [2x velocities]
    n_soft_collision = 6 * cfg.obstacles.N_OBS_MAX

    Zu_sh_branch = [1e4]*4 + [1e4]*n_soft_collision + [1e3]*2
    zu_sh_branch = [1e3]*4 + [1e3]*n_soft_collision + [1e2]*2
    Zl_sh_branch = [0.0]*4 + [0.0]*n_soft_collision + [1e3]*2
    zl_sh_branch = [0.0]*4 + [0.0]*n_soft_collision + [1e2]*2

    # These are the weights for the limits (upper and lower) of soft constraints
    # softening only the weight on theta_dot
    Zu_sbx = [1e3]*2
    zu_sbx = [1e2]*2
    Zl_sbx = [1e3]*2
    zl_sbx = [1e2]*2

    
    ocp.cost.Zu = np.array(Zu_sbx + Zu_sh_branch + Zu_sh_branch)
    ocp.cost.zu = np.array(zu_sbx + zu_sh_branch + zu_sh_branch)
    ocp.cost.Zl = np.array(Zl_sbx + Zl_sh_branch + Zl_sh_branch)
    ocp.cost.zl = np.array(zl_sbx + zl_sh_branch + zl_sh_branch)

    # Node 0 soft weights (states are fixed, only nonlinear active)
    ocp.cost.Zu_0 = np.array(Zu_sh_branch + Zu_sh_branch)
    ocp.cost.zu_0 = np.array(zu_sh_branch + zu_sh_branch)
    ocp.cost.Zl_0 = np.array(Zl_sh_branch + Zl_sh_branch)
    ocp.cost.zl_0 = np.array(zl_sh_branch + zl_sh_branch)


    # Soft constraints on state limits
    # Skip the first 8 decoupling eqns for node 0
    ocp.constraints.idxsh_0 = np.array([i + 8 for i in idxsh_total])
    # theta_dot is now index 0 for op and 2 for pe
    ocp.constraints.idxsbx = np.array([0, 2])

    

    # Constraint on Controls
    idxbu_op = list(range(4, n_u)) # Skip p0x, p0y, p1x, p1y
    idxbu_pe = [i + n_u for i in idxbu_op]


    ocp.constraints.lbu = np.array(bnd_u_min + bnd_u_min)
    ocp.constraints.ubu = np.array(bnd_u_max + bnd_u_max)
    ocp.constraints.idxbu = np.array(idxbu_op + idxbu_pe)

    ocp.parameter_values = np.zeros(n_p)

    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'DISCRETE'
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = cfg.simulation.max_iter
    # ocp.solver_options.qp_solver_warm_start = 1
    ocp.solver_options.globalization = 'MERIT_BACKTRACKING' # performs a merit function based backtracking line search following
    ocp.solver_options.levenberg_marquardt = 1e-4   # regolarization for the hessian diagonal elements
    tol = 1e-3
    ocp.solver_options.nlp_solver_tol_stat = tol   # NLP solver stationarity tolerance. Type: float > 0 Default: 1e-6
    ocp.solver_options.nlp_solver_tol_eq = tol     # NLP solver equality tolerance
    ocp.solver_options.nlp_solver_tol_ineq = tol   # NLP solver inequality tolerance
    ocp.solver_options.nlp_solver_tol_comp = tol   # NLP solver complementarity tolerance



    # Automatically export c_generated_code to cpp work space
    ocp.code_export_directory = '/home/elia/colcon_ws/src/legged_control/acados_legged_controller/c_generated_code'

    return AcadosOcpSolver(ocp, json_file='acados_ocp.json')