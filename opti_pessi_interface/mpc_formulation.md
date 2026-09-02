# Optimistic–Pessimistic MPC: Mathematical Definition

This document is the full mathematical statement of the quadruped **Optimistic–Pessimistic Model Predictive Control** (Opti-Pessi MPC) implemented in `ocp_quadruped.py`, with the receding-horizon loop in `mpc_utils.py`. Two related controllers — **robust (classic) MPC** and **scenario-based MPC** — share the same robot model and are stated as variants.

The controller plans **variable-duration diagonal-trot footsteps** for a **3D linear inverted pendulum (LIP)** with yaw, while avoiding a **dynamic obstacle** of bounded speed.

---

## 1. Problem setting

A quadruped must walk from an initial planar pose to a goal $c_{\mathrm{goal}}\in\mathbb{R}^2$ while remaining collision-free with one or more circular obstacles. Each obstacle $j$ has measured centre $o^{(j)}\in\mathbb{R}^2$, radius $r_{\mathrm{obs}}$, and **unknown** planar velocity with speed at most $v_{\mathrm{obs}}\geq 0$.

At every MPC iteration the robot replans a horizon of $N$ contact phases. Only the **first** control is applied; the obstacle is then observed again and the problem is resolved (receding horizon).

Opti-Pessi MPC jointly optimizes two predicted robot trajectories that share the current state and the first input:

- **Optimistic** trajectory: minimized for performance; avoids only the **currently observed** obstacle disks.
- **Pessimistic** trajectory: not in the cost; must remain feasible against the **worst-case reachable obstacle set** (a disk that grows at rate $v_{\mathrm{obs}}$).

Safety of the applied input follows from the pessimistic plan; the optimistic plan is what is executed in open loop until the next solve.

---

## 2. Notation and frames

| Symbol | Meaning |
| --- | --- |
| $c\in\mathbb{R}^2$ | CoM horizontal position (world) |
| $\theta\in\mathbb{R}$ | Yaw of the base |
| $\dot{c}\in\mathbb{R}^2$ | CoM horizontal velocity (world) |
| $\dot{\theta}\in\mathbb{R}$ | Yaw rate |
| $p^{(0)},p^{(1)}\in\mathbb{R}^2$ | Positions of the two **stance** feet |
| $z\in\mathbb{R}^2$ | Centre of pressure (CoP) |
| $\Delta t>0$ | Contact-phase duration (decision variable) |
| $\alpha\in(0,1)$ | CoP interpolation / normal-force split |
| $\beta,\gamma\in[0,1]$ | Tangential-force split between the two feet |
| $h>0$ | Constant CoM height |
| $g$ | Gravity |
| $m$ | Mass |
| $I$ | Yaw inertia about the vertical axis |
| $\omega=\sqrt{g/h}$ | LIP natural frequency |
| $\mu$ | Friction coefficient |
| $h_{\ell}\in\mathbb{R}^2$ | Nominal hip offset of leg $\ell\in\{\mathrm{FL},\mathrm{FR},\mathrm{RL},\mathrm{RR}\}$ in the **body** frame |

Body-to-world rotation and world-to-body rotation:

$$
R(\theta)
=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix},
\qquad
R_{01}(\theta)
=
R(-\theta)
=
\begin{bmatrix}
\cos\theta & \sin\theta \\
-\sin\theta & \cos\theta
\end{bmatrix}.
$$

Hip location in the world frame (used in collision constraints):

$$
r_{\ell}(c,\theta) = c + R(\theta)\, h_{\ell}.
$$

The gait is a **diagonal trot**. At contact phase $i$ the stance pair $\mathcal{G}_i=(\ell_i^{(0)},\ell_i^{(1)})$ alternates:

$$
(\mathrm{FR},\mathrm{RL})
\;\longleftrightarrow\;
(\mathrm{FL},\mathrm{RR}).
$$

Which pair is active at the beginning of the horizon depends on the parity of the simulation step (even: start with $\mathrm{FR},\mathrm{RL}$).

---

## 3. Continuous-time robot model

### 3.1 Linear inverted pendulum

The CoM height is locked at $h$. Horizontal CoM dynamics with a **constant** CoP $z$ are

$$
\ddot{c} = \omega^2\,(c-z).
$$

The unique CoP on the support segment $[p^{(0)},p^{(1)}]$ is parameterized by $\alpha$:

$$
z(p,\alpha) = p^{(0)}+\alpha\bigl(p^{(1)}-p^{(0)}\bigr).
$$

### 3.2 Tangential contact forces and yaw

The total horizontal force implied by the LIP is $m\ddot{c}$. It is split between the two stance feet by $(\beta,\gamma)$:

$$
\begin{aligned}
f_x^{(0)} &= \beta\, m\,\ddot{c}_x, &
f_y^{(0)} &= \gamma\, m\,\ddot{c}_y, \\
f_x^{(1)} &= (1-\beta)\, m\,\ddot{c}_x, &
f_y^{(1)} &= (1-\gamma)\, m\,\ddot{c}_y.
\end{aligned}
$$

The vertical (yaw) moment about the CoM is

$$
\tau
=
\sum_{k\in\{0,1\}}
\Bigl(
(p_x^{(k)}-c_x)\, f_y^{(k)}
-
(p_y^{(k)}-c_y)\, f_x^{(k)}
\Bigr),
$$

and

$$
\ddot{\theta} = \frac{\tau}{I}.
$$

Yaw kinematics: $\dot{\theta}$ is the yaw rate (no additional dynamics other than the Euler step below).

During a contact phase the stance feet are **fixed** in the world ($\dot{p}^{(k)}=0$). A new pair of footholds is chosen at the next phase.

---

## 4. Discrete state, control, and exact LIP step

Index $i=0,\ldots,N$ labels the knots of the horizon. One knot equals one contact phase.

### 4.1 State and control

$$
x_i
=
\begin{bmatrix}
c_i \\ \theta_i \\ \dot{c}_i \\ \dot{\theta}_i \\ p_i^{(0)} \\ p_i^{(1)}
\end{bmatrix}
\in\mathbb{R}^{10},
\qquad
u_i
=
\begin{bmatrix}
p_{i+1}^{(0)} \\ p_{i+1}^{(1)} \\ \alpha_i \\ \Delta t_i \\ \beta_i \\ \gamma_i
\end{bmatrix}
\in\mathbb{R}^{8}.
$$

The first four entries of $u_i$ are the **next** footholds. The current footholds live in $x_i$. CoP, friction, and yaw torque at step $i$ are evaluated with the **current** feet $p_i=(p_i^{(0)},p_i^{(1)})$ stored in $x_i$.

### 4.2 Exact discretization (constant CoP over $\Delta t_i$)

Let $\mathrm{ch}_i=\cosh(\omega\Delta t_i)$, $\mathrm{sh}_i=\sinh(\omega\Delta t_i)$, and $z_i=z(p_i,\alpha_i)$. The map $x_{i+1}=f(x_i,u_i)$ is

$$
\begin{aligned}
c_{i+1}
&=
\mathrm{ch}_i\, c_i
+
\frac{\mathrm{sh}_i}{\omega}\,\dot{c}_i
+
(1-\mathrm{ch}_i)\, z_i, \\
\theta_{i+1}
&=
\theta_i + \Delta t_i\,\dot{\theta}_i, \\
\dot{c}_{i+1}
&=
\omega\,\mathrm{sh}_i\, c_i
+
\mathrm{ch}_i\,\dot{c}_i
-
\omega\,\mathrm{sh}_i\, z_i, \\
\dot{\theta}_{i+1}
&=
\dot{\theta}_i + \Delta t_i\,\frac{\tau_i}{I}, \\
p_{i+1}
&=
\bigl(p_{i+1}^{(0)},\, p_{i+1}^{(1)}\bigr)
\quad\text{(foothold part of }u_i\text{)}.
\end{aligned}
$$

Here $\tau_i$ is computed from $(c_i,p_i,\alpha_i,\beta_i,\gamma_i)$ as in §3.2, with $\ddot{c}_i=\omega^2(c_i-z_i)$.

This is the exact flow of $\ddot{c}=\omega^2(c-z)$ and a forward-Euler step for yaw.

---

## 5. Constraints (both trajectories)

All of the following are imposed on **every** predicted robot trajectory (optimistic, pessimistic, robust, and each scenario), for $i=0,\ldots,N-1$, unless noted.

### 5.1 Input bounds

$$
\varepsilon \le \alpha_i \le 1-\varepsilon,
\qquad
0\le \beta_i\le 1,
\qquad
0\le \gamma_i\le 1,
\qquad
\Delta t_{\min}\le \Delta t_i\le \Delta t_{\max}.
$$

The margin $\varepsilon>0$ (code: `alpha_reduction`) keeps the CoP strictly inside the support segment so that both feet keep a positive normal load.

### 5.2 Velocity bounds

CoM velocity at knot $i+1$ is limited in the **body** frame of knot $i$:

$$
\bigl\lvert R_{01}(\theta_i)\,\dot{c}_{i+1}\bigr\rvert_x
\le
\dot{c}_{x,\max},
\qquad
\bigl\lvert R_{01}(\theta_i)\,\dot{c}_{i+1}\bigr\rvert_y
\le
\dot{c}_{y,\max},
\qquad
\bigl\lvert \dot{\theta}_{i+1}\bigr\rvert
\le
\dot{\theta}_{\max}.
$$

### 5.3 Friction (circular cone, $\alpha$-split normals)

Normal forces are taken as $f_n^{(0)}=\alpha_i\, mg$ and $f_n^{(1)}=(1-\alpha_i)\, mg$. With a small numerical slack $\epsilon_f=10^{-3}$,

$$
\begin{aligned}
\bigl(f_x^{(0)}\bigr)^2+\bigl(f_y^{(0)}\bigr)^2
&\le
(\alpha_i\,\mu\, mg)^2+\epsilon_f, \\
\bigl(f_x^{(1)}\bigr)^2+\bigl(f_y^{(1)}\bigr)^2
&\le
\bigl((1-\alpha_i)\,\mu\, mg\bigr)^2+\epsilon_f.
\end{aligned}
$$

### 5.4 Foot reachability (body frame of the **next** pose)

Let $p^{\mathrm{b}}(p,c,\theta)=R_{01}(\theta)\,(p-c)$ be a world point expressed in the body frame at $(c,\theta)$. For each stance foot $k\in\{0,1\}$ and each of the two poses $\{p_i,\,p_{i+1}\}$ relative to the **next** base $(c_{i+1},\theta_{i+1})$:

$$
\bigl\lVert
p^{\mathrm{b}}\bigl(p^{(k)},c_{i+1},\theta_{i+1}\bigr)
-
h_{\ell}
\bigr\rVert_2
\le
r_{\mathrm{hip}},
$$

where $\ell$ is the corresponding entry of $\mathcal{G}_i$ (current feet) or $\mathcal{G}_{i+1}$ (next feet), and $r_{\mathrm{hip}}$ is `foot_hip_max`.

Left/right self-collision of the support polygon is forbidden in that same body frame:

$$
p^{\mathrm{b}}_y > 0
\quad\text{if }\ell\in\{\mathrm{FL},\mathrm{RL}\},
\qquad
p^{\mathrm{b}}_y < 0
\quad\text{if }\ell\in\{\mathrm{FR},\mathrm{RR}\}.
$$

Thus current feet must remain reachable **after** the CoM moves, and the newly chosen footholds must be reachable at landing.

### 5.5 Separating-hyperplane collision avoidance

For a predicted robot pose $(c,\theta)$, optional feet $p$, and an obstacle centre $o\in\mathbb{R}^2$ with clearance $d_{\min}$, there must exist a unit vector $a\in\mathbb{R}^2$ and offset $b\in\mathbb{R}$ such that the robot (hips, and feet when required) lies in the half-space $a^\top y+b\le 0$ and the obstacle disk of radius $d_{\min}$ lies in $a^\top y+b\ge d_{\min}$:

$$
\begin{aligned}
a^\top r_{\ell}(c,\theta)+b &\le 0
&& \forall \ell\in\{\mathrm{FL},\mathrm{FR},\mathrm{RL},\mathrm{RR}\}, \\
a^\top p^{(k)}+b &\le 0
&& \text{(if feet are included), } k\in\{0,1\}, \\
a^\top o+b &\ge d_{\min}+\epsilon_c, \\
\lVert a\rVert_2 &= 1,
\end{aligned}
$$

with $\epsilon_c=10^{-3}$. This is a standard dual certificate that $\mathrm{conv}\{\text{hips (+ feet)}\}$ and the disk of radius $d_{\min}$ about $o$ are disjoint.

On each interval $i$, the constraint is imposed **twice** per obstacle, with independent $(a,b)$:

1. **Mid-step**, hips only: $c_{i+1/2}=(c_i+c_{i+1})/2$, $\theta_{i+1/2}=(\theta_i+\theta_{i+1})/2$.
2. **Landing**, hips and next feet $p_{i+1}$: pose $(c_{i+1},\theta_{i+1})$.

### 5.6 Initial and terminal conditions

$$
x_0 = x_{\mathrm{init}},
\qquad
c_N = z(p_N,\alpha_{N-1}).
$$

The terminal constraint places the final CoM on the last CoP (static LIP equilibrium in position). Terminal velocity is **not** forced to zero.

---

## 6. Running cost (optimistic trajectory only)

Let $w_c,w_{\dot{c}},w_{\theta},w_{\dot{\theta}},w_p,w_{\alpha},w_{\Delta t}\ge 0$ be weights and $\Delta t_\star$ the preferred phase duration (`dt_cost0`).

At every knot $i=0,\ldots,N$,

$$
\begin{aligned}
\ell_c(x_i)
&=
w_c\,\lVert c_i-c_{\mathrm{goal}}\rVert_2^2, \\
\ell_{\dot{c}}(x_i)
&=
w_{\dot{c}}\,\lVert \dot{c}_i\rVert_2^2, \\
\ell_{\dot{\theta}}(x_i)
&=
w_{\dot{\theta}}\,\dot{\theta}_i^2, \\
\ell_{\theta}(x_i)
&=
w_{\theta}\,
\Bigl(
\lVert \dot{c}_i\rVert_2^2\cos^2\theta_i
-
\dot{c}_{i,x}^2
+
\lVert \dot{c}_i\rVert_2^2\sin^2\theta_i
-
\dot{c}_{i,y}^2
\Bigr)^2.
\end{aligned}
$$

The last expression is algebraically identically zero; in the Aliengo configurations $w_{\theta}=0$. It is the implemented heading-alignment term (`cost_theta`).

For $i=0,\ldots,N-1$ the stage also includes

$$
\begin{aligned}
\ell_{\alpha}(u_i)
&=
w_{\alpha}\,(\alpha_i-\tfrac12)^2, \\
\ell_{\Delta t}(u_i)
&=
w_{\Delta t}\,(\Delta t_i-\Delta t_\star)^2, \\
\ell_{p}(x_{i+1})
&=
w_p\sum_{k\in\{0,1\}}
\bigl\lVert
p_{i+1}^{(k)}
-
\bigl(
c_{i+1}+R_{01}(\theta_{i+1})\, h_{\ell_{i+1}^{(k)}}
\bigr)
\bigr\rVert_2^2.
\end{aligned}
$$

(The hip used in $\ell_p$ is rotated with $R_{01}=R(-\theta)$, matching `hip_pos_01`; collision uses $R(\theta)$.)

The finite-horizon cost of a trajectory $(x,u)$ is

$$
J(x,u)
=
\sum_{i=0}^{N}
\Bigl(
\ell_c(x_i)+\ell_{\dot{c}}(x_i)+\ell_{\theta}(x_i)+\ell_{\dot{\theta}}(x_i)
\Bigr)
+
\sum_{i=0}^{N-1}
\Bigl(
\ell_{\alpha}(u_i)+\ell_{\Delta t}(u_i)+\ell_{p}(x_{i+1})
\Bigr).
$$

---

## 7. Obstacle prediction models

Let $o^{(j)}$ be the **currently measured** centre of obstacle $j$ (held fixed inside one OCP). Write $\mathcal{B}(o,r)=\{y:\lVert y-o\rVert_2\le r\}$.

### 7.1 Optimistic set (frozen obstacle)

$$
\mathcal{Y}^{\mathrm{opti}}_i
=
\mathcal{B}\bigl(o^{(j)},\, r_{\mathrm{obs}}\bigr)
\qquad\text{for all }i.
$$

Collision constraints of the optimistic trajectory use $d_{\min}=r_{\mathrm{obs}}$ at every knot.

### 7.2 Pessimistic / robust set (maximum-speed ball)

Any trajectory of an obstacle with $\lVert \dot{o}\rVert_2\le v_{\mathrm{obs}}$ starting at $o^{(j)}$ remains, after elapsed time $T_i=\sum_{k=0}^{i}\Delta t_k$, inside the disk of radius $r_{\mathrm{obs}}+v_{\mathrm{obs}} T_i$. The implementation grows the radius with the **pessimistic** durations:

$$
d_{\min,i}^{\mathrm{pessi}}
=
r_{\mathrm{obs}}
+
v_{\mathrm{obs}}\sum_{k=0}^{i}\Delta t_k^{\mathrm{pessi}},
\qquad
\mathcal{Y}^{\mathrm{pessi}}_i
=
\mathcal{B}\bigl(o^{(j)},\, d_{\min,i}^{\mathrm{pessi}}\bigr).
$$

The same $d_{\min,i}^{\mathrm{pessi}}$ is used for both the mid-step and the landing constraint of interval $i$. This is the set plotted as $\mathcal{Y}_i$ in the code.

### 7.3 Scenario samples

Scenario $s=1,\ldots,S$ draws a direction $\hat{v}_s$ uniformly on the circle (normalized Gaussian in $\mathbb{R}^2$) and a speed $v_s\sim\mathrm{Unif}[0,v_{\mathrm{obs}}]$, then integrates

$$
o^{s}_{0}=o,
\qquad
o^{s}_{i+1}
=
o^{s}_{i}
+
\Delta t_i^{s}\, v_s\hat{v}_s,
\qquad
d_{\min}=r_{\mathrm{obs}}.
$$

(The sampled velocity is drawn once per interval in the current code.)

---

## 8. Opti-Pessi OCP (the controller)

**Decision variables.** Two full trajectories

$$
\bigl(x^{\mathrm{opti}}_{0:N},\, u^{\mathrm{opti}}_{0:N-1}\bigr),
\qquad
\bigl(x^{\mathrm{pessi}}_{0:N},\, u^{\mathrm{pessi}}_{0:N-1}\bigr),
$$

plus separating-hyperplane parameters $(a,b)$ at mid-step and landing, for every obstacle and every interval, on each trajectory.

$$
\begin{aligned}
\min_{x^{\mathrm{opti}},u^{\mathrm{opti}},\, x^{\mathrm{pessi}},u^{\mathrm{pessi}}}
\quad
&
J\bigl(x^{\mathrm{opti}},u^{\mathrm{opti}}\bigr) \\
\text{subject to}
\quad
&
x^{\mathrm{opti}}_0 = x_{\mathrm{init}},
\quad
x^{\mathrm{pessi}}_0 = x_{\mathrm{init}}, \\
&
u^{\mathrm{opti}}_0 = u^{\mathrm{pessi}}_0, \\
&
x^{\bullet}_{i+1}=f(x^{\bullet}_i,u^{\bullet}_i)
\quad
\bullet\in\{\mathrm{opti},\mathrm{pessi}\},\;
i=0,\ldots,N-1, \\
&
\text{bounds, friction, reachability on both trajectories (sec. 5),} \\
&
\text{collision: }
x^{\mathrm{opti}}\text{ vs }\mathcal{Y}^{\mathrm{opti}}_i,\;
x^{\mathrm{pessi}}\text{ vs }\mathcal{Y}^{\mathrm{pessi}}_i, \\
&
c^{\mathrm{opti}}_N = z\bigl(p^{\mathrm{opti}}_N,\alpha^{\mathrm{opti}}_{N-1}\bigr), \\
&
c^{\mathrm{pessi}}_N = z\bigl(p^{\mathrm{pessi}}_N,\alpha^{\mathrm{pessi}}_{N-1}\bigr).
\end{aligned}
$$

**Interpretation.**

| Mechanism | Role |
| --- | --- |
| Cost on optimistic only | Performance as if the obstacle did not close in |
| Pessimistic vs $\mathcal{Y}^{\mathrm{pessi}}$ | Robust feasibility against any $\lVert \dot{o}\rVert_2\le v_{\mathrm{obs}}$ |
| $u^{\mathrm{opti}}_0=u^{\mathrm{pessi}}_0$ | Non-anticipativity: the applied input is safe for the pessimistic branch |
| $x^{\mathrm{opti}}_0=x^{\mathrm{pessi}}_0=x_{\mathrm{init}}$ | Both plans start from the measured robot state |

The applied closed-loop input is $u_0^\star=u^{\mathrm{opti}}_0$ (equal to $u^{\mathrm{pessi}}_0$). The successor state used to warm-start the next OCP is the first optimistic knot $x^{\mathrm{opti}}_1$.

---

## 9. Receding-horizon MPC

At simulation step $n$, with measured robot state $X_n$ and obstacle centres $O_n$:

1. Build the gait sequence $\mathcal{G}_{0:N}$ of length $N+1$, starting from $(\mathrm{FR},\mathrm{RL})$ if $n$ is even and from $(\mathrm{FL},\mathrm{RR})$ if $n$ is odd.
2. Solve the Opti-Pessi OCP with $x_{\mathrm{init}}=X_n$ and $o^{(j)}=O_n^{(j)}$, warm-started from the previous solution shifted by one phase.
3. Apply $u_0^\star$ for duration $\Delta t_0^\star$. The plant successor is $X_{n+1}=x^{\mathrm{opti}}_1$ when the NLP succeeds; otherwise the first input is saturated to bounds and the discrete dynamics of §4.2 are integrated.
4. Advance each obstacle with its **true** (unknown to the OCP) motion law for time $\Delta t_0^\star$: straight, patrol, circular about the goal, or antagonist (velocity toward the robot).
5. Stop when $t\ge T_{\mathrm{sim}}$ or $\lVert c_n-c_{\mathrm{goal}}\rVert_2\le \delta$.

The NLP is solved with IPOPT through CasADi `Opti`.

---

## 10. Variant: robust (classic) MPC

A **single** trajectory $(x,u)$. Same robot constraints and cost $J(x,u)$. Collision uses only the **pessimistic** growing disk

$$
d_{\min,i}
=
r_{\mathrm{obs}}
+
v_{\mathrm{obs}}\sum_{k=0}^{i}\Delta t_k,
$$

i.e. the robot must avoid $\mathcal{Y}^{\mathrm{pessi}}_i$ along the trajectory that is also used for performance. This is a standard tube / reachable-set robustification of the obstacle, and is typically more conservative than Opti-Pessi.

If a fixed schedule `dt_pattern` is provided, $\Delta t_i$ is not a decision variable.

---

## 11. Variant: scenario-based MPC

$S$ trajectories $(x^{s},u^{s})_{s=1}^{S}$ (code: `N_scenarios`). Cost is the **sum** of $J(x^{s},u^{s})$ over scenarios. Shared first input:

$$
u^{s}_0 = u^{1}_0
\qquad s=2,\ldots,S.
$$

Each scenario has its own sampled obstacle path (§7.3) and uses $d_{\min}=r_{\mathrm{obs}}$. Robot constraints of §5 hold per scenario. Terminal CoP constraints hold per scenario.

---

## 12. Compact Opti-Pessi statement

Let $\mathcal{X}(x,u)$ denote the robot inequalities of §5.1–§5.4 plus dynamics and terminal CoP, and let $\mathcal{C}(x,u;\mathcal{Y})$ denote the separating-hyperplane conditions of §5.5 against obstacle sets $\mathcal{Y}$. The controller solved at each receding-horizon iteration is

$$
\begin{aligned}
\min\quad
&
J(x^{\mathrm{opti}},u^{\mathrm{opti}}) \\
\mathrm{s.t.}\quad
&
x^{\mathrm{opti}}_0=x^{\mathrm{pessi}}_0=x_{\mathrm{init}},
\quad
u^{\mathrm{opti}}_0=u^{\mathrm{pessi}}_0, \\
&
(x^{\mathrm{opti}},u^{\mathrm{opti}})\in\mathcal{X},
\quad
(x^{\mathrm{pessi}},u^{\mathrm{pessi}})\in\mathcal{X}, \\
&
(x^{\mathrm{opti}},u^{\mathrm{opti}})\in\mathcal{C}(\mathcal{Y}^{\mathrm{opti}}),
\quad
(x^{\mathrm{pessi}},u^{\mathrm{pessi}})\in\mathcal{C}(\mathcal{Y}^{\mathrm{pessi}}).
\end{aligned}
$$

Apply $u_0^\star$, shift the horizon, repeat.

---

## 13. Conceptual ancestors (point-mass toys)

The same optimistic–pessimistic split appears in `resources/opti_pessi_single_int.py` and `resources/opti_pessi_double_int.py` with integrator dynamics $\dot{c}=u$ or $\ddot{c}=u$, cost on the optimistic trajectory only, pessimistic collision $\lVert c^{\mathrm{safe}}_{i+1}-o\rVert_2\ge (i+1)\,\Delta t\, v_{\mathrm{obs}}$, and $u^{\mathrm{opti}}_0=u^{\mathrm{safe}}_0$. The quadruped OCP is that idea lifted to LIP stepping.

The LIP / CoP / foothold discretization without obstacles is the SRB-footstep OCP in `resources/srb_footstep_ocp.py`.
