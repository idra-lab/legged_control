# Optimistic–Pessimistic MPC: Mathematical Definition (OCS2 / C++ implementation)

This document is the full mathematical statement of the quadruped **Optimistic–Pessimistic
Model Predictive Control** (Opti-Pessi MPC) as implemented in `opti_pessi_interface`
(C++, OCS2, HPIPM). It has the same structure and section numbering as
[`mpc_formulation.md`](mpc_formulation.md), which states the CasADi/IPOPT reference
(`ocp_quadruped.py`), so the two can be read side by side or independently.

A term-by-term comparison against the reference, with every point of divergence
justified, lives in [`mpc_implementation.md`](mpc_implementation.md). This document does
not repeat that comparison — it states the OCS2 formulation on its own terms. A plain-language
summary of *why* the two differ is at the end.

The controller plans **variable-duration diagonal-trot footsteps** for a **3D linear
inverted pendulum (LIP)** with yaw, while avoiding a **dynamic obstacle** of bounded speed.
The model, the two-trajectory idea, and the safety argument are unchanged from the
reference; what changes is how the optimal control problem is transcribed for a
multiple-shooting solver.

---

## 1. Problem setting

A quadruped must walk from an initial planar pose to a goal $c_{\mathrm{goal}}\in\mathbb{R}^2$
while remaining collision-free with a circular obstacle. The obstacle has measured centre
$o\in\mathbb{R}^2$, radius $r_{\mathrm{obs}}$, and **unknown** planar velocity with speed at
most $v_{\mathrm{obs}}\geq 0$.

At every MPC iteration the robot replans a horizon of $N$ contact phases. Only the
**first** control is applied; the obstacle is then observed again and the problem is
resolved (receding horizon).

Opti-Pessi MPC jointly optimizes two predicted robot trajectories that share the current
state and the first input:

- **Optimistic** trajectory: minimized for performance; avoids only the **currently
  observed** obstacle disk.
- **Pessimistic** trajectory: not in the cost; must remain feasible against the
  **worst-case reachable obstacle set** (a disk that grows at rate $v_{\mathrm{obs}}$).

Safety of the applied input follows from the pessimistic plan; the optimistic plan is
what is executed in open loop until the next solve.

---

## 2. Notation and frames

| Symbol | Meaning |
| --- | --- |
| $c\in\mathbb{R}^2$ | CoM horizontal position (world) |
| $\theta\in\mathbb{R}$ | Yaw of the base |
| $\dot{c}\in\mathbb{R}^2$ | CoM horizontal velocity (world) |
| $\dot{\theta}\in\mathbb{R}$ | Yaw rate |
| $p^{(0)},p^{(1)}\in\mathbb{R}^2$ | Positions of the two **stance** feet |
| $T\in\mathbb{R}$ | elapsed-time clock (explicit state) |
| $z\in\mathbb{R}^2$ | Centre of pressure (CoP) |
| $\Delta t>0$ | Contact-phase duration (decision variable) |
| $\alpha\in(0,1)$ | CoP interpolation / normal-force split |
| $\beta,\gamma\in[0,1]$ | Tangential-force split between the two feet |
| $\phi$ | Separating-hyperplane angle; $a=(\cos\phi,\sin\phi)$ |
| $b\in\mathbb{R}$ | Separating-hyperplane offset |
| $\sigma\in[0,1]$ | Keep-out continuation scale (closed-loop safeguard, not part of the OCP model) |
| $h>0$ | Constant CoM height |
| $g$ | Gravity |
| $m$ | Mass |
| $I$ | Yaw inertia about the vertical axis |
| $\omega=\sqrt{g/h}$ | LIP natural frequency |
| $\mu$ | Friction coefficient |
| $h_{\ell}\in\mathbb{R}^2$ | Nominal hip offset of leg $\ell\in\{\mathrm{FL},\mathrm{FR},\mathrm{RL},\mathrm{RR}\}$ in the **body** frame |
| $\rho_{\mathrm{hull}}$ | Radius of the hip hull about the CoM, $\max_\ell\lVert h_\ell\rVert$ |

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

Hip location in the world frame:

$$
r_{\ell}(c,\theta) = c + R(\theta)\, h_{\ell}.
$$

$R(\theta)$ (body→world) is used everywhere a body-frame offset is placed in the world —
hip locations for collision *and* for the foothold cost (§6). $R_{01}(\theta)$
(world→body) is used everywhere a world-frame vector is expressed in the body frame —
velocity bounds (§5.2) and reachability (§5.4).

The gait is a **diagonal trot**, alternating $(\mathrm{FR},\mathrm{RL})\leftrightarrow(\mathrm{FL},\mathrm{RR})$,
starting from $(\mathrm{FR},\mathrm{RL})$ on even simulation steps.

### 2.1 Time convention

OCS2 solves a continuous-time problem; this OCP is intrinsically discrete — one knot is
one contact phase, and the phase duration is itself a decision variable. The convention
used throughout is:

- "time" passed around by OCS2 is the **knot index**, not seconds. The horizon runs $0\to N$.
- the multiple-shooting discretization uses `dt = 1.0` with the `EULER` integrator, and the
  flow map returns $f(x,u)-x$, so the Euler defect

$$
x_{i+1} = x_i + 1.0\cdot\bigl(f(x_i,u_i)-x_i\bigr) = f(x_i,u_i)
$$

  reproduces the exact discrete LIP map below.
- physical seconds live only in the decision variable $\Delta t_i$ and in the clock state $T_i$.

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

The total horizontal force implied by the LIP is $m\ddot{c}$, split between the two
stance feet by $(\beta,\gamma)$:

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
\qquad
\ddot{\theta} = \frac{\tau}{I}.
$$

During a contact phase the stance feet are **fixed** in the world.

---

## 4. Discrete state, control, and augmented OCP

### 4.1 Per-branch state and control

$$
x_i
=
\bigl[\,c_i,\ \theta_i,\ \dot c_i,\ \dot\theta_i,\ p^{(0)}_i,\ p^{(1)}_i\,\bigr]
\in\mathbb{R}^{10},
\qquad
u_i
=
\begin{bmatrix}
p_{i+1}^{(0)} \\ p_{i+1}^{(1)} \\ \alpha_i \\ \Delta t_i \\ \beta_i \\ \gamma_i
\end{bmatrix}
\in\mathbb{R}^{8}.
$$

This is the reference's state vector, entry for entry: current pose, velocity, and the two
stance feet. **No previous-phase quantities are carried.** The two constraint families
that relate knot $i$ to knot $i+1$ — reachability (§5.4) and the mid-step collision plane
(§5.5) — recompute the successor knot explicitly inside the constraint,
$x_{i+1}=f(x_i,u_i)$, exactly as the reference substitutes its dynamics.

### 4.2 Exact discretization

Let $\mathrm{ch}_i=\cosh(\omega\Delta t_i)$, $\mathrm{sh}_i=\sinh(\omega\Delta t_i)$, and
$z_i=z(p_i,\alpha_i)$. The per-branch flow map $x_{i+1}=f(x_i,u_i)$ is

$$
\begin{aligned}
c_{i+1} &= \mathrm{ch}_i\, c_i + \frac{\mathrm{sh}_i}{\omega}\,\dot{c}_i + (1-\mathrm{ch}_i)\, z_i, \\
\theta_{i+1} &= \theta_i + \Delta t_i\,\dot{\theta}_i, \\
\dot{c}_{i+1} &= \omega\,\mathrm{sh}_i\, c_i + \mathrm{ch}_i\,\dot{c}_i - \omega\,\mathrm{sh}_i\, z_i, \\
\dot{\theta}_{i+1} &= \dot{\theta}_i + \Delta t_i\,\frac{\tau_i}{I}, \\
p_{i+1} &= \bigl(p_{i+1}^{(0)},\, p_{i+1}^{(1)}\bigr)\quad\text{(foothold part of }u_i\text{)},
\end{aligned}
$$

with $\tau_i$ computed from $(c_i,p_i,\alpha_i,\beta_i,\gamma_i)$ as in §3.2. This is the
exact flow of $\ddot{c}=\omega^2(c-z)$ and a forward-Euler step for yaw, identical to the
reference's discrete map.

### 4.3 Augmented state, input, and flow map

The OCP that OCS2 actually sees stacks both branches plus an elapsed-time clock:

$$
X_i
=
\begin{bmatrix} x^{\mathrm{opti}}_i \\ x^{\mathrm{pessi}}_i \\ T_i \end{bmatrix}
\in\mathbb{R}^{21},
\qquad
U_i
=
\begin{bmatrix}
u^{\mathrm{opti}}_i \\ u^{\mathrm{pessi}}_i \\
(\phi,b)^{\mathrm{opti}}_{\mathrm{mid,land}} \\ (\phi,b)^{\mathrm{pessi}}_{\mathrm{mid,land}}
\end{bmatrix}
\in\mathbb{R}^{24}
\quad\text{(one obstacle)},
$$

$$
F(X_i,U_i)
=
\begin{bmatrix}
f\bigl(x^{\mathrm{opti}}_i,u^{\mathrm{opti}}_i\bigr) \\[2pt]
f\bigl(x^{\mathrm{pessi}}_i,u^{\mathrm{pessi}}_i\bigr) \\[2pt]
T_i + \Delta t^{\mathrm{pessi}}_i
\end{bmatrix}.
$$

The clock advances with the **pessimistic** branch's duration, matching the elapsed time
used to grow the pessimistic keep-out (§7.2). Reason for augmenting rather than solving
two separate OCS2 problems: the non-anticipativity constraint $u^{\mathrm{opti}}_0=u^{\mathrm{pessi}}_0$
(§5.8) couples the two branches through a shared decision vector, which OCS2 can only
enforce if both branches live inside one problem.

---

## 5. Constraints

Every row below is a function of $(x_i,u_i)$ on **one interval**. Where a bound belongs to
knot $i+1$, the successor is recomputed inside the row by stepping the dynamics,
$x_{i+1}=f(x_i,u_i)$ — the transcription is *composed*, not knot-local. Path and collision
rows are imposed over intervals $i=0,\dots,N-1$; interval 0 included, because its rows
constrain $x_1$, the state the applied input lands in, and never the measured $x_0$.

### 5.1 Input bounds

Active at every knot including 0:

$$
\varepsilon \le \alpha_i \le 1-\varepsilon,
\qquad
0\le \beta_i\le 1,
\qquad
0\le \gamma_i\le 1,
\qquad
\Delta t_{\min}\le \Delta t_i\le \Delta t_{\max},
$$

per branch, 8 rows.

### 5.2 Velocity bounds

On the **successor** velocity $\dot c_{i+1}$, expressed in knot $i$'s body frame — the
reference's `dc_bound(x[3:5, i+1], ..., x[2, i])`:

$$
\dot c^{\,2}_{x,\max}-\bigl(R_{01}(\theta_i)\dot c_{i+1}\bigr)^2_x\ \ge 0,
\qquad
\dot c^{\,2}_{y,\max}-\bigl(R_{01}(\theta_i)\dot c_{i+1}\bigr)^2_y\ \ge 0,
\qquad
\dot\theta^{\,2}_{\max}-\dot\theta_{i+1}^2\ \ge 0.
$$

The squared form keeps the rows smooth for the AD Jacobians.

### 5.3 Friction (circular cone, $\alpha$-split normals, rescaled)

With $f_n^{(0)}=\alpha_i\mu m g$, $f_n^{(1)}=(1-\alpha_i)\mu m g$:

$$
1-\frac{\lVert f^{(0)}\rVert^2}{f_n^{(0)2}+10^{-3}}\ \ge 0,
\qquad
1-\frac{\lVert f^{(1)}\rVert^2}{f_n^{(1)2}+10^{-3}}\ \ge 0.
$$

Dividing through by $f_n^{2}+10^{-3}$ normalizes the row to the same order of magnitude as
the box bounds, which matters for a QP that assembles every row into one matrix.

### 5.4 Foot reachability

For a foot $p$, its hip offset $h_\ell$, side sign $s_\ell=\pm1$, and
$p^{\mathrm{b}}=R_{01}(\theta_{i+1})(p-c_{i+1})$, both measured against the **successor**
base $(c_{i+1},\theta_{i+1})$:

$$
r_{\mathrm{hip}}^2-\bigl\lVert p^{\mathrm b}-h_\ell\bigr\rVert^2\ \ge 0,
\qquad
s_\ell\, p^{\mathrm b}_y\ \ge 0.
$$

Applied to four feet per branch: the two stance feet of this phase $p^{(0)}_i,p^{(1)}_i$
(pair $\mathcal{G}_i$) and the two commanded landing feet $p^{(0)}_{i+1},p^{(1)}_{i+1}$
(pair $\mathcal{G}_{i+1}$) — 8 rows per branch, exactly `reachibility_bounds(x_i, x_{i+1})`
of the reference. This family caps stride length and therefore top speed.

### 5.5 Separating-hyperplane collision avoidance

The normal is parameterized by angle, $a=(\cos\phi,\sin\phi)$, so $\lVert a\rVert_2=1$
holds identically — there is no separate norm constraint.

Two independent planes per obstacle per interval, matching the two-certificate structure
of the model:

**Mid-step plane**, hips only, at the pose halfway along interval $i$:

$$
c_{i+1/2}=\tfrac12(c_i+c_{i+1}),
\qquad
\theta_{i+1/2}=\tfrac12(\theta_i+\theta_{i+1}),
$$

$$
-(a_{\mathrm{mid}}^\top r_{\ell}(c_{i+1/2},\theta_{i+1/2})+b_{\mathrm{mid}})\ \ge 0
\quad\forall\ell,
\qquad
a_{\mathrm{mid}}^\top o+b_{\mathrm{mid}}-d_{\min,i}-10^{-3}\ \ge 0.
$$

**Landing plane**, hips and both landing feet, at the successor pose
$(c_{i+1},\theta_{i+1})$:

$$
-(a_{\mathrm{land}}^\top r_{\ell}(c_{i+1},\theta_{i+1})+b_{\mathrm{land}})\ \ge 0\quad\forall\ell,
\qquad
-(a_{\mathrm{land}}^\top p^{(k)}_{i+1}+b_{\mathrm{land}})\ \ge 0\quad k\in\{0,1\},
\qquad
a_{\mathrm{land}}^\top o+b_{\mathrm{land}}-d_{\min,i}-10^{-3}\ \ge 0.
$$

12 rows per obstacle per branch. Both planes use the same $d_{\min,i}$ (§7.2). This is the
reference's pair of `collision_avoidance` calls per interval, index for index.

### 5.6 Terminal CoP

Active for the last knot, composed with the dynamics since $x_N$ carries no input:

$$
c_N - z\bigl(p_N,\alpha_{N-1}\bigr) = 0,
\qquad
x_N=f(x_{N-1},u_{N-1}),
$$

two equality rows per branch.

### 5.7 Terminal obstacle constraint

A final-state ball-vs-ball certificate, needed because no input (and hence no $(\phi,b)$)
exists at knot $N$ to carry a hyperplane:

$$
\bigl\lVert c_N-o\bigr\rVert^2 - \bigl(r_{\mathrm{obs}}+v_{\mathrm{obs}}T_N+\rho_{\mathrm{hull}}\bigr)^2\ \ge 0,
$$

one row per branch per obstacle. Conservative relative to a hyperplane certificate, but
only at the single terminal knot.

### 5.8 First-input consensus

$$
u^{\mathrm{opti}}_0-u^{\mathrm{pessi}}_0=0,
$$

8 equality rows, active only at knot 0.

---

## 6. Running cost (optimistic trajectory only)

Let $w_c,w_{\dot{c}},w_{\theta},w_{\dot{\theta}},w_p,w_{\alpha},w_{\Delta t}\ge 0$ be
weights and $\Delta t_\star$ the preferred phase duration.

At every knot $i=0,\ldots,N$,

$$
\ell_c(x_i)=w_c\lVert c_i-c_{\mathrm{goal}}\rVert_2^2,
\qquad
\ell_{\dot c}(x_i)=w_{\dot c}\lVert\dot c_i\rVert_2^2,
\qquad
\ell_{\dot\theta}(x_i)=w_{\dot\theta}\dot\theta_i^2,
$$

$$
\ell_\theta(x_i)=w_\theta\bigl(\lVert\dot c_i\rVert_2^2\cos^2\theta_i-\dot c_{i,x}^2+\lVert\dot c_i\rVert_2^2\sin^2\theta_i-\dot c_{i,y}^2\bigr)^2
$$

(algebraically identically zero; carried at $w_\theta=0$ for fidelity with the reference).
For $i=0,\ldots,N-1$ the stage also includes

$$
\ell_\alpha(u_i)=w_\alpha(\alpha_i-\tfrac12)^2,
\qquad
\ell_{\Delta t}(u_i)=w_{\Delta t}(\Delta t_i-\Delta t_\star)^2,
$$

$$
\ell_p(x_i,u_i)=w_p\sum_{k\in\{0,1\}}\bigl\lVert p^{(k)}_{i+1}-\bigl(c_{i+1}+R(\theta_{i+1})h_{\ell^{(k)}_{i+1}}\bigr)\bigr\rVert_2^2,
\qquad
x_{i+1}=f(x_i,u_i).
$$

$\ell_p$ is written on the **successor** knot, as the reference writes it
(`cost_p(x[-4:-2, i+1], hip_pos_0)` with the hips of $\mathcal{G}_{i+1}$ taken at
$(c_{i+1},\theta_{i+1})$): the landing feet are $u_i$'s foothold entries, but the hips they
are measured against move with the base, so the successor is recomputed inside the cost.

The hip offset in $\ell_p$ is placed in the world with $R(\theta)$ (body→world) — the
same rotation used for collision (§2), so the cost pulls the landing feet toward the
*actual* nominal hip position under the predicted yaw, with no dependence on which rotation
convention is used elsewhere.

Total cost:

$$
J\bigl(x^{\mathrm{opti}},u^{\mathrm{opti}}\bigr)
=
\sum_{i=0}^{N}\Bigl(\ell_c+\ell_{\dot c}+\ell_\theta+\ell_{\dot\theta}\Bigr)_i
+
\sum_{i=0}^{N-1}\Bigl(\ell_\alpha+\ell_{\Delta t}+\ell_p\Bigr)_i.
$$

The pessimistic branch carries no cost.

---

## 7. Obstacle prediction models

Let $o$ be the currently measured obstacle centre, held fixed inside one OCP.

### 7.1 Optimistic set

$d_{\min}=r_{\mathrm{obs}}$ at every knot.

### 7.2 Pessimistic set

$$
d^{\mathrm{pessi}}_{\min,i} = r_{\mathrm{obs}} + \sigma\, v_{\mathrm{obs}}\, T_i,
\qquad
T_i=\sum_{k<i}\Delta t^{\mathrm{pessi}}_k,
$$

with $T_i$ read directly off the clock state (§4.3) rather than resummed at every row.
$\sigma\in[0,1]$ is a **closed-loop continuation scale**, not part of the mathematical
model — nominally $\sigma=1$ recovers the full worst-case guarantee; see §9.2 for when and
why it is relaxed.

$v_{\mathrm{obs}}$ is a compile-time constant for the generated cost/constraint
libraries: changing `obstacles.maxSpeed` in a scenario config requires regenerating them.

### 7.3 Scenario samples

Not implemented. Only the two-trajectory Opti-Pessi controller exists in this package.

---

## 8. Opti-Pessi OCP (the controller)

$$
\begin{aligned}
\min_{X_{0:N},\,U_{0:N-1}}\quad
& \sum_{i=0}^{N}\ell_i\bigl(x^{\mathrm{opti}}_i,u^{\mathrm{opti}}_i\bigr) \\
\text{s.t.}\quad
& X_0=\bigl[x_{\mathrm{init}},\,x_{\mathrm{init}},\,0\bigr], \\
& X_{i+1}=F(X_i,U_i), && i=0,\dots,N-1, \\
& u^{\mathrm{opti}}_0=u^{\mathrm{pessi}}_0, \\
& \text{input bounds (§5.1)}, && i=0,\dots,N-1, \\
& \text{path + collision rows (§5.2–5.5)}, && i=1,\dots,N, \\
& \text{terminal CoP (§5.6)}, \\
& \text{terminal obstacle (§5.7)}.
\end{aligned}
$$

Row counts per knot, $M$ obstacles: input bounds $2\times10$; path
$2\times(11+12M)$; consensus 8 (knot 0 only); terminal CoP 4; terminal obstacle $2M$.

The applied closed-loop input is $u_0^\star=u^{\mathrm{opti}}_0$, equal to
$u^{\mathrm{pessi}}_0$ by construction.

---

## 9. Receding-horizon MPC

### 9.1 Loop

At simulation step $n$: build the gait sequence from the step parity, solve the OCP above
with $x_{\mathrm{init}}=X_n$ and $o=O_n$, apply $u_0^\star$ for $\Delta t_0^\star$, advance
the true (unknown to the OCP) obstacle motion, stop when $t\ge T_{\mathrm{sim}}$ or the
robot reaches the goal within tolerance. Warm start shifts the **stored** trajectory one
knot and replaces knot 0 with the new measurement; it does not re-integrate, because the
LIP is exponentially unstable ($\cosh(\omega\Delta t)\approx2.96$ per knot at
$\Delta t_{\max}$) and the footholds in $u$ are absolute world positions, so re-integrating
would amplify any measurement mismatch by orders of magnitude across the horizon.

### 9.2 Closed-loop safeguards

`ocs2::IpmSolver` (HPIPM-based) has no restoration phase: on a near-infeasible problem the
linesearch can collapse to a zero step and the accepted iterate can drift. Four mechanisms
guard against this, all outside the OCP itself:

| Mechanism | Rule |
| --- | --- |
| Applied-step gate | worst constraint violation at knot 1 $<0.05$, and consistency with the dynamics $<0.02$ |
| Plan-trustworthiness gate | worst violation over the whole horizon $<0.05$; gates warm-start reuse |
| Keep-out continuation | on an untrustworthy plan, re-solve at $\sigma\in\{0,0.25,0.5,0.75,1\}$, each warm-starting the next, keep the best feasible one |
| Capture-point fallback | if every solve fails: CoP at the DCM $\xi=c+\dot c/\omega$, footholds under $\xi$, $\Delta t=\Delta t_{\min}$ |

A step whose best successful solve had $\sigma<1$ is not robust to the full
$v_{\mathrm{obs}}$ bound and is counted and reported separately, never silently accepted
as a full-guarantee step.

### 9.3 Solver

| | Setting |
| --- | --- |
| Algorithm | `ocs2::IpmSolver` — primal-dual interior point, no restoration phase |
| Linear algebra | HPIPM, `reg_prim = 1e2` (unusually high; needed because the collision rows are bilinear in $(\phi,b)$ and the pose, so the QP Hessian is routinely singular) |
| Barrier | $10^{-2}\to10^{-4}$ over `ipmIteration` outer iterations |
| Integrator | `EULER`, `dt = 1.0` (knot-index time convention, §2.1) |

---

## 10–11. Variants

Robust (classic) single-trajectory MPC and scenario-based MPC are **not implemented**.
Only the Opti-Pessi controller of §8 exists in this package.

---

## 12. Compact statement

Let $\mathcal{X}(x,u)$ denote the robot inequalities of §5.1–§5.4 plus dynamics, terminal
CoP and terminal obstacle, and let $\mathcal{C}(x,u;\mathcal{Y})$ denote the
separating-hyperplane conditions of §5.5 against obstacle set $\mathcal{Y}$. The
controller solved at each receding-horizon iteration is

$$
\begin{aligned}
\min\quad
& J(x^{\mathrm{opti}},u^{\mathrm{opti}}) \\
\mathrm{s.t.}\quad
& x^{\mathrm{opti}}_0=x^{\mathrm{pessi}}_0=x_{\mathrm{init}},
\quad
u^{\mathrm{opti}}_0=u^{\mathrm{pessi}}_0, \\
& (x^{\mathrm{opti}},u^{\mathrm{opti}})\in\mathcal{X},
\quad
(x^{\mathrm{pessi}},u^{\mathrm{pessi}})\in\mathcal{X}, \\
& (x^{\mathrm{opti}},u^{\mathrm{opti}})\in\mathcal{C}(\mathcal{Y}^{\mathrm{opti}}),
\quad
(x^{\mathrm{pessi}},u^{\mathrm{pessi}})\in\mathcal{C}(\mathcal{Y}^{\mathrm{pessi}}),
\end{aligned}
$$

on the augmented state/input pair $(X,U)$ of §4.3. Apply $u_0^\star$, shift the horizon,
repeat.

---

## 13. Configuration

| Quantity | Value | Source |
| --- | --- | --- |
| $h,g,m,I,\mu$ | $0.38\,$m, $9.81$, $24.24\,$kg, $1.048\,$kg m², $0.8$ | `task.info` |
| $\Delta t\in[\Delta t_{\min},\Delta t_{\max}]$ | $[0.20,\,0.25]\,$s | `task.info` |
| $\Delta t_\star$ (`dtCost0`) | $0.35\,$s | `task.info` |
| $\dot c_{x,\max},\dot c_{y,\max},\dot\theta_{\max}$ | $1.5,\ 0.45,\ 0.8$ | `task.info` |
| $r_{\mathrm{hip}}$ | $0.1\,$m | `task.info` |
| $\varepsilon$ (`alphaReduction`) | $0.1$ | `task.info` |
| $N$ | 6 | `task.info` |
| $w_c,w_{\dot c},w_p,w_\alpha,w_{\Delta t},w_\theta,w_{\dot\theta}$ | $1.0,\ 2.0,\ 0.5,\ 0.5,\ 10^{-2},\ 0,\ 2.0$ | `task.info` |
| $r_{\mathrm{obs}},v_{\mathrm{obs}}$ | $0.2\,$m, $1.0\,$m/s | scenario file |
| `reg_prim` | $10^2$ | `task.info` |
| `ipmIteration` | 80 | `task.info` |

Note that $\Delta t_\star=0.35\,$s currently sits **above** $\Delta t_{\max}=0.25\,$s, so
the cost pulls $\Delta t$ toward the upper bound without ever reaching the reference
value — the same qualitative effect as setting $\Delta t_\star=\Delta t_{\max}$ directly.

With $N=6$, even at $\Delta t_{\max}=0.25\,$s the pessimistic keep-out reaches
$r_{\mathrm{obs}}+v_{\mathrm{obs}}\cdot N\cdot\Delta t_{\max}\approx1.7\,$m at the final
knot; at the reference's $\Delta t_{\max}=0.35\,$s it reached $\approx2.3\,$m. The lower
bound is what keeps the far knots of the pessimistic branch away from the edge of
infeasibility in the closed-loop tests run so far (§9.2 for what happens when they are
not).

---

## Perché i due solutori sono diversi (in breve)

Il modello matematico — LIP, trotto, ramo ottimistico/pessimistico, garanzia di
robustezza — è lo stesso descritto in `mpc_formulation.md`. Le differenze qui sopra
esistono per tre ragioni distinte, ed è utile tenerle separate.

**1. La libreria impone una struttura diversa.** CasADi (`Opti`) lascia scrivere ogni
vincolo dove serve, sostituendo la dinamica al suo interno. OCS2 impone invece che ogni
costo e vincolo sia funzione dello stato e dell'ingresso a un **singolo istante**. Per
scrivere nella forma richiesta due famiglie di vincoli che nel riferimento collegano due
istanti (raggiungibilità dei piedi, collisione a metà passo), ho dovuto portare nello
stato le grandezze della fase precedente — da qui lo stato R¹⁰→R¹⁷. Per la stessa ragione
i due rami (ottimistico e pessimistico) e il tempo trascorso sono impilati in un unico
stato "aumentato" che OCS2 vede come un problema solo: è l'unico modo per far rispettare
il vincolo di consenso sul primo ingresso, che lega i due rami. Questa è una differenza di
**trascrizione**, non di modello: l'insieme delle soluzioni ammissibili resta lo stesso.

**2. Due scelte deliberate, perché ho trovato problemi nel riferimento.** L'angolo al
posto del vettore libero per la normale del piano separatore (§5.5) evita che il solver
"bari" gonfiando la norma di quel vettore per soddisfare geometricamente
l'insoddisfacibile — misurato: norma ≈95 nel riferimento al primo passo dello scenario
più difficile. La rotazione nel costo sui footholds (§6) usa `R(θ)` invece di `R(−θ)`:
con yaw diverso da zero, la versione del riferimento tira il piede verso un'anca
specchiata, non verso quella vera.

**3. Il mio solver non ha una restoration phase.** IPOPT, quando il problema diventa
temporaneamente irrisolvibile, sa "recuperare" ignorando il costo e minimizzando solo la
violazione. `ocs2::IpmSolver` (HPIPM) no: se il ramo pessimistico è vicino
all'infeasibility — cosa che succede quasi per costruzione quando il disco di sicurezza
cresce abbastanza da avvicinarsi al robot — la ricerca di linea collassa e la soluzione
può divergere invece di recuperare. Le contromisure del §9.2 (controlli sul passo
applicato, rilassamento progressivo del disco, un passo di sicurezza di riserva) esistono
solo per compensare questa assenza, e sono per questo motivo la parte più delicata
dell'intera implementazione.
