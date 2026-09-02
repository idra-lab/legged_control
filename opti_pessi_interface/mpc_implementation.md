# Opti-Pessi MPC: the OCS2 Implementation, Mathematically

This document states the optimal control problem **as actually implemented** in
`opti_pessi_interface` (C++ / OCS2 / HPIPM), and compares it term by term with
[`mpc_formulation.md`](mpc_formulation.md), which states the CasADi/IPOPT reference
(`ocp_quadruped.py`).

Section numbers mirror `mpc_formulation.md` so the two can be read side by side.
§14 collects every divergence in one table.

Throughout, **R** marks the reference formulation and **I** marks this implementation.

---

## 0. What is structurally different, in one paragraph

R writes its constraints on knot $i+1$, obtained by substituting the dynamics
$x_{i+1}=f(x_i,u_i)$ into each constraint. I writes the *same feasible set* on knot $i$
and imposes it over knots $1..N$, exploiting the fact that multiple shooting already
carries $x_{i+1}$ as a decision variable. I also carries the previous stance feet, the
previous pose, and an elapsed-time clock inside the state so that every row is a function
of one knot only, and
parameterises the separating hyperplane by angle instead of by a free vector with a
norm equality. These are transcription choices, not model changes — but they are the
reason the two solve differently.

---

## 1. Problem setting

Identical to R §1. Two trajectories, cost on the optimistic one, shared first input,
worst-case reachable disk for the pessimistic one.

---

## 2. Notation

R's notation is adopted unchanged. Additional symbols used only by I:

| Symbol | Meaning | Code |
| --- | --- | --- |
| $p^{-,(0)},p^{-,(1)}\in\mathbb{R}^2$ | stance feet of the **previous** contact phase | `RobotX::PP0*`, `PP1*` |
| $c^-\in\mathbb{R}^2,\ \theta^-\in\mathbb{R}$ | CoM and yaw of the **previous** contact phase | `RobotX::PCX/PCY/PTH` |
| $T_i\in\mathbb{R}$ | elapsed-time clock, an explicit state | `CLOCK_INDEX` |
| $\phi$ | separating-hyperplane **angle** | `Hyperplane::MID_PHI`, `LAND_PHI` |
| $\sigma\in[0,1]$ | keep-out continuation scale | `pessiScale` |
| $\rho_{\mathrm{hull}}$ | radius of the hip hull about the CoM | `hullRadius_` |

Rotations, hip offsets $h_\ell$, and the diagonal-trot pair $\mathcal{G}_i$ are as in R §2.
`gaitPair` returns $(\mathrm{FR},\mathrm{RL})$ for even parity and $(\mathrm{FL},\mathrm{RR})$
for odd, matching R.

### 2.1 Time convention (I only)

OCS2 solves a continuous-time problem; this OCP is intrinsically discrete. I therefore
uses **knot index as OCS2 "time"**: the horizon runs $0\to N$, the IPM discretisation is
`dt = 1.0` with `EULER`, and the flow map returns $f(x,u)-x$ so the Euler defect

$$
x_{i+1} = x_i + 1.0\cdot\bigl(f(x_i,u_i)-x_i\bigr) = f(x_i,u_i)
$$

reproduces R's exact discrete map. Physical seconds live only in the decision variable
$\Delta t_i$ and in the clock $T_i$. (`LipKinematics.h`, "TIME CONVENTION".)

---

## 3. Continuous-time model

Identical to R §3. `computeCop`, `computeTangentialForces`, `yawTorque` implement
$z=p^{(0)}+\alpha(p^{(1)}-p^{(0)})$, the $(\beta,\gamma)$ force split, and

$$
\tau=\sum_{k}\bigl((p^{(k)}_x-c_x)f^{(k)}_y-(p^{(k)}_y-c_y)f^{(k)}_x\bigr)
$$

verbatim.

---

## 4. Discrete state, control, exact step

### 4.1 Per-branch state and control — **differs from R**

$$
x_i
=
\bigl[\,c_i,\ \theta_i,\ \dot c_i,\ \dot\theta_i,\ p^{(0)}_i,\ p^{(1)}_i,\
\underbrace{p^{-,(0)}_i,\ p^{-,(1)}_i,\ c^-_i,\ \theta^-_i}_{\text{new in I}}\,\bigr]
\in\mathbb{R}^{17},
\qquad
u_i\in\mathbb{R}^{8}\ \text{(as in R)}.
$$

The seven extra entries are the previous phase's stance feet and pose. In the exact step
they are all **pure copies** — $p^{-}_{i+1}:=p_i$, $c^-_{i+1}:=c_i$,
$\theta^-_{i+1}:=\theta_i$ — so they add no coupling and no ill-conditioning. They exist so
that two constraint families R writes across a pair of knots can each be written at a
single knot: reachability (§5.4) and the mid-step collision plane (§5.5).

### 4.2 Augmented state and input (I only)

The OCP that OCS2 sees stacks both branches plus the clock:

$$
X_i
=
\begin{bmatrix} x^{\mathrm{opti}}_i \\ x^{\mathrm{pessi}}_i \\ T_i \end{bmatrix}
\in\mathbb{R}^{35},
\qquad
U_i
=
\begin{bmatrix}
u^{\mathrm{opti}}_i \\ u^{\mathrm{pessi}}_i \\
(\phi,b)^{\mathrm{opti}}_{1:M} \\ (\phi,b)^{\mathrm{pessi}}_{1:M}
\end{bmatrix}
\in\mathbb{R}^{16+8M},
$$

with $M$ obstacles, each contributing a $(\phi,b)$ pair for the mid-step plane and another
for the landing plane, per branch. For $M=1$: $\dim X=35$, $\dim U=24$ (as printed at
start-up).

### 4.3 Augmented flow map

$$
F(X_i,U_i)
=
\begin{bmatrix}
f\bigl(x^{\mathrm{opti}}_i,u^{\mathrm{opti}}_i\bigr) \\[2pt]
f\bigl(x^{\mathrm{pessi}}_i,u^{\mathrm{pessi}}_i\bigr) \\[2pt]
T_i + \Delta t^{\mathrm{pessi}}_i
\end{bmatrix},
$$

with $f$ exactly R §4.2. The clock advances with the **pessimistic** duration, matching
R §7.2's $\sum_k \Delta t^{\mathrm{pessi}}_k$. Measured in a live run the clock reaches
$T_N \approx 1.5\,\mathrm{s}$ for $N=6$.

---

## 5. Constraints

### 5.0 Knot-local transcription — **the central difference**

Every row below is a function of $(x_i,u_i)$ at **one** knot. Nothing is composed with the
dynamics, except the two terminal constraints (§5.6, §5.7). Rows that R writes on knot
$i+1$ for $i=0..N-1$ are written here on knot $i$ and imposed over $i=1..N$ via

```
StageInequalityConstraint::isActive(time) == (time >= 0.5)
```

Both express the same feasible set, because the shooting defect forces
$x_{i+1}=f(x_i,u_i)$ at any solution. They are very different *optimisation problems*:
composing with the dynamics pushes $\cosh/\sinh$ of the decision variable $\Delta t$,
products with $\alpha$, and absolute world position through every Jacobian the QP sees.

Knot 0 carries no path row in either formulation: it is pinned to the measurement, and
constraining it would make the OCP infeasible exactly when the controller is needed most.

### 5.1 Input bounds — equivalent

$\varepsilon\le\alpha\le1-\varepsilon$, $0\le\beta,\gamma\le1$,
$\Delta t_{\min}\le\Delta t\le\Delta t_{\max}$, per branch. Written as 8 rows $g\ge0$.
Unlike the path rows these are active at **every** knot including 0
(`InputBoundsConstraint` has no `isActive` override).

### 5.2 Velocity bounds — **frame index differs**

I imposes, at knot $i$, in that knot's own body frame:

$$
\dot c^{\,2}_{x,\max}-\bigl(R_{01}(\theta_i)\dot c_i\bigr)^2_x\ \ge 0,
\qquad
\dot c^{\,2}_{y,\max}-\bigl(R_{01}(\theta_i)\dot c_i\bigr)^2_y\ \ge 0,
\qquad
\dot\theta^{\,2}_{\max}-\dot\theta_i^2\ \ge 0 .
$$

R imposes $\bigl|R_{01}(\theta_i)\,\dot c_{i+1}\bigr|\le\dot c_{\max}$ — the velocity of knot
$i+1$ rotated by the yaw of knot $i$. I rotates $\dot c_i$ by $\theta_i$. Re-indexed onto
the same knot, R evaluates the rotation one knot *earlier* than the velocity; I evaluates
both at the same knot. **Not an exact re-indexing of R** — see §14.

Squared form is used so the rows are smooth.

### 5.3 Friction — equivalent set, rescaled

$$
1-\frac{\lVert f^{(0)}\rVert^2}{f_n^{(0)2}+10^{-3}}\ \ge 0,
\qquad
1-\frac{\lVert f^{(1)}\rVert^2}{f_n^{(1)2}+10^{-3}}\ \ge 0,
$$

with $f_n^{(0)}=\alpha\mu m g$, $f_n^{(1)}=(1-\alpha)\mu m g$. Clearing the denominator
gives R §5.3 exactly. The division normalises the row to the same order of magnitude as
the box bounds, which matters for a QP that sees all rows in one matrix.

### 5.4 Reachability — **re-indexed, same set**

For a foot $p$, its hip offset $h_\ell$ and side sign $s_\ell=\pm1$, with everything
measured against **this** knot's base $(c_i,\theta_i)$ and
$p^{\mathrm b}=R_{01}(\theta_i)(p-c_i)$:

$$
r_{\mathrm{hip}}^2-\bigl\lVert p^{\mathrm b}-h_\ell\bigr\rVert^2\ \ge 0,
\qquad
s_\ell\, p^{\mathrm b}_y\ \ge 0 .
$$

Applied to **four** feet: the two current stance feet $p^{(0)},p^{(1)}$ with the pair
$\mathcal{G}_i$, and the two previous stance feet $p^{-,(0)},p^{-,(1)}$ with the pair
$\mathcal{G}_{i-1}$. That is 8 rows per branch.

R applies the same two conditions to $\{p_i,p_{i+1}\}$ against base $i+1$. Shifting the
index maps one onto the other; I needs $p^-$ in the state to do it knot-locally.

This family is what caps stride length, and hence top speed. The header notes that
dropping it lets the robot reach $1.53\,\mathrm{m/s}$ against the reference's
$0.88\,\mathrm{m/s}$.

### 5.5 Separating-hyperplane collision avoidance — **two differences**

**(a) Angle parameterisation.** I carries $\phi$ and sets $a=(\cos\phi,\sin\phi)$, so
$\lVert a\rVert_2=1$ holds *identically* and R's norm equality disappears from the problem
entirely. This is load-bearing: the rows

$$
-(a^\top y+b)\ \ge 0,
\qquad
a^\top o+b-d_{\min}\ \ge 0
$$

are positively homogeneous in $(a,b)$, so with a free $a$ the solver can certify a
geometrically **impossible** separation by inflating $\lVert a\rVert$ and dumping the
infeasibility into the norm equality. Measured with the free-vector encoding:
$\lVert a\rVert\approx95$ on the first solve of S4.

**(b) Two planes per obstacle, matching R — with the midpoint formed backwards.** R §5.5
imposes the certificate **twice** per obstacle per interval: mid-step (hips only, at
$c_{i+1/2},\theta_{i+1/2}$) and landing (hips and feet). I imposes both, but the mid-step
pose cannot be $(c_i+c_{i+1})/2$ without composing with the dynamics, which §5.0 exists to
avoid. Instead the previous pose is carried in the state (§4.1) and the midpoint is formed
backwards:

$$
c_{i-1/2}=\tfrac{1}{2}\bigl(c^-_i+c_i\bigr),
\qquad
\theta_{i-1/2}=\tfrac{1}{2}\bigl(\theta^-_i+\theta_i\bigr),
$$

imposed over $i=1..N$. That is the same family of interval midpoints as R's, shifted one
index — the same device already used for the previous footholds in §5.4.

Mid-step plane $(a_{\mathrm{mid}},b_{\mathrm{mid}})$, hips only — 5 rows:

$$
-(a^\top r_\ell(c_{i-1/2},\theta_{i-1/2})+b)\ \ge 0
\quad \forall\ell,
\qquad
a^\top o+b-d_{\min,i}-10^{-3}\ \ge 0 .
$$

Landing plane $(a_{\mathrm{land}},b_{\mathrm{land}})$, hips and both stance feet at the
knot pose — 7 rows:

$$
-(a^\top r_\ell(c_i,\theta_i)+b)\ \ge 0
\quad\forall\ell,
\qquad
-(a^\top p^{(k)}_i+b)\ \ge 0
\quad k\in\{0,1\},
\qquad
a^\top o+b-d_{\min,i}-10^{-3}\ \ge 0 .
$$

12 rows per obstacle per branch. Both planes use the same $d_{\min,i}$, as R does.

> Adding the mid-step plane did more than restore fidelity: on S4 at $N=6$ it took the
> closed loop from 8.65 s with a collision and 3 fallbacks to the **full 23.1 s,
> collision-free, with 0 fallbacks**. The plane variables are otherwise free and
> uncoupled between knots, so a single plane per knot let the solver swing the certificate
> arbitrarily from knot to knot; the mid-step row shares $c^-$ with the previous knot and
> supplies the missing continuity. Cost: solve time roughly doubled (0.54 s → 0.97 s mean).

### 5.6 Terminal CoP — equivalent, composed with dynamics

Active for `time >= N - 1.5`. Unlike the path rows this one *does* substitute the
dynamics, since $x_N$ is not a knot at which an input lives:

$$
c_N - z\bigl(p_N,\alpha_{N-1}\bigr) = 0,
\qquad
x_N=f(x_{N-1},u_{N-1}),
$$

as two equality rows per branch (4 total). Matches R §5.6.

### 5.7 Terminal obstacle constraint — **extra, not in R**

I adds a final-state inequality per branch per obstacle, absent from R:

$$
\bigl\lVert c_N-o\bigr\rVert^2
-
\bigl(r_{\mathrm{obs}}+\text{growth}+\rho_{\mathrm{hull}}\bigr)^2
\ \ge 0,
\qquad
\rho_{\mathrm{hull}}=\max_\ell\lVert h_\ell\rVert .
$$

The squared form keeps it smooth at $c_N=o$. It is a conservative ball-vs-ball
sufficient condition, replacing the hyperplane certificate at the final knot where no
input (and hence no $(\phi,b)$) exists.

### 5.8 First-input consensus

$$
u^{\mathrm{opti}}_0-u^{\mathrm{pessi}}_0=0
$$

as 8 equality rows, active for `time < 0.5`. Matches R §8.

---

## 6. Running cost — **one genuine discrepancy**

Per knot, on the **optimistic branch only** (matching R, and matching
`ocp_quadruped.py:485`):

$$
\ell_i
=
\underbrace{w_c\lVert c_i-c_{\mathrm{goal}}\rVert^2
+w_{\dot c}\lVert\dot c_i\rVert^2
+w_{\dot\theta}\dot\theta_i^2
+w_\theta(\cdot)^2}_{\texttt{runningStateCost}}
+
w_p\sum_{k}\bigl\lVert p^{(k)}_i-(c_i+R(\theta_i)h_{\ell^{(k)}_i})\bigr\rVert^2
+
w_\alpha(\alpha_i-\tfrac12)^2
+
w_{\Delta t}(\Delta t_i-\Delta t_\star)^2 .
$$

Final cost: `runningStateCost` on $x^{\mathrm{opti}}_N$ only.

Two notes:

1. **Foothold term index.** R writes $\ell_p$ on $x_{i+1}$; I writes it on knot $i$ and
   sums over $0..N$. Same family of terms.
2. **Rotation discrepancy — R appears to be wrong here.** R §6 states $\ell_p$ uses
   $R_{01}=R(-\theta)$, explicitly flagging that collision uses $R(\theta)$. The reference
   code confirms it: `R01(v,theta)` is $R(-\theta)$, i.e. world→body, and `hip_pos_01`
   computes `c + R01(hip_pos, theta)` — applying a **world→body** rotation to a
   **body-frame** hip offset and adding the result to a world CoM position. Placing a body
   offset in the world requires $R(+\theta)$. I uses `c + applyR(theta, hip)` $=R(+\theta)$,
   which is the geometrically consistent choice and matches what I does everywhere else.

   The two agree only at $\theta=0$; elsewhere R's cost pulls the feet toward a
   mirrored-yaw hip position. Because this sits in the **cost** and not a constraint, it
   biases the preferred foothold rather than making anything infeasible — which is
   presumably why it survived in the reference.

The $w_\theta$ alignment term is algebraically identically zero and $w_\theta=0$ in all
configs; it is carried in both for 1:1 fidelity.

Weights (`OptiPessiModelParameters.h`): $w_c=1.0$, $w_{\dot c}=2.0$, $w_p=0.5$,
$w_\alpha=0.5$, $w_{\Delta t}=10^{-2}$, $w_{\dot\theta}=2.0$, $w_\theta=0$.

> **No slack, no penalty term.** A per-obstacle slack $s_j$ relaxing the pessimistic
> keep-out, penalised at $10^4 s+10^5 s^2$, was tried and removed. Against $w_c=1.0$ it
> made flying the CoM tens of metres cheaper than carrying $0.2\,$m of slack; the solver
> did exactly that and the closed loop diverged. See the note in `definitions.h`.

---

## 7. Obstacle prediction

### 7.1 Optimistic set

$d_{\min}=r_{\mathrm{obs}}$ at every knot. Matches R.

### 7.2 Pessimistic set — **plus a continuation scale**

$$
d^{\mathrm{pessi}}_{\min,i}
=
r_{\mathrm{obs}}
+
\sigma\, v_{\mathrm{obs}}\, T_i ,
\qquad
T_i=\sum_{k<i}\Delta t^{\mathrm{pessi}}_k .
$$

With $\sigma=1$ this is R §7.2 exactly, with $T_i$ read off the clock state rather than
re-summed. $\sigma$ is the closed-loop **keep-out continuation** knob (§9.2); it is not
part of the mathematical model.

> **Implementation hazard.** $\sigma$ is a genuine runtime parameter, passed through
> `getParameters`. $v_{\mathrm{obs}}$ is **not** — it is captured at CppAD trace time and
> compiled into the generated library as a constant. Changing `obstacles.maxSpeed` in a
> config has no effect unless the library folder is regenerated.

### 7.3 Scenario samples

Not implemented. R §7.3 and R §11 (scenario MPC) have no counterpart here.

---

## 8. The OCP

$$
\begin{aligned}
\min_{X_{0:N},\,U_{0:N-1}}\quad
& \sum_{i=0}^{N}\ell_i\bigl(x^{\mathrm{opti}}_i,u^{\mathrm{opti}}_i\bigr) \\
\text{s.t.}\quad
& X_0=\bigl[x_{\mathrm{init}},\,x_{\mathrm{init}},\,0\bigr], \\
& X_{i+1}=F(X_i,U_i), && i=0..N-1, \\
& u^{\mathrm{opti}}_0=u^{\mathrm{pessi}}_0, \\
& \text{input bounds (§5.1)}, && i=0..N-1, \\
& \text{path + collision rows (§5.2--5.5)}, && i=1..N, \\
& \text{terminal CoP (§5.6)}, && i\ge N-1, \\
& \text{terminal obstacle (§5.7)}, && i=N .
\end{aligned}
$$

Row counts per knot, $M$ obstacles: input bounds $2\times10$; path
$2\times(11+12M)$; consensus 8 (knot 0 only); terminal CoP 4; terminal obstacle $2M$.

---

## 9. Receding horizon

### 9.1 Loop

Steps 1–5 of R §9 are implemented in `runClosedLoopSimulation`, with the same gait parity
rule, the same obstacle motion laws, and the same stopping rule
($t\ge T_{\mathrm{sim}}$ or $\lVert c_n-c_{\mathrm{goal}}\rVert\le\delta$).

Warm start: `shiftPrimalSolution` shifts the **stored** trajectory one knot and replaces
knot 0 with the measurement. It deliberately does *not* re-integrate: the LIP is
exponentially unstable ($\cosh(\omega\Delta t)\approx2.96$) and the footholds in $u$ are
absolute world positions, so re-integrating amplifies any measurement mismatch $\sim700\times$
across the horizon. R does the same (`set_initial(x[:,i], x_guess[:,i+1])`).

### 9.2 Safeguards not present in R

R applies whatever IPOPT returns, saturating the first input on failure. I adds four
mechanisms, all of which exist because HPIPM's IPM has **no restoration phase**:

| Mechanism | Rule |
| --- | --- |
| Applied-step gate | worst violation at knot 1 $<0.05$, and $\lVert x_1^{\text{solver}}-f(x_0,u_0)\rVert<0.02$ |
| Plan-trustworthiness | worst violation over the horizon $<0.05$; gates warm-start reuse |
| Keep-out continuation | on an untrustworthy plan, re-solve at $\sigma\in\{0,0.25,0.5,0.75,1\}$, each warm-starting the next, and keep the best feasible one |
| Fallback | capture-point (deadbeat) step: CoP at the DCM $\xi=c+\dot c/\omega$, footholds under $\xi$, $\Delta t=\Delta t_{\min}$ |

Two caveats worth stating in a formulation document, because they bound what the closed
loop can be said to guarantee:

- The fallback is **not** deadbeat once $\xi$ leaves the support segment: $\alpha$
  saturates, the CoP cannot reach the capture point, and the DCM grows. Measured
  $1.70\times$ per phase against $e^{\omega\Delta t}=2.76$ open loop.
- `result.collision` is evaluated **only on steps where every solve failed**, so
  "no collision" is not a statement about the successful steps.

### 9.3 Solver

| | R | I |
| --- | --- | --- |
| NLP | CasADi `Opti` | OCS2 `OptimalControlProblem` |
| Algorithm | IPOPT (primal-dual IPM **with restoration**) | `ocs2::IpmSolver` (primal-dual IPM, **no restoration**) |
| Linear algebra | MUMPS | HPIPM, `reg_prim = 1e2` |
| Barrier | IPOPT default | $10^{-2}\to10^{-4}$, 100 iterations |

`reg_prim = 1e2` is unusually large (HPIPM defaults are $\sim10^{-9}$) and is needed
because the collision rows are bilinear in $(\phi,b)$ and the pose, so the QP Hessian is
routinely singular. A sweep over $10^{-6}..10^{1}$ on S4 makes every outcome worse, so
the value is empirically justified rather than merely inherited.

---

## 10–11. Variants

R §10 (robust/classic single-trajectory MPC) and R §11 (scenario MPC) are **not
implemented**. Only the Opti-Pessi controller of R §8 exists here.

---

## 12. Compact statement

Identical in form to R §12, with $\mathcal{X}$ additionally containing the previous-foot
reachability family and the terminal obstacle ball, and $\mathcal{C}$ containing one
angle-parameterised hyperplane per obstacle per knot instead of two free-vector planes
per interval.

---

## 13. Configuration

| Quantity | Value | Source |
| --- | --- | --- |
| $h,g,\omega$ | $0.38\,$m, $9.81$, $5.08\,\mathrm{s^{-1}}$ | `task.info` |
| $\Delta t\in[\Delta t_{\min},\Delta t_{\max}]$ | $[0.20,0.35]\,$s | `task.info` |
| $\dot c_{x,\max},\dot c_{y,\max},\dot\theta_{\max}$ | $1.5,\ 0.45,\ 0.8$ | `task.info` |
| $r_{\mathrm{hip}}$ | $0.1\,$m | `task.info` |
| $\varepsilon$ (`alphaReduction`) | $0.1$ | `task.info` |
| $N$ | 6 | `task.info` |
| $r_{\mathrm{obs}},v_{\mathrm{obs}}$ | $0.2\,$m, $1.0\,$m/s | scenario |

With $N=6$ and $\Delta t_{\max}$, the keep-out reaches
$r_{\mathrm{obs}}+v_{\mathrm{obs}}T_N\approx2.3\,$m while the robot caps at
$1.5\,$m/s — the far knots of the pessimistic branch are then genuinely infeasible. This
is the dominant practical limitation of the implementation, and it is a property of the
*model* (R §7.2), not of the port. Measured on S4: $N=4$ completes the scenario,
$N=6$ did not until the mid-step plane was added (§5.5); with it, $N=6$ completes S4
collision-free.

---

## 14. Divergence table

**Equivalent set, different transcription** — these should not change the optimum:

| # | Item | R | I |
| --- | --- | --- | --- |
| 1 | Path/collision rows | on $x_{i+1}$, composed with $f$ | on knot $i$, imposed $1..N$ |
| 2 | Reachability | $\{p_i,p_{i+1}\}$ vs base $i+1$ | $\{p^-_i,p_i\}$ vs base $i$, using state-carried $p^-$ |
| 3 | Friction | $\lVert f\rVert^2\le f_n^2+\epsilon_f$ | same, divided through by $f_n^2+\epsilon_f$ |
| 4 | Elapsed time | $\sum_k\Delta t^{\mathrm{pessi}}_k$ recomputed | clock state $T_i$ |
| 5 | Foothold cost index | on $x_{i+1}$ | on knot $i$ |

**Genuine differences in the problem being solved:**

| # | Item | R | I | Consequence |
| --- | --- | --- | --- | --- |
| 6 | Hyperplane normal | free $a$, $\lVert a\rVert_2=1$ equality | $a=(\cos\phi,\sin\phi)$ | I removes the homogeneity cheat ($\lVert a\rVert\approx95$ in R's encoding); strictly better |
| 7 | Planes per obstacle per interval | 2 (mid-step + landing), midpoint towards $i+1$ | 2, midpoint formed backwards from $c^-$ | **closed** — same family shifted one index; fixed S4 at $N=6$ |
| 8 | Foothold-cost rotation | $R_{01}=R(-\theta)$ — body offset through a world→body map, **inconsistent** | $R(\theta)$ | I is correct; R's cost pulls toward a mirrored-yaw hip whenever $\theta\ne0$ |
| 9 | Velocity-bound frame | $R_{01}(\theta_i)\,\dot c_{i+1}$ | $R_{01}(\theta_i)\,\dot c_i$ | rotation evaluated at a different knot than the velocity |
| 10 | Terminal obstacle | — | ball-vs-ball with $\rho_{\mathrm{hull}}$ | I is more conservative at knot $N$ |
| 11 | Robust / scenario variants | present | absent | — |

**Numerical / infrastructure:**

| # | Item | R | I |
| --- | --- | --- | --- |
| 12 | Restoration phase | IPOPT has one | HPIPM does not — hence §9.2 |
| 13 | Failure handling | saturate first input, integrate | applied gate, plan gate, continuation, capture-point fallback |
| 14 | $v_{\mathrm{obs}}$ | runtime | **baked into the CppAD library** at trace time |
| 15 | Hessian regularisation | IPOPT adaptive | fixed `reg_prim = 1e2` |

---

## 15. Open items

- §14.7 (missing mid-step plane) is **closed**: both planes are now imposed, with the
  midpoint formed from the previous knot. No remaining divergence weakens a safety
  guarantee relative to R.
- §14.8 is settled: I is geometrically correct and R has a sign bug in the foothold cost.
  `mpc_formulation.md` §6 should be amended to record that, since it currently presents
  R's $R_{01}$ as the definition rather than as a defect.
- §14.9 (velocity-bound frame) is small and still unreviewed; one of the two is a
  mis-indexing and it is worth deciding which.
- `result.collision` should be evaluated on every step, not only on fallback steps,
  before any collision statistic from this code is quoted.
