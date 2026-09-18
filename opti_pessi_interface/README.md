# opti_pessi_interface

OCS2 formulation of the **Optimistic–Pessimistic MPC** for robust dynamic obstacle avoidance (Leonardelli et al., 2026).

The robot is a 3D linear inverted pendulum with yaw, walking a variable-duration diagonal trot. Two full trajectories are optimized at once:

* the **optimistic** branch carries the whole cost and only avoids the *currently observed* obstacle
  disk `B(o, r_obs)`;
* the **pessimistic** branch carries *no* cost but must stay feasible against the worst-case
  reachable disk `B(o, r_obs + v_obs · Σ Δt)`;

coupled by the non-anticipativity constraint `u₀^opti = u₀^pessi`. The applied input is therefore
optimal for the nominal future *and* the first step of a plan that survives any obstacle motion
bounded by `v_obs` — without the conservatism of tracking the growing tube along the executed
trajectory.

## Layout

```
config/     task.info (model, cost, limits, horizon, ipm/mpc/rollout) + scenario_S1..S4.info
include/opti_pessi_interface/
  definitions.h                     state/input layout, augmented dimensions, TIME CONVENTION note
  OptiPessiModelParameters.h        everything loaded from the .info files
  LipKinematics.h                   templated scalar/AD helpers, the exact discrete LIP map
  OptiPessiReferenceManager.h       goal, measured obstacles, trot parity, homotopy knobs
  OptiPessiInterface.h              : ocs2::RobotInterface, assembles the OptimalControlProblem
  OptiPessiMpc.h                    : ocs2::MPC_BASE run by OptiPessiController, publishes one Plan per solve
  ObstacleDetour.h                  temporary OCP goal that walks around an obstacle blocking the goal
  dynamics/ cost/ constraint/ initialization/ simulation/
src/                                mirrors include/, plus OptiPessiMpcNode.cpp (executable)
test/                               testLipStep, testConfigLoading, testS4ClosedLoop
python/                             plot_mpc.py (reads the same .info files), info_parser.py
```

`OptiPessiInterface` owns the problem, not the solver — construct an `ocs2::IpmSolver` from
`ipmSettings()`, `getOptimalControlProblem()` and `getInitializer()`, exactly as `LeggedController`
builds its `SqpMpc` from `LeggedInterface`.

## Build and run

```bash
colcon build --symlink-install --packages-select opti_pessi_interface
```

The node writes `x_quad_*.npy`, `u_*.npy` and `y_obs_*.npy` into the working directory; plot them
with `python/plot_mpc.py <taskFile> <scenarioFile>`.

## On the robot: OptiPessiController and OptiPessiWbc

The MPC decides **one footstep per contact phase**: the next footholds, the CoP split `alpha`, the
tangential force split `beta`/`gamma` and the phase duration `dt`. It does not produce a continuous
trajectory. Two classes outside this package turn that decision into joint torques:

| class | file | role |
| --- | --- | --- |
| `legged::OptiPessiController` | `legged_controllers/{include/legged_controllers,src}/OptiPessiController.{h,cpp}` | ros2_control plugin `legged/OptiPessiController`: state estimation, LIP phase clock, CoM/foot/force references, MPC hand-off |
| `legged::OptiPessiWbc` | `legged_wbc/{include/legged_wbc,src}/OptiPessiWbc.{h,cpp}` | whole-body QP that tracks those references |
| `opti_pessi::OptiPessiMpc` | `include/opti_pessi_interface/OptiPessiMpc.h` (this package) | the MPC the controller runs; publishes one `Plan` per solve |

`legged_controllers/config/controllers.yaml` loads the plugin under the name `legged_controller`
(`update_rate: 100` Hz), and `legged_controllers/launch/load_optipessicontroller_launch.xml` passes its
parameters. The controller owns both models: `LeggedInterface` for the whole-body side (URDF, Pinocchio,
estimator, WBC) and `OptiPessiInterface` for the planner. It deliberately does not build the centroidal
`SqpMpc` of `LeggedInterface`.

### Threads

```
spin thread     /opti_pessi/goal, /opti_pessi/obstacles, /contact callbacks ──► (mutex)
                                                                                   │
MPC thread      pushOptiPessiObservation(): phase, its start state, goal, obstacles ◄─┘
(50 Hz)         advanceMpc() ──► OptiPessiMpc::calculateController() ──► Plan {phase, startState, inputs[N], source}
                                                                                   │
control thread  update(): state estimate ──► plan intake ◄─────────────────────────┘
(400 Hz)                  ──► LIP phase clock ──► CoM / foot / force references ──► OptiPessiWbc ──► joint commands
```

The MPC rate is `mpc.mpcDesiredFrequency` in `config/task.info`. Goal and obstacles are handed to the
reference manager from the MPC thread, so they cannot change while a solve is running.

### Parameters

| parameter | launch default | meaning |
| --- | --- | --- |
| `urdfFile`, `taskFile`, `referenceFile` | `legged_controllers/config/<robot_type>/…` | whole-body model, estimator and WBC settings |
| `optipessiFile` | `opti_pessi_interface/config/task.info` | this package's model, cost, limits, solver settings |
| `scenarioFile` | `config/scenario_S1.info` | obstacle slot count (`obstacles.numObstacles`) and `goalTolerance`; obstacles themselves come from the topic |
| `libraryFolder` | `opti_pessi_interface/lib` (install share) | CppAD libraries, one subfolder per robot model |
| `recompile` | `false` | regenerate the CppAD libraries |
| `backend` | `Ipm` | `Ipm` or `Sqp` |
| `obstacleDetour` | `true` | use `ObstacleDetour` (below) |

The LIP mass and yaw inertia come from the URDF (total mass, and the composite inertia about the
vertical axis at the default stance and `comHeight`), not from `task.info`. Both are compiled into the
CppAD libraries, so each model gets its own folder `libraryFolder/m<mass>_I<inertia>`, for example
`lib/m22.647_I1.3721`. **Every `task.info` value is compiled in too**: after editing `task.info`,
relaunch with `recompile:=true`, or the change has no effect.

### Startup

`on_activate()` binds the 12 joints, the IMU and the foot contact sensors. If the hardware exports no
contact interfaces, it subscribes to `/contact` instead (`std_msgs/Int16MultiArray`, contact when
`> 40`). Then the robot stands up: all four feet down, the CoM height ramped from its measured value
to `comHeight` over 1.5 s, then 0.5 s of holding. The controller then waits for a goal. When the first
goal arrives it measures phase 0 and the MPC thread, idle until then, starts solving.

### The phase loop

The LIP loop closes **once per phase**. At every phase boundary, `measureLipState()` measures the
10-dof LIP state `[cx, cy, theta, dcx, dcy, dtheta, p0x, p0y, p1x, p1y]`:

* whole-body CoM and CoM velocity from Pinocchio (the swinging legs move the CoM relative to the base);
* yaw and yaw rate from the estimator (linear Kalman filter);
* the stance feet of the new phase (`opti_pessi::gaitPair`) from forward kinematics.

Within a phase, the only feedback is the WBC's PD terms.

The MPC thread keeps re-solving the **same** phase from that same start state until the phase ends
(solver time is always knot 0; the phase index is the gait offset). A phase therefore sees several
plans, logged as `policy updates`. `storeAcceptedPlan()` takes a plan only when it was solved for the
current phase from exactly this phase's start state, which rejects a solve still running across a
restart, or when it is newer than the stored plan for an earlier phase. Until the phase's own plan
arrives, the latest plan runs shifted by the number of phases completed since it was solved (knot
`offset`, clamped to the last knot), starting from the measured state. The time spent this way is
logged as `shifted=`. The controller applies whatever `OptiPessiMpc` published, with one exception:

* **Duration floor.** A plan that arrives mid-phase may shorten the phase, but touchdown can never
  come sooner than `optiPessiMinLandingTime_` (0.08 s) from now, or sooner than the previous plan had
  it, whichever is earlier. Without this, a late plan whose `dt` is shorter than the time already spent
  ends the phase at once, and the swing feet become the stance pair in mid-air. The floor never moves
  touchdown later than the previous plan had it, so re-solves cannot stretch a phase forever. When the
  floor applies, the stretched `dt` drives everything else (swing, CoM reference, LIP prediction, phase
  end), and the touchdown log reports it as `stretched=`.

From the applied input `u` and the phase start state `x`, every control tick builds:

* **CoM:** the closed-form LIP flow of `x` with the CoP held at `z = p0 + alpha (p1 − p0)`, at height
  `comHeight`. Yaw follows the torque of the tangential forces, as in `opti_pessi::lipMap`.
* **Stance pair:** held at the positions measured at liftoff, with forces
  `f0 = (beta m ddcx, gamma m ddcy, (1 − alpha) m g)` and
  `f1 = ((1 − beta) m ddcx, (1 − gamma) m ddcy, alpha m g)`, where `ddc = omega² (c − z)`.
* **Swing pair:** cubic splines from liftoff to the footholds in `u`, with an apex of
  `optiPessiSwingHeight_` (0.08 m) above liftoff at mid-phase and zero velocity at both ends. Their
  touchdown forces come from the next knot.

At the end of the phase, the swing pair becomes the stance pair with its touchdown forces, the old
stance pair stays down unloaded, and the next phase is measured.

**Goal reached.** When the measured CoM is within the scenario's `goalTolerance` of the goal,
`holdStance()` puts all four feet down at `m g / 4` each and holds the CoM and heading. With
`scenario_S1.info` the tolerance is 0.01 m, so this rarely fires. A new goal away from the robot calls
`restartFromStance()`: the swinging feet go straight down, the robot settles, and the walk starts again
from phase 0.

### Goal and obstacles

| topic | type | content |
| --- | --- | --- |
| `/opti_pessi/goal` | `visualization_msgs/MarkerArray` | goal = position of the first `ADD` marker |
| `/opti_pessi/obstacles` | `legged_controllers/ObstacleArray` | every obstacle currently seen; each message replaces the previous one |

Both must be in `odom`, the frame the LIP state is measured in. The controller does no TF lookup and
ignores other frames. An obstacle's `type` (`HUMAN = 0`, `CAR = 1`) selects its keep-out radius and
speed bound from `obstacleTypes` in `config/task.info` (human 0.3 m, 1.0 m/s; car 2.4 m, 2.0 m/s).

The OCP has a fixed number of obstacle slots, `obstacles.numObstacles` in the scenario file. With more
obstacles than slots, the ones whose keep-out disks come closest to the CoM take the slots. Free slots
hold a point obstacle with zero radius and zero speed bound, parked 100 m away.

With `obstacleDetour` on, `ObstacleDetour` replaces the OCP goal with a point past the tangent to an
inflated disk around any obstacle that blocks the straight line to the goal. The short horizon never
pays for walking around an obstacle, so without the detour the robot stalls at the grown keep-out. The
goal-reached check still uses the real goal. RViz draws the active detour goal in cyan.

### What OptiPessiMpc publishes

Each call is a single IPM solve: `solveWithRetries(…, realTimeIteration = true)`, so there is no cold
retry and no keep-out continuation. The solve is judged by `evaluateSolve()`, which checks the dynamics
residual and the constraint violation of the step about to be applied.

* **Warm start**, only from solves that `evaluateSolve()` accepted:
  * the same phase reuses the accepted solution as it is;
  * the next phase uses it shifted by one knot (`shiftPrimalSolution`);
  * anything else starts cold, including a phase whose shifted warm start was rejected.
* **Solver reset after a rejected run.** `ocs2::IpmSolver` carries its slack and dual variables over
  from its previous run whatever primal guess it is given, so they are cleared.
* **Publishing:**
  * an accepted solve is published as `nominal`;
  * a rejected solve is published as `unchecked` while its phase has no nominal plan;
  * a rejected re-solve of a phase that already has a nominal plan is dropped, and the log reports
    `source=rejected`.

Warm-starting from rejected solves used to leave the solver stuck on its own failed output: fewer
iterations on every solve, the same violated plan executed each time, and a fall a few phases later.

### OptiPessiWbc

`OptiPessiWbc` uses the same QP as `WeightedWbc`: decision variables `x = [qdd, F, tau]`, solved with
qpOASES, under the same hard constraints from `WbcBase`: floating-base equations of motion, torque
limits, friction cone, and no motion of the stance feet. Only the weighted tasks differ, because a LIP
footstep plan carries no joint references. They are written on a `WbcReference` (feet, CoM and yaw
references in `odom`):

* **swing feet:** `J qdd = Kp (p_ref − p) + Kd (v_ref − v) − dJ v` on the spline references;
* **centroidal:** 6 rows on the **measured** model. Three rows set the CoM acceleration,
  `m ddc = A_lin qdd + dA_lin v`, with `ddc` equal to the reference CoM acceleration plus a PD on the
  CoM. Three rows act on the base orientation: PD on yaw towards the reference, PD on roll and pitch
  towards zero. Using the measured centroidal momentum matrix already includes the swinging legs, which
  is why no joint references are needed;
* **contact forces:** `F = F_ref` on all 12 components.

The contact set comes from the **reference**, not the estimator: the phase clock decides when a foot is
loaded, and the QP must agree with the force reference it is given. The weights are
`(swingLeg, baseAccel, contactForce) = (100, 1, 0.01)`. A task weight enters the objective squared, so
the ratio is 1e4 : 1 : 1e-4: the force task only nudges the QP towards the planned
`alpha`/`beta`/`gamma` split, and the centroidal task wins.

If qpOASES fails, `update()` returns the last solved `x` and counts the failure (`getNumQpFailures()`).
`getLastCentroidalResidual()` gives the achieved minus the requested CoM acceleration.

Settings are read from the controller's `taskFile`, `legged_controllers/config/<robot_type>/task.info`:

| block | keys | aliengo value |
| --- | --- | --- |
| `weight` | `swingLeg`, `baseAccel`, `contactForce` | 100, 1, 0.01 |
| `optiPessiWbc` | `comKpXY`, `comKdXY`, `comKpZ`, `comKdZ` | 10, 6, 100, 20 (untuned) |
| `optiPessiWbc` | `yawKp`, `yawKd`, `rollPitchKp`, `rollPitchKd` | 20, 9, 100, 20 (untuned) |
| `swingLegTask` | `kp`, `kd` | 350, 37 |
| `frictionConeTask` | `frictionCoefficient` | 0.6 (Gazebo foot mu; 0.3 saturated the stance feet) |
| `torqueLimitsTask` | HAA, HFE, KFE | 35.278, 35.278, 44.4 Nm |

**Joint command write-out** (`writeHardwareCommand`): feedforward torque `tau`, `kp = 0`, `kd = 3`,
and a velocity target equal to the measured joint velocity plus one control step of `qdd`. The write
is gated by `SafetyChecker`. If the base roll leaves ±π/2, `update()` returns an error and the
controller manager deactivates the controller: `[OptiPessi Controller] Safety check failed!`.

### Reading the per-phase log

At the end of every phase the controller logs one block of lines for that phase. The first line
`phase N: dt=… c=… theta=… v=…` is printed at the **start** of phase N: `c`, `theta` and `v` are the
measured start state, and `dt` is the duration of the phase that just ended.

| line | fields |
| --- | --- |
| `diag` | `t` sim time; `shifted` time the phase ran on an older plan; `solve` last MPC call [ms]; `LIP-measured` LIP prediction minus the measured state (large `p0`/`p1` errors mean the feet did not land where planned); `ref-measured` WBC reference minus the measurement; `measured \|dc\|`; WBC QP failures |
| `posture` | pitch and roll ranges, minimum base height; ticks with a stance reference but no sensor contact, and with a swing reference but sensor contact |
| `wbc` | mean CoM acceleration error of the WBC; ticks each stance foot sat on the friction pyramid |
| `touchdown` | height of the landing pair above its reference and its sensor contact; `dt seen` range of planned durations; `stretched` duration-floor extension; foothold drift; policy updates |
| `plan` | predicted body-frame velocity and yaw rate against the `dcxMax`/`dcyMax`/`dthetaMax` limits; the latest solve (`gaitOffset`, `warmStart`, `iterations`, `cost`, `source`, `pessiScale`, `trustworthy`, `dynRes`, `appliedViol`, `horizonViol`); which plan was applied, its shift and its source |
| `obstacle j` | distance from the obstacle centre to the hull of the hips and landed feet, measured and predicted, against its keep-out radius |

When a fall follows `source=unchecked` with a large `dynRes`, look at solver robustness before tuning
cost weights.

## Deviations from the reference implementation

This package was restructured from `src/legged_optipessi_interface`. The formulation is unchanged
except for three deliberate fixes:

1. **Angle-parameterized hyperplane normals** (above). Mathematically identical to the spec's
   `‖a‖₂ = 1`; removes the scale-inflation cheat and drops the input dimension from 28 to 24.
2. **The warm start shifts the stored state trajectory instead of re-integrating it.** Re-integrating
   from the new measurement looks more principled, but the LIP is exponentially unstable
   (`cosh(ω·Δt) ≈ 2.96` per knot) and the footholds in `u` are absolute world positions, so any
   mismatch is amplified ~700× across the horizon and compounds every MPC step. The Python reference
   shifts (`ocp_quadruped.py:455`); so does this port.
3. **Continuation on the keep-out growth.** On solve failure the step is retried along
   `pessiScale ∈ {0, 0.25, 0.5, 0.75, 1}`, each relaxed solve warm-starting the next. A step whose
   best successful solve had `pessiScale < 1` is *not* robust to the full `v_obs` bound; those are
   counted in `ClosedLoopResult::relaxedSteps` and reported per step, never silently accepted.

`legged_optipessi_interface` also carried `ProblemContext` as a `shared_ptr` captured by every cost
and constraint; that is now `OptiPessiReferenceManager`, consumed by `const&` through the existing
`getParameters(...)` hooks, matching how `legged_interface` constraints consume
`SwitchedModelReferenceManager`.
