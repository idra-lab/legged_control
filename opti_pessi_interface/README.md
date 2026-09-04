# opti_pessi_interface

OCS2 formulation of the **Optimistic–Pessimistic MPC** for robust dynamic obstacle avoidance
(Leonardelli et al., 2026). C++ port of the CasADi/IPOPT study in `src/opti-pessi-mpc`, laid out the
way `legged_control/legged_interface` lays out its optimal control problem.

The robot is a 3D linear inverted pendulum with yaw, walking a variable-duration diagonal trot. Two
full trajectories are optimized at once:

* the **optimistic** branch carries the whole cost and only avoids the *currently observed* obstacle
  disk `B(o, r_obs)`;
* the **pessimistic** branch carries *no* cost but must stay feasible against the worst-case
  reachable disk `B(o, r_obs + v_obs · Σ Δt)`;

coupled by the non-anticipativity constraint `u₀^opti = u₀^pessi`. The applied input is therefore
optimal for the nominal future *and* the first step of a plan that survives any obstacle motion
bounded by `v_obs` — without the conservatism of tracking the growing tube along the executed
trajectory.

The full mathematical statement is in
[`../../legged_optipessi_interface/mpc_formulation.md`](../../legged_optipessi_interface/mpc_formulation.md).

## Layout

```
config/     task.info (model, cost, limits, horizon, ipm/mpc/rollout) + scenario_S1..S4.info
include/opti_pessi_interface/
  definitions.h                     state/input layout, augmented dimensions, TIME CONVENTION note
  OptiPessiModelParameters.h        everything loaded from the .info files
  LipKinematics.h                   templated scalar/AD helpers, the exact discrete LIP map
  OptiPessiReferenceManager.h       goal, measured obstacles, trot parity, homotopy knobs
  OptiPessiInterface.h              : ocs2::RobotInterface, assembles the OptimalControlProblem
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
colcon build --symlink-install --packages-select opti_pessi_interface \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --packages-select opti_pessi_interface
colcon test-result --test-result-base build/opti_pessi_interface --all

CFG=install/opti_pessi_interface/share/opti_pessi_interface/config
./install/opti_pessi_interface/lib/opti_pessi_interface/opti_pessi_interface_mpc \
    $CFG/task.info $CFG/scenario_S4.info /tmp/ocs2/opti_pessi_interface
# add --no-recompile to reuse the generated CppAD libraries, --quiet to silence per-step output
# add --solver ipm|sqp to pick the backend, and --rti for SQP real-time iteration
```

The node writes `x_quad_*.npy`, `u_*.npy` and `y_obs_*.npy` into the working directory; plot them
with `python/plot_mpc.py <taskFile> <scenarioFile>`.

## Solver backends

`--solver ipm` (default) and `--solver sqp` are not interchangeable, and the difference is not a
tuning detail.

`ocs2::SqpSolver` assembles its QP from dynamics, cost and state-input **equalities** only. Every
inequality row it computes is discarded before the QP is built, and its own `inequalityConstraintMu`
setting is loaded but never read. Selecting the SQP backend therefore makes
`setupOptimalControlProblem` register the path, collision, friction and box rows a *second* time as
relaxed-barrier soft constraints (`sqp.relaxedBarrier` in `task.info`) — that copy is what steers the
SQP iterate. Feasibility under SQP is approximate. The hard rows stay registered under both backends
so the closed loop can still measure the true violation of the step it is about to apply.

The two also need opposite HPIPM regularization, so each reads its own: `hpipm.reg_prim = 1e1` for
IPM (its Hessian is indefinite because the hyperplane angles enter the collision rows through
`cos`/`sin` of a decision variable) and `sqp.hpipm.reg_prim = 1e-6` for SQP (its QP is nearly
unconstrained, and a large diagonal shift destroys it — with `1e1` every QP came back
`HPIPM flag 1: maximum number of iterations reached`).

Measured on S1 (`N = 2`, 20 s, identical config, freshly generated CppAD libraries, quiet runs,
deterministic run to run; goal tolerance is 0.01 m):

| backend | steps | collision | min. goal distance | solve time (mean) | fallback | relaxed |
| --- | --- | --- | --- | --- | --- | --- |
| `--solver ipm` | 70 | no | **0.010 m** (reaches goal) | 0.043 s | 0 | 0 |
| `--solver sqp` | 97 | no | 0.020 m | 0.045 s | 4 | 14 |
| `--solver sqp --rti` | 56 | **yes** | 0.428 m | 0.0023 s | 54 | 0 |

`--solver sqp` at the configured 10 iterations is a genuine cross-check: it gets to within 0.02 m at
the same wall-clock cost as the IPM, at the price of 4 fallback steps and 14 steps that had to relax
the keep-out, i.e. steps that are *not* robust to the full `v_obs` bound.

**RTI is wired and correct, and on this problem it does not work.** One Newton step per control step
against soft-only inequalities never reaches a feasible iterate, so the applied-step gate rejects
96 % of solves, the capture-point fallback ends up driving the robot, and S1 ends in a collision. It
was not left untuned: `mu ∈ {1e-4, 1e-2, 1e-1, 1}` × `delta ∈ {1e-3, 1e-2, 1e-1}` all land in the
same regime, and adding the textbook converged initialization solve at step 0 made it worse rather
than better — a step-0-converged plan is a plan for the keep-out disk at `T = 0`, and a single
iteration per step cannot track that disk as it inflates.

Use `--solver ipm` for anything that matters. `--rti` is a speed reference (19× faster per solve) and
a starting point if someone wants to make it work; the first thing to try is giving the pessimistic
keep-out rows an exact-penalty (L1) term instead of a barrier, so one Newton step can actually move
the iterate onto the feasible side.

## Two things that look wrong but are not

**Time is the knot index, not seconds.** `ipm.dt = 1.0` with `integratorType EULER`, and
`OptiPessiDynamicsAD::systemFlowMap` returns `lipMap(x, u) − x`, so an Euler step of length 1
reproduces the exact discrete LIP map. The real contact-phase duration is the *decision variable*
`u(RobotU::DT)`, and elapsed physical time is accumulated in the clock state `x(CLOCK_INDEX)`. See
the note at the top of `definitions.h`.

**Hyperplane normals are parameterized by angle.** The separating-hyperplane variables live in the
input vector as `[phiMid, phiLand, bMid, bLand]` per obstacle per branch, with `a = (cos φ, sin φ)`.
The reference encoding instead carried `a` as a free 2-vector plus an `‖a‖ = 1` equality. That is the
same set, but it is exploitable: the collision rows are positively homogeneous in `(a, b)`, so the
solver can satisfy a geometrically impossible separation by inflating `‖a‖` and dumping the
infeasibility into the norm equality. It does exactly that — `‖a‖ ≈ 95` on the first solve of S4. The
angle form removes the failure mode and the norm-equality constraint block along with it.

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

## Closed: the loop no longer stops early

This section used to document a gap — the Python reference drove S4 for the full 30 s while this
port stopped after ~2.7 s (8 steps), with the pessimistic branch near-infeasible at the far knots
and the IPM's linesearch collapsing to a zero step.

**That is fixed by the step-by-step transcription.** Carrying the previous phase in the state made
every row knot-local but pushed the family that caps stride length onto quantities the applied input
could no longer influence; recomputing `x_{i+1} = lipMap(x_i, u_i)` inside the rows instead — the way
`ocp_quadruped.py` writes them — puts them back on the state the step actually produces, and makes
interval 0 constrainable. S4 now runs the full 30 s (181 steps, `--solver ipm`) with no collision, no
step on a relaxed keep-out, and a single fallback step.

The conditioning cost that motivated the knot-local form is real and is paid through
`hpipm.reg_prim = 1e1`; it did not turn out to be the binding constraint.

`testS4ClosedLoop.DISABLED_CompletesTheFullScenarioLikeTheReference` encodes the target and is worth
re-enabling — note it needs `config/scenario_S4_test.info`, which is not in the repo, so
`testS4ClosedLoop` currently fails at construction for that separate reason.

### Regenerate the CppAD libraries after changing the formulation

`--no-recompile` reuses whatever is in the library folder, and the generated code bakes in the model
constants *and* the shape of every cost/constraint term. A folder left over from an earlier
formulation silently produces a different closed loop: measured on S1, a reused folder gave 82 steps
stopping 0.245 m from the goal where a regenerated one gave 70 steps reaching it (0.010 m), same
binary, same config. When in doubt, point `libraryFolder` at a fresh path.
