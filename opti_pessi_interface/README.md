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
```

The node writes `x_quad_*.npy`, `u_*.npy` and `y_obs_*.npy` into the working directory; plot them
with `python/plot_mpc.py <taskFile> <scenarioFile>`.

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

## Known gap: the closed loop stops early

**The Python reference drives S4 for the full 30 s. This port stops after ~2.7 s (8 steps), and so do
S1–S3 (~2.2 s).** No collision occurs; the loop stops because the solver stops producing usable
solutions and the saturated fallback input is rejected as physically insane.

This is *not* a transcription error, and it is not introduced here — the
`legged_optipessi_interface` binary this package was restructured from reproduces the same
trajectory step for step and stops at the same point. (Its `compare_s4/cpp/` dump showing a full 30 s
run came from an older build; the current source does not even compile.)

Diagnosis, from `StageInequalityConstraint` evaluated on the initializer rollout: at the S4 start the
*only* violated rows are the pessimistic branch's obstacle-separation rows at knots 3–5, growing by
exactly `Δt · v_obs = 0.35` per knot. With `v_obstacle = 1.0 m/s` and a 6-phase horizon the
worst-case disk inflates to ≈ 2.1 m while the obstacle starts 1.7 m away, so the pessimistic branch
is near-infeasible at the far knots. The problem is still feasible — the pessimistic branch only has
to flee — but a standing-still guess is far from that, and the open-loop LIP is too unstable to nudge
into feasibility (biasing the initializer diverges immediately). IPOPT absorbs this with its
restoration phase; the HPIPM-based IPM has none, its linesearch collapses to a zero step size, and
the accepted iterate drifts until the velocity bound is violated by 3–4×.

Directions worth trying, roughly in order of expected payoff:

* an elastic/restoration mode — soften the pessimistic collision rows with a heavily penalized slack
  so the solver always has a feasible point to work from;
* a shorter horizon or a smaller assumed `v_obstacle`, so the worst-case disk stays inside the
  reachable region (this changes the guarantee, so it is a modelling decision, not a fix);
* seeding the pessimistic branch from a dedicated retreat plan rather than from the optimistic guess.

`testS4ClosedLoop.DISABLED_CompletesTheFullScenarioLikeTheReference` encodes the target; enable it
once this is resolved.
