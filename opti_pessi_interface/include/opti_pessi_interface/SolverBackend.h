#pragma once

#include <memory>
#include <string>

#include <ocs2_oc/oc_solver/SolverBase.h>

namespace opti_pessi {

class OptiPessiInterface;

/**
 * Which multiple-shooting solver runs the Opti-Pessi OCP.
 *
 * Both are HPIPM-based multiple shooting over the same transcription; they differ in what they do
 * with the INEQUALITY rows, and that difference is not cosmetic:
 *
 *   Ipm  ocs2::IpmSolver -- a primal-dual interior point method. The path, collision, friction and
 *        box rows go into the QP as HARD inequalities, with a slack and a dual each and a barrier
 *        parameter driven down on a schedule. This is the reference backend.
 *
 *   Sqp  ocs2::SqpSolver -- Gauss-Newton SQP. Its QP carries ONLY dynamics, cost and state-input
 *        equalities (SqpSolver.cpp, getOCPSolution): every inequality it computes is discarded
 *        before the QP is assembled, and its own `inequalityConstraintMu` setting is loaded but
 *        never read. Selecting this backend therefore ALSO registers the same inequality terms as
 *        soft constraints with a relaxed-barrier penalty, which is what actually keeps the SQP
 *        iterate away from collisions and out-of-bounds inputs. Feasibility is then approximate and
 *        depends on (mu, delta) -- see `sqp.relaxedBarrier` in config/task.info.
 *
 * The hard inequality terms stay registered in BOTH cases. The SQP QP ignores them, but the closed
 * loop evaluates them directly to measure the true violation of the step it is about to apply, and
 * that gate must not become blind just because the solver changed.
 */
enum class SolverBackend { Ipm, Sqp };

/** Parses "ipm" / "sqp". Throws std::invalid_argument on anything else. */
SolverBackend solverBackendFromString(const std::string& name);

std::string toString(SolverBackend backend);

/**
 * Builds the solver the interface was set up for, already bound to its reference manager.
 *
 * @param interface  must have had setupOptimalControlProblem() called with the SAME backend: the
 *                   SQP path needs the soft-constraint terms that call registers.
 * @param realTimeIteration  SQP only. Caps the solve at ONE Newton step per call
 *                   (sqp::Settings::sqpIteration = 1), which is the real-time iteration scheme:
 *                   the horizon is shifted and one iteration is taken per control step instead of
 *                   iterating to convergence. Ignored by the IPM backend, which has no equivalent
 *                   single-step mode (its barrier schedule needs its outer iterations).
 */
std::unique_ptr<ocs2::SolverBase> makeSolver(const OptiPessiInterface& interface, SolverBackend backend, bool realTimeIteration);

}  // namespace opti_pessi
