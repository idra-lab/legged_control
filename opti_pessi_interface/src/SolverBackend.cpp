#include "opti_pessi_interface/SolverBackend.h"

#include <stdexcept>

#include <ocs2_ipm/IpmSolver.h>
#include <ocs2_sqp/SqpSolver.h>

#include "opti_pessi_interface/OptiPessiInterface.h"

namespace opti_pessi {

SolverBackend solverBackendFromString(const std::string& name) {
  if (name == "ipm") {
    return SolverBackend::Ipm;
  }
  if (name == "sqp") {
    return SolverBackend::Sqp;
  }
  throw std::invalid_argument("[solverBackendFromString] unknown solver backend '" + name + "' (expected 'ipm' or 'sqp')");
}

std::string toString(SolverBackend backend) {
  return backend == SolverBackend::Ipm ? "ipm" : "sqp";
}

std::unique_ptr<ocs2::SolverBase> makeSolver(const OptiPessiInterface& interface, SolverBackend backend, bool realTimeIteration) {
  if (backend != interface.solverBackend()) {
    throw std::invalid_argument("[makeSolver] the interface was set up for backend '" + toString(interface.solverBackend()) +
                                "' but a '" + toString(backend) +
                                "' solver was requested. The backend changes the problem, not just the solver "
                                "(see SolverBackend.h) -- pass it to setupOptimalControlProblem().");
  }

  std::unique_ptr<ocs2::SolverBase> solver;
  if (backend == SolverBackend::Ipm) {
    solver = std::make_unique<ocs2::IpmSolver>(interface.ipmSettings(), interface.getOptimalControlProblem(),
                                               interface.getInitializer());
  } else {
    ocs2::sqp::Settings settings = interface.sqpSettings();
    if (realTimeIteration) {
      // Real-time iteration: exactly one Newton step per control step, taken from the shifted
      // previous solution. The iterate is never driven to convergence -- convergence happens ALONG
      // the closed loop instead, which is the whole point of the scheme.
      settings.sqpIteration = 1;
    }
    solver = std::make_unique<ocs2::SqpSolver>(std::move(settings), interface.getOptimalControlProblem(),
                                               interface.getInitializer());
  }
  solver->setReferenceManager(interface.getReferenceManagerPtr());
  return solver;
}

}  // namespace opti_pessi
