#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>

#include "opti_pessi_interface/OptiPessiInterface.h"
#include "opti_pessi_interface/SolverBackend.h"
#include "opti_pessi_interface/simulation/ClosedLoopSimulation.h"
#include "opti_pessi_interface/simulation/NpyIo.h"

using namespace opti_pessi;

namespace {

scalar_t minimumGoalDistance(const matrix_t& X, const vector_t& goal) {
  scalar_t best = std::numeric_limits<scalar_t>::max();
  for (int k = 0; k < X.cols(); ++k) {
    best = std::min(best, (X.block(0, k, 2, 1) - goal).norm());
  }
  return best;
}

/** Mean, standard deviation and 95th percentile, dropping the first solve (CppAD warm-up). */
void solveTimeStatistics(const vector_t& times, scalar_t& mean, scalar_t& stddev, scalar_t& p95) {
  const int n = static_cast<int>(times.size());
  const int start = n > 1 ? 1 : 0;
  const int m = n - start;
  if (m <= 0) {
    mean = stddev = p95 = 0.0;
    return;
  }
  mean = times.segment(start, m).mean();
  stddev = std::sqrt((times.segment(start, m).array() - mean).square().sum() / static_cast<scalar_t>(m));
  vector_t sorted = times.segment(start, m);
  std::sort(sorted.data(), sorted.data() + m);
  const int index = std::min(m - 1, std::max(0, static_cast<int>(std::ceil(0.95 * static_cast<scalar_t>(m))) - 1));
  p95 = sorted(index);
}

void dumpTrajectories(const OptiPessiModelParameters& params, const ClosedLoopResult& result) {
  const std::string suffix = params.figName + "_opti_pessi.npy";
  saveNpy("x_quad_" + suffix, result.stateTrajectory);
  saveNpy("u_" + suffix, result.inputTrajectory);
  saveNpy3("y_obs_" + suffix, result.obstacleTrajectories);
  std::cout << "Wrote x_quad_/u_/y_obs_" << suffix << "\n";
}

void printUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " <taskFile> <scenarioFile> [libraryFolder] [--solver ipm|sqp] [--rti]"
            << " [--no-recompile] [--quiet]\n"
            << "  taskFile       config/task.info\n"
            << "  scenarioFile   config/scenario_S4.info\n"
            << "  libraryFolder  CppAD library folder (default /tmp/ocs2/opti_pessi_interface)\n"
            << "  --solver       ipm (default, hard inequalities) or sqp (relaxed-barrier soft\n"
            << "                 inequalities -- SqpSolver's QP has no inequality rows)\n"
            << "  --rti          SQP only: one Newton step per control step, always warm-started,\n"
            << "                 no cold retry and no keep-out continuation (real-time iteration)\n"
            << "  --no-recompile reuse the CppAD libraries already in libraryFolder\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string optipessiFile = argv[1];
  const std::string scenarioFile = argv[2];
  std::string libraryFolder = "/tmp/ocs2/opti_pessi_interface";
  bool recompile = true;
  bool verbose = true;
  SolverBackend backend = SolverBackend::Ipm;
  bool realTimeIteration = false;

  for (int i = 3; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--no-recompile") {
      recompile = false;
    } else if (arg == "--quiet") {
      verbose = false;
    } else if (arg == "--rti") {
      realTimeIteration = true;
    } else if (arg == "--solver") {
      if (i + 1 >= argc) {
        printUsage(argv[0]);
        return 1;
      }
      try {
        backend = solverBackendFromString(argv[++i]);
      } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        printUsage(argv[0]);
        return 1;
      }
    } else if (arg.rfind("--", 0) == 0) {
      printUsage(argv[0]);
      return 1;
    } else {
      libraryFolder = arg;
    }
  }

  if (backend != SolverBackend::Ipm || realTimeIteration) {
    std::cerr << "The closed loop runs on ocs2::IpmMpc: --solver sqp and --rti (SQP only) are not supported.\n";
    return 1;
  }

  try {
    OptiPessiInterface interface(optipessiFile, scenarioFile, libraryFolder, recompile, verbose);
    std::cout << "Built the Opti-Pessi optimal control problem: stateDim=" << interface.stateDim()
              << " inputDim=" << interface.inputDim() << " numObstacles=" << interface.numObstacles() << "\n";

    interface.setupOptimalControlProblem(libraryFolder, recompile, backend);

    ocs2::IpmMpc mpc(interface.mpcSettings(), interface.ipmSettings(), interface.getOptimalControlProblem(),
                     interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());

    std::cout << "\nSimulating Opti-Pessi MPC  [solver=" << toString(backend) << "]\n";
    const ClosedLoopResult result = runClosedLoopSimulation(interface, mpc, verbose, realTimeIteration);
    const auto& params = interface.modelParameters();

    std::cout << "\nCollision:             " << (result.collision ? "yes" : "no") << "\n";
    std::cout << "Number of steps:       " << result.inputTrajectory.cols() << "\n";
    std::printf("Minimum goal distance: %.3f\n", minimumGoalDistance(result.stateTrajectory, params.goal));

    scalar_t mean = 0.0;
    scalar_t stddev = 0.0;
    scalar_t p95 = 0.0;
    solveTimeStatistics(result.solveTimes, mean, stddev, p95);
    std::printf("Solve time [mean, std, p95]: %.4f %.4f %.4f s\n", mean, stddev, p95);
    std::printf("Simulated time:        %.3f s\n", result.inputTrajectory.row(RobotU::DT).sum());
    std::printf("Steps on a relaxed keep-out (pessiScale < 1): %d\n", result.relaxedSteps);
    std::printf("Steps where every solve failed (fallback):    %d\n", result.fallbackSteps);
    if (result.relaxedSteps > 0 || result.fallbackSteps > 0) {
      std::cout << "  NOTE: those steps are not robust to the full v_obstacle bound.\n";
    }

    dumpTrajectories(params, result);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "opti_pessi_mpc failed: " << e.what() << "\n";
    return 1;
  }
}
