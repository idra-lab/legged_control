#include <cstdio>
#include <string>

#include <gtest/gtest.h>

#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_interface/OptiPessiInterface.h"
#include "opti_pessi_interface/OptiPessiMpc.h"
#include "opti_pessi_interface/initialization/OptiPessiInitializer.h"

using namespace opti_pessi;

namespace {

std::string configPath(const std::string& name) {
  return std::string(OPTI_PESSI_CONFIG_DIR) + "/" + name;
}

const std::string kLibraryFolder = "/tmp/ocs2/opti_pessi_interface_test";

/** How each contact phase is solved. Only the warm start differs, the horizon is always mpcSettings().timeHorizon_. */
enum class WarmStart {
  MpcRun,   // OptiPessiMpc::run(0, x): what OptiPessiController does through the MRT
  Cold,     // solver reset before every phase
  Shifted,  // previous solution shifted one knot, as ClosedLoopSimulation does
};

/**
 * Solves `numPhases` contact phases on the ideal LIP (x_{k+1} = lipMap(x_k, u_k)) with the gait offset advanced
 * between phases, and returns how many planned footholds land on the wrong side of the body.
 */
int countWrongSideFootholds(WarmStart warmStart, int numPhases) {
  OptiPessiInterface interface(configPath("task.info"), configPath("scenario_S1.info"), kLibraryFolder, /*recompile=*/false,
                               /*verbose=*/false);
  interface.setupOptimalControlProblem(kLibraryFolder, /*recompile=*/false);
  const auto referenceManager = interface.getOptiPessiReferenceManagerPtr();
  OptiPessiMpc mpc(interface.mpcSettings(), interface.ipmSettings(), interface.getOptimalControlProblem(), interface.getInitializer(),
                   referenceManager, interface.modelParameters());
  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  const auto& params = interface.modelParameters();
  const scalar_t horizon = interface.mpcSettings().timeHorizon_;
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;

  int wrongSide = 0;
  // Standing robot as OptiPessiController measures it: CoM at the origin, stance pair of phase 0 under its hips.
  vector_t x = vector_t::Zero(RobotX::DIM);
  const auto stance = gaitPair(0);
  x.segment(RobotX::P0X, 2) = hipOf(params, stance[0]);
  x.segment(RobotX::P1X, 2) = hipOf(params, stance[1]);
  ocs2::PrimalSolution previous;
  for (int phase = 0; phase < numPhases; ++phase) {
    referenceManager->setGaitOffset(phase);
    auto& solver = *mpc.getSolverPtr();
    if (warmStart == WarmStart::MpcRun) {
      mpc.run(0.0, packInitialState(x));
    } else if (warmStart == WarmStart::Shifted && phase > 0) {
      solver.run(0.0, packInitialState(x), horizon, shiftPrimalSolution(previous, x));
    } else {
      solver.reset();
      solver.run(0.0, packInitialState(x), horizon);
    }
    previous = solver.primalSolution(solver.getFinalTime());
    const vector_t u = extractRobotInput(previous.inputTrajectory_.front());
    const vector_t xNext = lipMapScalar(x, u, params.omega(), params.mass, params.inertia);

    const vector2_t cNext(xNext(RobotX::CX), xNext(RobotX::CY));
    const auto next = gaitPair(phase + 1);
    for (int k = 0; k < 2; ++k) {
      const vector2_t foothold(u(RobotU::P0X + 2 * k), u(RobotU::P0Y + 2 * k));
      const scalar_t lateral = applyR01(xNext(RobotX::TH), vector2_t(foothold - cNext))(1);
      std::printf("phase %d: foot %d foothold=(%.3f, %.3f) lateral=%.3f  stance p%d=(%.3f, %.3f)\n", phase, static_cast<int>(next[k]),
                  foothold(0), foothold(1), lateral, k, x(RobotX::P0X + 2 * k), x(RobotX::P0Y + 2 * k));
      if ((lateral > 0.0) != isLeft(next[k])) {
        ++wrongSide;
      }
    }
    x = xNext;
  }
  return wrongSide;
}

}  // namespace

// Every planned foothold must land on its own side of the body: left feet left of the CoM, right feet right of it.
TEST(MpcPhaseWarmStart, MpcRunKeepsFootholdsOnTheirSide) {
  EXPECT_EQ(countWrongSideFootholds(WarmStart::MpcRun, 10), 0);
}

TEST(MpcPhaseWarmStart, ColdStartKeepsFootholdsOnTheirSide) {
  EXPECT_EQ(countWrongSideFootholds(WarmStart::Cold, 10), 0);
}

TEST(MpcPhaseWarmStart, ShiftedWarmStartKeepsFootholdsOnTheirSide) {
  EXPECT_EQ(countWrongSideFootholds(WarmStart::Shifted, 10), 0);
}
