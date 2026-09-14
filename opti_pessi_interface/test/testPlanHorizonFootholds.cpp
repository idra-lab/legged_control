#include <algorithm>
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
const char* const kFootNames[] = {"FL", "FR", "RL", "RR"};

}  // namespace

// Every foothold of the whole planned horizon, not only the applied one, must be within reach of the hip of the
// foot that lands on it. Prints each foothold's distance to its hip at liftoff and landing pose, and to the nearest
// foot standing at that knot, to show where the swing trajectories drawn in RViz come from.
TEST(PlanHorizonFootholds, EveryPlannedFootholdIsWithinReachOfItsHip) {
  OptiPessiInterface interface(configPath("task.info"), configPath("scenario_S1.info"), kLibraryFolder, /*recompile=*/false,
                               /*verbose=*/false);
  interface.setupOptimalControlProblem(kLibraryFolder, /*recompile=*/false);
  const auto referenceManager = interface.getOptiPessiReferenceManagerPtr();
  OptiPessiMpc mpc(interface.mpcSettings(), interface.ipmSettings(), interface.getOptimalControlProblem(), interface.getInitializer(),
                   referenceManager);
  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  const auto& params = interface.modelParameters();
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;

  auto worldHip = [&](const vector_t& robotState, Foot foot) {
    const vector2_t c(robotState(RobotX::CX), robotState(RobotX::CY));
    return vector2_t(c + applyR(robotState(RobotX::TH), hipOf(params, foot)));
  };

  // Standing robot as OptiPessiController measures it: CoM at the origin, stance pair of phase 0 under its hips.
  vector_t x = vector_t::Zero(RobotX::DIM);
  const auto stance0 = gaitPair(0);
  x.segment(RobotX::P0X, 2) = hipOf(params, stance0[0]);
  x.segment(RobotX::P1X, 2) = hipOf(params, stance0[1]);

  int unreachable = 0;
  for (int phase = 0; phase < 4; ++phase) {
    referenceManager->setGaitOffset(phase);
    mpc.run(0.0, packInitialState(x));
    const ocs2::PrimalSolution solution = mpc.getSolverPtr()->primalSolution(mpc.getSolverPtr()->getFinalTime());
    const int numKnots = std::min(static_cast<int>(solution.inputTrajectory_.size()), static_cast<int>(solution.stateTrajectory_.size()) - 1);

    for (int i = 0; i < numKnots; ++i) {
      const vector_t xi = extractRobotState(solution.stateTrajectory_[static_cast<size_t>(i)]);
      const vector_t xNext = extractRobotState(solution.stateTrajectory_[static_cast<size_t>(i + 1)]);
      const vector_t ui = extractRobotInput(solution.inputTrajectory_[static_cast<size_t>(i)]);
      const auto stance = gaitPair(phase + i);
      const auto next = gaitPair(phase + i + 1);
      std::printf("phase %d knot %d: CoM (%.3f, %.3f) -> (%.3f, %.3f) | velocity (%.3f, %.3f) -> (%.3f, %.3f) | theta -> %.3f | alpha=%.3f\n",
                  phase, i, xi(RobotX::CX), xi(RobotX::CY), xNext(RobotX::CX), xNext(RobotX::CY), xi(RobotX::DCX), xi(RobotX::DCY),
                  xNext(RobotX::DCX), xNext(RobotX::DCY), xNext(RobotX::TH), ui(RobotU::ALPHA));
      for (int k = 0; k < 2; ++k) {
        const vector2_t foothold(ui(RobotU::P0X + 2 * k), ui(RobotU::P0Y + 2 * k));
        const scalar_t toHipAtLanding = (foothold - worldHip(xNext, next[k])).norm();
        const scalar_t toHipAtLiftoff = (foothold - worldHip(xi, next[k])).norm();
        scalar_t toStance = 1e9;
        int nearestStance = 0;
        for (int j = 0; j < 2; ++j) {
          const scalar_t d = (foothold - vector2_t(xi(RobotX::P0X + 2 * j), xi(RobotX::P0Y + 2 * j))).norm();
          if (d < toStance) {
            toStance = d;
            nearestStance = j;
          }
        }
        std::printf("phase %d knot %d: %s lands at (%.3f, %.3f) | to its hip at landing=%.3f at liftoff=%.3f (reach %.2f) | nearest "
                    "standing foot %s at %.3f | dt=%.3f\n",
                    phase, i, kFootNames[static_cast<int>(next[k])], foothold(0), foothold(1), toHipAtLanding, toHipAtLiftoff,
                    params.footHipMax, kFootNames[static_cast<int>(stance[nearestStance])], toStance, ui(RobotU::DT));
        if (toHipAtLanding > params.footHipMax + 0.02) {
          ++unreachable;
        }
      }
    }
    x = lipMapScalar(x, extractRobotInput(solution.inputTrajectory_.front()), params.omega(), params.mass, params.inertia);
  }
  EXPECT_EQ(unreachable, 0);
}
