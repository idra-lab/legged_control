#include <cmath>
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

}  // namespace

// The planned knot-0 forces, with the normal loads the CoP at alpha implies (z = p0 + alpha (p1 - p0), so foot 0
// carries (1 - alpha) m g and foot 1 alpha m g, as OptiPessiController::computeContactForces commands them), must
// lie inside the friction cone the OCP was given. Closed loop on the ideal LIP, as the controller walks it.
TEST(PlannedFrictionCone, CommandedForcesStayInsideTheCone) {
  OptiPessiInterface interface(configPath("task.info"), configPath("scenario_S1.info"), kLibraryFolder, /*recompile=*/false,
                               /*verbose=*/false);
  interface.setupOptimalControlProblem(kLibraryFolder, /*recompile=*/false);
  const auto referenceManager = interface.getOptiPessiReferenceManagerPtr();
  OptiPessiMpc mpc(interface.mpcSettings(), interface.ipmSettings(), interface.getOptimalControlProblem(), interface.getInitializer(),
                   referenceManager);
  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  const auto& params = interface.modelParameters();
  const scalar_t weight = params.mass * params.gravity;
  using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;

  // Standing robot as OptiPessiController measures it: CoM at the origin, stance pair of phase 0 under its hips.
  vector_t x = vector_t::Zero(RobotX::DIM);
  const auto stance = gaitPair(0);
  x.segment(RobotX::P0X, 2) = hipOf(params, stance[0]);
  x.segment(RobotX::P1X, 2) = hipOf(params, stance[1]);

  int outsideCone = 0;
  for (int phase = 0; phase < 10; ++phase) {
    referenceManager->setGaitOffset(phase);
    mpc.run(0.0, packInitialState(x));
    const ocs2::PrimalSolution solution = mpc.getSolverPtr()->primalSolution(mpc.getSolverPtr()->getFinalTime());
    const vector_t u = extractRobotInput(solution.inputTrajectory_.front());

    const vector2_t c(x(RobotX::CX), x(RobotX::CY));
    const vector2_t p0(x(RobotX::P0X), x(RobotX::P0Y));
    const vector2_t p1(x(RobotX::P1X), x(RobotX::P1Y));
    const scalar_t alpha = u(RobotU::ALPHA);
    vector2_t f0, f1;
    computeTangentialForces(c, p0, p1, alpha, u(RobotU::BETA), u(RobotU::GAMMA), params.omega(), params.mass, f0, f1);

    const scalar_t commandedRatio0 = f0.norm() / ((1.0 - alpha) * weight);
    const scalar_t commandedRatio1 = f1.norm() / (alpha * weight);
    const scalar_t ocpRatio0 = f0.norm() / (alpha * weight);
    const scalar_t ocpRatio1 = f1.norm() / ((1.0 - alpha) * weight);
    std::printf("phase %d: alpha=%.3f beta=%.3f gamma=%.3f |f0|=%.1f |f1|=%.1f  |ft|/fn commanded=(%.3f, %.3f) as the OCP bounds it=(%.3f, %.3f)"
                "  mu=%.2f\n",
                phase, alpha, u(RobotU::BETA), u(RobotU::GAMMA), f0.norm(), f1.norm(), commandedRatio0, commandedRatio1, ocpRatio0,
                ocpRatio1, params.frictionCoefficient);
    outsideCone += (commandedRatio0 > 1.02 * params.frictionCoefficient) ? 1 : 0;
    outsideCone += (commandedRatio1 > 1.02 * params.frictionCoefficient) ? 1 : 0;

    x = lipMapScalar(x, u, params.omega(), params.mass, params.inertia);
  }
  EXPECT_EQ(outsideCone, 0);
}
