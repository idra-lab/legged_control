#include "opti_pessi_wbc_bridge/ReferenceSynthesizer.h"

#include <algorithm>
#include <cmath>

#include "opti_pessi_interface/LipKinematics.h"
#include "opti_pessi_wbc_bridge/LegIndexing.h"

namespace opti_pessi_bridge {

ocs2::vector_t lipStateAt(const PhasePlan& plan, double tau, const SynthesisSettings& settings) {
  const double omega = std::sqrt(settings.gravity / settings.comHeight);

  // Duration is read from u(RobotU::DT) alone, so a copy with DT := tau gives the flow at tau.
  ocs2::vector_t partialInput = plan.robotInput;
  partialInput(opti_pessi::RobotU::DT) = std::max(0.0, tau);

  return opti_pessi::lipMapScalar(plan.robotState, partialInput, omega, settings.mass, settings.inertia);
}

namespace {

/** Cubic with zero slope at both ends: s(0) = 0, s(1) = 1, s'(0) = s'(1) = 0. */
double smoothStep(double u) {
  return u * u * (3.0 - 2.0 * u);
}

double smoothStepDerivative(double u) {
  return 6.0 * u * (1.0 - u);
}

/** Raised cosine arc: 0 at both ends, peak height at the midpoint, zero slope at the ends. */
double swingArc(double u, double height) {
  return 0.5 * height * (1.0 - std::cos(2.0 * M_PI * u));
}

double swingArcDerivative(double u, double height) {
  return M_PI * height * std::sin(2.0 * M_PI * u);
}

}  // namespace

FootReference footReferenceAt(const PhasePlan& plan, double tau, int parity, const SynthesisSettings& settings) {
  FootReference reference;
  for (size_t i = 0; i < kNumLegs; ++i) {
    reference.position[i].setZero();
    reference.velocity[i].setZero();
    reference.inContact[i] = false;
  }

  const double dtValue = std::max(plan.robotInput(opti_pessi::RobotU::DT), 1e-6);
  const double u = std::min(1.0, std::max(0.0, tau / dtValue));

  // --- stance pair: held exactly where the plan says they stand -------------------------------
  const auto stancePair = opti_pessi::gaitPair(parity);
  const Eigen::Vector2d stance0(plan.robotState(opti_pessi::RobotX::P0X),
                                plan.robotState(opti_pessi::RobotX::P0Y));
  const Eigen::Vector2d stance1(plan.robotState(opti_pessi::RobotX::P1X),
                                plan.robotState(opti_pessi::RobotX::P1Y));

  const size_t stanceContact0 = contactIndexOf(stancePair[0]);
  const size_t stanceContact1 = contactIndexOf(stancePair[1]);
  reference.position[stanceContact0] = Eigen::Vector3d(stance0.x(), stance0.y(), 0.0);
  reference.position[stanceContact1] = Eigen::Vector3d(stance1.x(), stance1.y(), 0.0);
  reference.inContact[stanceContact0] = true;
  reference.inContact[stanceContact1] = true;

  // --- swing pair: from the previous footholds to the commanded next footholds -----------------
  // The swing legs this phase are the pair that stood LAST phase, so their take-off points are the
  // previous footholds carried in the state.
  const auto swingPair = opti_pessi::gaitPair(parity + 1);
  const Eigen::Vector2d takeOff0(plan.robotState(opti_pessi::RobotX::PP0X),
                                 plan.robotState(opti_pessi::RobotX::PP0Y));
  const Eigen::Vector2d takeOff1(plan.robotState(opti_pessi::RobotX::PP1X),
                                 plan.robotState(opti_pessi::RobotX::PP1Y));
  const Eigen::Vector2d land0(plan.robotInput(opti_pessi::RobotU::P0X),
                              plan.robotInput(opti_pessi::RobotU::P0Y));
  const Eigen::Vector2d land1(plan.robotInput(opti_pessi::RobotU::P1X),
                              plan.robotInput(opti_pessi::RobotU::P1Y));

  const std::array<Eigen::Vector2d, 2> takeOff{takeOff0, takeOff1};
  const std::array<Eigen::Vector2d, 2> land{land0, land1};

  const double s = smoothStep(u);
  const double dsdu = smoothStepDerivative(u);

  for (int k = 0; k < 2; ++k) {
    const size_t contact = contactIndexOf(swingPair[k]);
    const Eigen::Vector2d planar = takeOff[k] + s * (land[k] - takeOff[k]);
    const Eigen::Vector2d planarVel = (dsdu / dtValue) * (land[k] - takeOff[k]);

    reference.position[contact] =
        Eigen::Vector3d(planar.x(), planar.y(), swingArc(u, settings.swingHeight));
    reference.velocity[contact] =
        Eigen::Vector3d(planarVel.x(), planarVel.y(), swingArcDerivative(u, settings.swingHeight) / dtValue);
    reference.inContact[contact] = false;
  }

  return reference;
}

std::array<Eigen::Vector3d, kNumLegs> contactForcesAt(const PhasePlan& plan, double tau, int parity,
                                                      const SynthesisSettings& settings) {
  std::array<Eigen::Vector3d, kNumLegs> forces;
  for (auto& f : forces) {
    f.setZero();
  }

  const double omega = std::sqrt(settings.gravity / settings.comHeight);

  // CoM at tau -- the horizontal force follows the true LIP law at the current CoM, not the knot.
  const auto xAtTau = lipStateAt(plan, tau, settings);
  const Eigen::Vector2d c(xAtTau(opti_pessi::RobotX::CX), xAtTau(opti_pessi::RobotX::CY));

  const Eigen::Vector2d p0(plan.robotState(opti_pessi::RobotX::P0X), plan.robotState(opti_pessi::RobotX::P0Y));
  const Eigen::Vector2d p1(plan.robotState(opti_pessi::RobotX::P1X), plan.robotState(opti_pessi::RobotX::P1Y));
  const double alpha = plan.robotInput(opti_pessi::RobotU::ALPHA);
  const double beta = plan.robotInput(opti_pessi::RobotU::BETA);
  const double gamma = plan.robotInput(opti_pessi::RobotU::GAMMA);

  // Horizontal split, reusing the interface's own routine so the two can never drift apart.
  Eigen::Matrix<double, 2, 1> f0;
  Eigen::Matrix<double, 2, 1> f1;
  opti_pessi::computeTangentialForces<double>(c, p0, p1, alpha, beta, gamma, omega, settings.mass, f0, f1);

  // Vertical split from the CoP definition: the net vertical force acts at p0 + alpha*(p1 - p0).
  const double weight = settings.mass * settings.gravity;
  const double fz0 = (1.0 - alpha) * weight;
  const double fz1 = alpha * weight;

  const auto stancePair = opti_pessi::gaitPair(parity);
  forces[contactIndexOf(stancePair[0])] = Eigen::Vector3d(f0.x(), f0.y(), fz0);
  forces[contactIndexOf(stancePair[1])] = Eigen::Vector3d(f1.x(), f1.y(), fz1);

  return forces;
}

CentroidalReference synthesize(const LegGeometry& geom, const PhasePlan& plan, double tau, int parity,
                               const SynthesisSettings& settings) {
  constexpr int kStateDim = 24;
  constexpr int kInputDim = 24;

  CentroidalReference reference;
  reference.state = ocs2::vector_t::Zero(kStateDim);
  reference.input = ocs2::vector_t::Zero(kInputDim);
  reference.mode = modeNumberForParity(parity);
  reference.allFeetReachable = true;

  const auto lip = lipStateAt(plan, tau, settings);
  const double yaw = lip(opti_pessi::RobotX::TH);
  const Eigen::Vector2d base(lip(opti_pessi::RobotX::CX), lip(opti_pessi::RobotX::CY));

  // --- normalized centroidal momentum (h / m) --------------------------------------------------
  reference.state(0) = lip(opti_pessi::RobotX::DCX);
  reference.state(1) = lip(opti_pessi::RobotX::DCY);
  reference.state(2) = 0.0;  // the LIP has no vertical CoM motion
  reference.state(3) = 0.0;
  reference.state(4) = 0.0;
  reference.state(5) = settings.inertia * lip(opti_pessi::RobotX::DTH) / settings.mass;

  // --- base pose: planar pose at locked height, level attitude ----------------------------------
  reference.state(6) = base.x();
  reference.state(7) = base.y();
  reference.state(8) = settings.comHeight;
  reference.state(9) = yaw;
  reference.state(10) = 0.0;  // pitch
  reference.state(11) = 0.0;  // roll

  // --- joints, from IK against the foot targets --------------------------------------------------
  const auto feet = footReferenceAt(plan, tau, parity, settings);

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  for (int foot = 0; foot < static_cast<int>(kNumLegs); ++foot) {
    const opti_pessi::Foot f = static_cast<opti_pessi::Foot>(foot);
    const size_t block = jointBlockOf(f);
    const size_t contact = contactIndexOf(f);

    // World -> base frame: undo the base translation, then the yaw rotation.
    const Eigen::Vector3d delta = feet.position[contact] - Eigen::Vector3d(base.x(), base.y(), settings.comHeight);
    const Eigen::Vector3d footInBase(c * delta.x() + s * delta.y(), -s * delta.x() + c * delta.y(), delta.z());

    bool reachable = false;
    const Eigen::Vector3d q = inverseKinematics(geom, block, footInBase, &reachable);
    reference.allFeetReachable = reference.allFeetReachable && reachable;

    reference.state(12 + 3 * block + 0) = q.x();
    reference.state(12 + 3 * block + 1) = q.y();
    reference.state(12 + 3 * block + 2) = q.z();

    // Joint velocities: the foot's velocity relative to the base, expressed in the base frame.
    // The base itself translates at (dcx, dcy, 0) and yaws at dtheta, so the foot's velocity in
    // the base frame subtracts both the base translation and the rotational carry.
    const Eigen::Vector3d footVelWorld = feet.velocity[contact];
    const Eigen::Vector3d baseVelWorld(lip(opti_pessi::RobotX::DCX), lip(opti_pessi::RobotX::DCY), 0.0);
    const double yawRate = lip(opti_pessi::RobotX::DTH);

    const Eigen::Vector3d relativeWorld = footVelWorld - baseVelWorld;
    // Undo the yaw rotation, then remove the rigid rotation term omega x r.
    const Eigen::Vector3d relativeBase(c * relativeWorld.x() + s * relativeWorld.y(),
                                       -s * relativeWorld.x() + c * relativeWorld.y(), relativeWorld.z());
    const Eigen::Vector3d rotationCarry(-yawRate * footInBase.y(), yawRate * footInBase.x(), 0.0);
    const Eigen::Vector3d footVelInBase = relativeBase - rotationCarry;

    const Eigen::Vector3d qdot = jointVelocities(geom, block, q, footVelInBase);
    reference.input(12 + 3 * block + 0) = qdot.x();
    reference.input(12 + 3 * block + 1) = qdot.y();
    reference.input(12 + 3 * block + 2) = qdot.z();
  }

  // --- contact forces, in contact order ----------------------------------------------------------
  const auto forces = contactForcesAt(plan, tau, parity, settings);
  for (size_t contact = 0; contact < kNumLegs; ++contact) {
    reference.input(3 * contact + 0) = forces[contact].x();
    reference.input(3 * contact + 1) = forces[contact].y();
    reference.input(3 * contact + 2) = forces[contact].z();
  }

  return reference;
}

}  // namespace opti_pessi_bridge
