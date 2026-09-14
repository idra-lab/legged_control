//
// Weighted whole-body control on explicit CoM and foot references.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/OptiPessiWbc.h"

#include <pinocchio/algorithm/centroidal.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <qpOASES.hpp>

#include <cmath>
#include <stdexcept>

namespace legged {

vector_t OptiPessiWbc::update(const WbcReference& reference, const vector_t& rbdStateMeasured) {
  contactFlag_ = reference.contact;
  numContacts_ = 0;
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
  }
  updateMeasured(rbdStateMeasured);

  // Constraints first: the EoM rows read data.M and data.nle, and formulateCentroidalTask() runs dccrba on the same data.
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // Cost
  const Task swingFootTask = formulateSwingFootTask(reference);
  const Task centroidalTask = formulateCentroidalTask(reference);
  const Task forceTask = formulateReferenceForceTask(reference);
  Task weighedTask = swingFootTask * weightSwingLeg_ + centroidalTask * weightCentroidal_ + forceTask * weightContactForce_;
  // Small regularisation as in WeightedWbc: H = AᵀA is only semi-definite.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
      weighedTask.a_.transpose() * weighedTask.a_ + 1e-6 * matrix_t::Identity(getNumDecisionVars(), getNumDecisionVars());
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 200;

  vector_t qpSol(getNumDecisionVars());
  const bool solved =
      qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr) == qpOASES::SUCCESSFUL_RETURN &&
      qpProblem.getPrimalSolution(qpSol.data()) == qpOASES::SUCCESSFUL_RETURN;
  if (solved) {
    lastSolution_ = qpSol;
  } else {
    // getPrimalSolution() leaves qpSol untouched when the QP was not solved: hold the last solution instead.
    ++numQpFailures_;
    if (lastSolution_.size() != qpSol.size()) {
      lastSolution_.setZero(qpSol.size());
    }
    qpSol = lastSolution_;
  }

  lastCentroidalResidual_ = centroidalLinearA_ * qpSol - centroidalLinearB_;
  return qpSol;
}

vector_t OptiPessiWbc::update(const vector_t& /*stateDesired*/, const vector_t& /*inputDesired*/, const vector_t& /*rbdStateMeasured*/,
                              size_t /*mode*/, scalar_t /*period*/) {
  throw std::runtime_error("[OptiPessiWbc] tracks a WbcReference: call update(reference, rbdStateMeasured).");
}

Task OptiPessiWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
}

Task OptiPessiWbc::formulateSwingFootTask(const WbcReference& reference) {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  const std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  const std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

  const size_t nq = info_.generalizedCoordinatesNum;
  matrix_t a = matrix_t::Zero(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b = vector_t::Zero(a.rows());
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      const vector3_t accel = swingKp_ * (reference.footPosition[i] - posMeasured[i]) + swingKd_ * (reference.footVelocity[i] - velMeasured[i]);
      a.block(3 * j, 0, 3, nq) = j_.block(3 * i, 0, 3, nq);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, nq) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task OptiPessiWbc::formulateReferenceForceTask(const WbcReference& reference) const {
  matrix_t a = matrix_t::Zero(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3).setIdentity();
    b.segment<3>(3 * i) = reference.footForce[i];
  }

  return {a, b, matrix_t(), vector_t()};
}

Task OptiPessiWbc::formulateCentroidalTask(const WbcReference& reference) {
  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();
  const size_t nq = info_.generalizedCoordinatesNum;

  // Also fills data.Ag, data.com[0] and data.vcom[0] of the measured robot.
  const auto& dAg = pinocchio::dccrba(model, data, qMeasured_, vMeasured_);
  const vector3_t com = data.com[0];
  const vector3_t comVelocity = data.vcom[0];

  vector3_t comAcceleration = reference.comAcceleration;
  comAcceleration.head<2>() += comKpXY_ * (reference.comPosition.head<2>() - com.head<2>()) +
                               comKdXY_ * (reference.comVelocity.head<2>() - comVelocity.head<2>());
  comAcceleration.z() += comKpZ_ * (reference.comPosition.z() - com.z()) + comKdZ_ * (reference.comVelocity.z() - comVelocity.z());

  matrix_t a = matrix_t::Zero(6, numDecisionVars_);
  vector_t b(6);

  // Linear momentum rate m ddc = A_lin qdd + dA_lin v, divided by the mass so the rows are in m/s^2 like the rows below.
  a.block(0, 0, 3, nq) = data.Ag.topRows<3>() / info_.robotMass;
  b.head<3>() = comAcceleration - dAg.topRows<3>() * vMeasured_ / info_.robotMass;

  // Base orientation: rows 3..5 of qdd are the ZYX Euler accelerations [yaw, pitch, roll].
  a.block(3, 3, 3, 3).setIdentity();
  const scalar_t yawError = std::remainder(reference.yaw - qMeasured_(3), 2.0 * M_PI);
  b(3) = reference.yawAcceleration + yawKp_ * yawError + yawKd_ * (reference.yawRate - vMeasured_(3));
  b(4) = -rollPitchKp_ * qMeasured_(4) - rollPitchKd_ * vMeasured_(4);
  b(5) = -rollPitchKp_ * qMeasured_(5) - rollPitchKd_ * vMeasured_(5);

  centroidalLinearA_ = a.topRows(3);
  centroidalLinearB_ = b.head<3>();

  return {a, b, matrix_t(), vector_t()};
}

void OptiPessiWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### OptiPessi WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightCentroidal_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);

  prefix = "optiPessiWbc.";
  if (verbose) {
    std::cerr << "\n #### OptiPessi WBC centroidal task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, comKpXY_, prefix + "comKpXY", verbose);
  loadData::loadPtreeValue(pt, comKdXY_, prefix + "comKdXY", verbose);
  loadData::loadPtreeValue(pt, comKpZ_, prefix + "comKpZ", verbose);
  loadData::loadPtreeValue(pt, comKdZ_, prefix + "comKdZ", verbose);
  loadData::loadPtreeValue(pt, yawKp_, prefix + "yawKp", verbose);
  loadData::loadPtreeValue(pt, yawKd_, prefix + "yawKd", verbose);
  loadData::loadPtreeValue(pt, rollPitchKp_, prefix + "rollPitchKp", verbose);
  loadData::loadPtreeValue(pt, rollPitchKd_, prefix + "rollPitchKd", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
}

}  // namespace legged
