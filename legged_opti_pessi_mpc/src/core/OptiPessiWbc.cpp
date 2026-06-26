#include "opti_pessi_control/OptiPessiWbc.h"
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace legged {

OptiPessiWbc::OptiPessiWbc(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : WbcBase(pinocchioInterface, std::move(info), eeKinematics) {
  // Initialize swing foot targets to nominal or zero
  swingPosDesired_.resize(info_.numThreeDofContacts, vector3_t::Zero());
  swingVelDesired_.resize(info_.numThreeDofContacts, vector3_t::Zero());
  mpcInput_ = vector_t::Zero(5);
  mpcInput_[4] = 0.5; // Default alpha
}

void OptiPessiWbc::setMpcInput(const vector_t& mpcInput) {
  mpcInput_ = mpcInput;
}

void OptiPessiWbc::setSwingFootTargets(const std::vector<vector3_t>& posDesired, const std::vector<vector3_t>& velDesired) {
  swingPosDesired_ = posDesired;
  swingVelDesired_ = velDesired;
}

vector_t OptiPessiWbc::getGravityCompensation(const vector_t& rbdStateMeasured) {
  updateMeasured(rbdStateMeasured);
  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();
  
  // Compute generalized gravity vector by calling nonLinearEffects with zero joint velocity
  vector_t vZero = vector_t::Zero(model.nv);
  pinocchio::nonLinearEffects(model, data, qMeasured_, vZero);
  
  return data.nle.tail(info_.actuatedDofNum);
}

vector_t OptiPessiWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                              scalar_t period) {
  // Update state variables (qMeasured_, vMeasured_, contactFlag_)
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // Constraints
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);
  
  A.block(0, 0, constraints.a_.rows(), getNumDecisionVars()) = constraints.a_;
  A.block(constraints.a_.rows(), 0, constraints.d_.rows(), getNumDecisionVars()) = constraints.d_;

  lbA.segment(0, constraints.b_.size()) = constraints.b_;
  lbA.segment(constraints.b_.size(), constraints.f_.size()) = -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  
  ubA.segment(0, constraints.b_.size()) = constraints.b_;
  ubA.segment(constraints.b_.size(), constraints.f_.size()) = constraints.f_;

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve using qpOASES
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 50;

  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol = vector_t::Zero(getNumDecisionVars());

  qpProblem.getPrimalSolution(qpSol.data());

  // Safety clamp: limit joint torques (last 12 elements of qpSol) to ±25 Nm (Go2 limit).
  // If QP diverges or initial state is far from feasible the raw solution can be
  // arbitrarily large and will flip/destroy the robot in simulation.
  constexpr double MAX_TORQUE = 25.0;  // Nm — Go2 peak joint torque
  int torqueOffset = static_cast<int>(qpSol.size()) - 12;
  if (torqueOffset >= 0) {
    for (int i = torqueOffset; i < static_cast<int>(qpSol.size()); ++i) {
      qpSol[i] = std::clamp(qpSol[i], -MAX_TORQUE, MAX_TORQUE);
    }
  }

  return qpSol;
}

Task OptiPessiWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
}

Task OptiPessiWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ + 
         formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_;
}

Task OptiPessiWbc::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& /*inputDesired*/, scalar_t /*period*/) {
  matrix_t a(6, numDecisionVars_);
  a.setZero();
  a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

  vector_t b = vector_t::Zero(6);

  double p0_x  = (mpcInput_.size() > 0) ? mpcInput_[0] : 0.0;
  double p0_y  = (mpcInput_.size() > 1) ? mpcInput_[1] : 0.0;
  double p1_x  = (mpcInput_.size() > 2) ? mpcInput_[2] : 0.0;
  double p1_y  = (mpcInput_.size() > 3) ? mpcInput_[3] : 0.0;
  double alpha = (mpcInput_.size() > 4) ? mpcInput_[4] : 0.5;

  double cop_x = p0_x + alpha * (p1_x - p0_x);
  double cop_y = p0_y + alpha * (p1_y - p0_y);

  // CoM desiderato dal vettore stateDesired (indici 9 e 10)
  double des_com_x = (stateDesired.size() > 9)  ? stateDesired[9]  : 0.0;
  double des_com_y = (stateDesired.size() > 10) ? stateDesired[10] : 0.0;

  double omega_sq = 9.81 / z_nom_;

  // LIPM: accelerazione = omega^2 * (errore CoM - errore CoP rispetto al CoM des)
  double com_err_x = qMeasured_[0] - des_com_x;
  double com_err_y = qMeasured_[1] - des_com_y;
  b[0] = omega_sq * (com_err_x - (cop_x - des_com_x));
  b[1] = omega_sq * (com_err_y - (cop_y - des_com_y));

  // Controllo z
  b[2] = kp_z_ * (z_nom_ - qMeasured_[2]) + kd_z_ * (0.0 - vMeasured_[2]);

  // Controllo orientazione
  double yaw_des  = stateDesired[6];
  double yaw_meas = qMeasured_[3];
  double yaw_err  = std::atan2(std::sin(yaw_des - yaw_meas), std::cos(yaw_des - yaw_meas));

  b[3] = kp_yaw_   * yaw_err               + kd_yaw_   * (0.0 - vMeasured_[3]);
  b[4] = kp_pitch_ * (0.0 - qMeasured_[4]) + kd_pitch_ * (0.0 - vMeasured_[4]);
  b[5] = kp_roll_  * (0.0 - qMeasured_[5]) + kd_roll_  * (0.0 - vMeasured_[5]);

  return {a, b, matrix_t(), vector_t()};
}

Task OptiPessiWbc::formulateSwingLegTask() {
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();

  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      vector3_t posTarget = (swingPosDesired_.size() > i) ? swingPosDesired_[i] : posMeasured[i];
      vector3_t velTarget = (swingVelDesired_.size() > i) ? swingVelDesired_[i] : vector3_t::Zero();

      vector3_t accel = swingKp_ * (posTarget - posMeasured[i]) + swingKd_ * (velTarget - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task OptiPessiWbc::formulateContactForceTask(const vector_t& /*inputDesired*/) const {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }

  // Synthesize reference contact forces
  double mass = info_.robotMass;
  double g = 9.81;
  double F_total = mass * g;
  double alpha = (mpcInput_.size() > 4) ? mpcInput_[4] : 0.5;
  // w0 = weight for front pair (FL=0, FR=1), w1 = weight for rear pair (RL=2, RR=3)
  double w0 = 1.0 - alpha;  // front
  double w1 = alpha;         // rear

  double sum_w = 0.0;
  if (contactFlag_[0]) sum_w += w0;  // FL: front
  if (contactFlag_[1]) sum_w += w0;  // FR: front
  if (contactFlag_[2]) sum_w += w1;  // RL: rear
  if (contactFlag_[3]) sum_w += w1;  // RR: rear

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      // Front pair (FL=0, FR=1) gets w0, rear pair (RL=2, RR=3) gets w1
      double w = (i <= 1) ? w0 : w1;
      double Fz = 0.0;
      if (sum_w > 1e-3) {
        Fz = (w / sum_w) * F_total;
      } else if (numContacts_ > 0) {
        Fz = F_total / numContacts_;
      }
      b[3 * i + 2] = Fz;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

void OptiPessiWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Call base loader
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_info(taskFile, pt);
    
    std::string prefix = "weight.";
    loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
    loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
    loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);

    prefix = "optiPessiWbc.";
    loadData::loadPtreeValue(pt, kp_z_, prefix + "kp_z", verbose);
    loadData::loadPtreeValue(pt, kd_z_, prefix + "kd_z", verbose);
    loadData::loadPtreeValue(pt, kp_roll_, prefix + "kp_roll", verbose);
    loadData::loadPtreeValue(pt, kd_roll_, prefix + "kd_roll", verbose);
    loadData::loadPtreeValue(pt, kp_pitch_, prefix + "kp_pitch", verbose);
    loadData::loadPtreeValue(pt, kd_pitch_, prefix + "kd_pitch", verbose);
    loadData::loadPtreeValue(pt, kp_yaw_, prefix + "kp_yaw", verbose);
    loadData::loadPtreeValue(pt, kd_yaw_, prefix + "kd_yaw", verbose);
    loadData::loadPtreeValue(pt, z_nom_, prefix + "z_nom", verbose);
  } catch (const std::exception& e) {
    if (verbose) {
      std::cerr << "Warning: OptiPessiWbc using default gains due to config reading error: " << e.what() << std::endl;
    }
  }
}

}  // namespace legged
