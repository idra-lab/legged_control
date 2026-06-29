//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedWbc.h"

#include <qpOASES.hpp>

namespace legged {

vector_t WeightedWbc::update(const vector_t &stateDesired,
                             const vector_t &inputDesired,
                             const vector_t &rbdStateMeasured, size_t mode,
                             scalar_t period) {
  WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

  // Constraints
  Task constraints = formulateConstraints();
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_; // clang-format on

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
  // Add small regularisation: H = AᵀA is only semi-definite; without this
  // qpOASES reports "Projected Hessian not positive definite" at startup.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
      weighedTask.a_.transpose() * weighedTask.a_ +
      1e-6 * matrix_t::Identity(getNumDecisionVars(), getNumDecisionVars());
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 200;  // raised from 20: prevents "Max working-set recalcs" at startup

  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(),
                 ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());

  qpProblem.getPrimalSolution(qpSol.data());
  return qpSol;
}

Task WeightedWbc::formulateConstraints() {
  return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() +
         formulateFrictionConeTask() + formulateNoContactMotionTask();
}

Task WeightedWbc::formulateWeightedTasks(const vector_t &stateDesired,
                                         const vector_t &inputDesired,
                                         scalar_t period) {
  return formulateSwingLegTask() * weightSwingLeg_ +
         formulateBaseAccelTask(stateDesired, inputDesired, period) *
             weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_;
}

void WeightedWbc::loadTasksSetting(const std::string &taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce",
                           verbose);
}

} // namespace legged
