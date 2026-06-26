#pragma once

#include <legged_wbc/WbcBase.h>
#include <qpOASES.hpp>

namespace legged {

class OptiPessiWbc : public WbcBase {
public:
  OptiPessiWbc(const PinocchioInterface &pinocchioInterface,
               CentroidalModelInfo info,
               const PinocchioEndEffectorKinematics &eeKinematics);

  vector_t update(const vector_t &stateDesired, const vector_t &inputDesired,
                  const vector_t &rbdStateMeasured, size_t mode,
                  scalar_t period) override;

  void loadTasksSetting(const std::string &taskFile, bool verbose) override;

  // Setters for MPC inputs and swing foot targets (called by the control node)
  void setMpcInput(const vector_t &mpcInput);
  void setSwingFootTargets(const std::vector<vector3_t> &posDesired,
                           const std::vector<vector3_t> &velDesired);

  vector_t getGravityCompensation(const vector_t &rbdStateMeasured);

protected:
  // Override the 3 custom tasks
  Task formulateBaseAccelTask(const vector_t &stateDesired,
                              const vector_t &inputDesired, scalar_t period);
  Task formulateSwingLegTask();
  Task formulateContactForceTask(const vector_t &inputDesired) const;

  // Helper for constraints formulation
  Task formulateConstraints();
  Task formulateWeightedTasks(const vector_t &stateDesired,
                              const vector_t &inputDesired, scalar_t period);

private:
  // Weights for tasks
  scalar_t weightSwingLeg_ = 200.0;
  scalar_t weightBaseAccel_ = 10.0;
  scalar_t weightContactForce_ = 1e-3;

  // PD gains for base height and orientation (since they are not planned by
  // MPC)
  scalar_t kp_z_ = 100.0;
  scalar_t kd_z_ = 10.0;
  scalar_t kp_roll_ = 100.0;
  scalar_t kd_roll_ = 10.0;
  scalar_t kp_pitch_ = 100.0;
  scalar_t kd_pitch_ = 10.0;
  scalar_t kp_yaw_ = 100.0;
  scalar_t kd_yaw_ = 10.0;

  // Nominal CoM height reference
  scalar_t z_nom_ = 0.30;

  // Swing foot references from setter
  std::vector<vector3_t> swingPosDesired_;
  std::vector<vector3_t> swingVelDesired_;

  // Raw MPC input [p0_x, p0_y, p1_x, p1_y, alpha, ...]
  vector_t mpcInput_;
};

} // namespace legged
