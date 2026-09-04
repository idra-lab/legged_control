//
// OptiPessiController: drives legged_wbc from an Opti-Pessi LIP plan through
// opti_pessi_wbc_bridge, instead of from an OCS2 nonlinear-MPC policy the way LeggedController
// does. See task-12-brief.md for the design this mirrors.
//
// This controller deliberately has NO LeggedInterface and NO MPC_MRT_Interface: it builds its own
// PinocchioInterface / CentroidalModelInfo directly from the URDF (the same factory functions
// LeggedInterface uses internally), and gets its reference each tick from
// opti_pessi_wbc_bridge::synthesize() instead of MPC_MRT_Interface::evaluatePolicy(). Task 12 stops
// short of the background solver thread: on_activate() seeds a fixed standing PhasePlan so the
// loop can be exercised end-to-end before Task 13 attaches a real solver.
//

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_wbc/WbcBase.h>

#include <ocs2_ipm/IpmSolver.h>
#include <opti_pessi_interface/OptiPessiInterface.h>
#include <opti_pessi_wbc_bridge/LegKinematics.h>
#include <opti_pessi_wbc_bridge/PhaseClock.h>
#include <opti_pessi_wbc_bridge/PlanBuffer.h>
#include <opti_pessi_wbc_bridge/ReferenceSynthesizer.h>
#include <opti_pessi_wbc_bridge/StateFolder.h>

#include "legged_controllers/SafetyChecker.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

/**
 * Hardware-interface handle structs. These are COPIED from legged_controllers/LeggedController.h
 * verbatim -- ros2_control hardware-interface plumbing shared by every controller in this package,
 * not something to design differently per controller (see task-12-brief.md). LeggedController.h is
 * off-limits to edit (the RULING scopes the allowed LeggedController.cpp edit to calling the
 * extracted HardwareCommandWriter helper only), so this is a duplicate definition rather than a
 * shared one; HardwareCommandWriter.h is templated on the handle type precisely so that does not
 * matter.
 */
struct HybridJointHandle {
  hardware_interface::LoanedStateInterface* positionState{nullptr};
  hardware_interface::LoanedStateInterface* velocityState{nullptr};
  hardware_interface::LoanedStateInterface* effortState{nullptr};
  hardware_interface::LoanedCommandInterface* positionCmd{nullptr};
  hardware_interface::LoanedCommandInterface* velocityCmd{nullptr};
  hardware_interface::LoanedCommandInterface* kpCmd{nullptr};
  hardware_interface::LoanedCommandInterface* kdCmd{nullptr};
  hardware_interface::LoanedCommandInterface* effortCmd{nullptr};

  double getPosition() const { return positionState->get_value(); }
  double getVelocity() const { return velocityState->get_value(); }
  double getEffort() const { return effortState->get_value(); }
  void setCommand(double pos, double vel, double kp, double kd, double effort) {
    positionCmd->set_value(pos);
    velocityCmd->set_value(vel);
    kpCmd->set_value(kp);
    kdCmd->set_value(kd);
    effortCmd->set_value(effort);
  }
};

struct ImuSensorHandle {
  std::vector<hardware_interface::LoanedStateInterface*> orientation;         // size 4
  std::vector<hardware_interface::LoanedStateInterface*> angularVelocity;     // size 3
  std::vector<hardware_interface::LoanedStateInterface*> linearAcceleration;  // size 3

  const double* getOrientation() const {
    static double ori[4];
    for (size_t i = 0; i < 4; ++i) ori[i] = orientation[i]->get_value();
    return ori;
  }
  const double* getAngularVelocity() const {
    static double ang[3];
    for (size_t i = 0; i < 3; ++i) ang[i] = angularVelocity[i]->get_value();
    return ang;
  }
  const double* getLinearAcceleration() const {
    static double lin[3];
    for (size_t i = 0; i < 3; ++i) lin[i] = linearAcceleration[i]->get_value();
    return lin;
  }
  const double* getOrientationCovariance() const {
    static const double cov[9] = {0};
    return cov;
  }
  const double* getAngularVelocityCovariance() const {
    static const double cov[9] = {0};
    return cov;
  }
  const double* getLinearAccelerationCovariance() const {
    static const double cov[9] = {0};
    return cov;
  }
};

struct ContactSensorHandle {
  hardware_interface::LoanedStateInterface* contactState{nullptr};
  bool isContact() const {
    if (contactState) {
      return contactState->get_value() > 0.5;
    }
    return false;
  }
};

class OptiPessiController : public controller_interface::ControllerInterface {
 public:
  OptiPessiController() = default;
  ~OptiPessiController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

 protected:
  void updateStateEstimation(const rclcpp::Time& time, const rclcpp::Duration& period);

  // --- Background solver thread (Task 13) -------------------------------------------------------
  /** Solver-thread body: wait for a request, copy it out, solve OUTSIDE the lock, publish. */
  void solverThreadLoop();
  /** Runs one gated Opti-Pessi solve from `initialState` (a 17-dof LIP state) for `phaseIndex`.
   *  Called ONLY from solverThread_. On a sane solve, publishes that solve's applied input; on a
   *  failed/insane solve, falls back to the failed solve's own saturated input, and if that is
   *  itself unusable, to the capture-point (deadbeat) stop -- mirroring
   *  opti_pessi_interface/src/simulation/ClosedLoopSimulation.cpp's extractSolve / isInsane /
   *  fallbackInput. See OptiPessiController.cpp for why those could not be reused directly. */
  opti_pessi_bridge::PhasePlan solveOnePhase(const ocs2::vector_t& initialState, int phaseIndex);
  /** Signals solverThread_ to exit and joins it. Idempotent: safe to call from on_deactivate() and
   *  again from the destructor (e.g. if on_deactivate was never reached). */
  void stopSolverThread();

  // Robot model, built directly from the URDF (the same factory functions
  // legged_interface::LeggedInterface uses internally) -- no LeggedInterface, no
  // OptimalControlProblem for the quadruped's own nonlinear MPC.
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

  // Opti-Pessi OCP interface. Built in on_init() so modelParameters() (comHeight, mass, dtCost0,
  // ...) is available for the standing plan seeded in on_activate(); Task 13 is what actually runs
  // it on a background solver thread and publishes real plans through planBuffer_.
  std::shared_ptr<opti_pessi::OptiPessiInterface> interface_;

  // Whole body control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // State estimation
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
  vector_t measuredRbdState_;
  vector_t centroidalState_;
  vector_t centroidalInput_;

  // Hardware handles (ros2_control plumbing, see the structs above)
  std::vector<HybridJointHandle> hybridJointHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  ImuSensorHandle imuSensorHandle_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr contactSub_;
  contact_flag_t topicContacts_{};

  // Helper ROS 2 node: KalmanFilterEstimate needs a spinning node for its tf listener and odometry
  // subscription, exactly as LeggedController provides one for its own ROS 2 communication.
  rclcpp::Node::SharedPtr ros2_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spinThread_;

  // Opti-Pessi bridge
  opti_pessi_bridge::LegGeometry legGeometry_;
  opti_pessi_bridge::SynthesisSettings synthesisSettings_;
  opti_pessi_bridge::PhaseClock phaseClock_;
  opti_pessi_bridge::PlanBuffer planBuffer_;
  opti_pessi_bridge::FoldedHistory history_;

  // --- Background solver thread (Task 13) ---------------------------------------------------------
  //
  // Ownership / thread-safety map:
  //
  //  * solver_ is constructed once, in on_init(), on the ros2_control lifecycle thread, BEFORE
  //    solverThread_ ever runs. After that point it is touched ONLY from solverThread_ (inside
  //    solveOnePhase()/solverThreadLoop()). Thread creation is a happens-before edge, so no mutex is
  //    needed for solver_ itself -- there is no second thread that ever calls into it.
  //
  //  * solveInputMutex_ guards exactly solveInitialState_ and solvePhaseIndex_: update() (the
  //    control thread) writes them at a phase boundary, solverThreadLoop() (the solver thread)
  //    reads/copies them out at the top of its loop. The lock is held only for that copy, NEVER
  //    across the solve itself, so the control thread's critical section is O(copy a 17-vector) and
  //    can never be blocked behind a ~1s solve.
  //
  //  * solveRequested_ and solverRunning_ are the condition-variable predicate, checked together
  //    with solveInputMutex_ held (via solveCv_.wait's predicate form) so a notify can never be
  //    missed between the control thread's write and the solver thread's wait.
  //
  //  * planBuffer_ is PlanBuffer's own internal mutex (SINGLE READER by its own contract): the
  //    solver thread only ever calls publish(); the control thread only ever calls
  //    hasPending()/takePending()/current()/setCurrent(). Task 13 does not change that split.
  //
  // No code path holds solveInputMutex_ while calling into planBuffer_ or the solver, and no code
  // path holds two of these mutexes at once, so there is no lock-ordering cycle and hence no
  // deadlock.
  std::unique_ptr<ocs2::IpmSolver> solver_;
  std::thread solverThread_;
  std::atomic<bool> solverRunning_{false};  // false requests solverThreadLoop() to exit
  std::atomic<bool> solveRequested_{false}; // true: a solve request from update() is waiting to be picked up
  std::mutex solveInputMutex_;              // guards solveInitialState_, solvePhaseIndex_, and the cv predicate
  std::condition_variable solveCv_;
  ocs2::vector_t solveInitialState_;  // 17-dof LIP state, guarded by solveInputMutex_
  int solvePhaseIndex_{0};            // guarded by solveInputMutex_
};

}  // namespace legged
