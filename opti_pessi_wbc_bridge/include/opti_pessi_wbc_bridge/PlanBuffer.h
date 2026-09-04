#pragma once

#include <mutex>

#include <ocs2_core/Types.h>

#include "opti_pessi_interface/definitions.h"

namespace opti_pessi_bridge {

/** One contact phase of the plan: the optimistic branch, already extracted from the augmented vectors. */
struct PhasePlan {
  ocs2::vector_t robotState;  // RobotX, 17
  ocs2::vector_t robotInput;  // RobotU, 8
  int phaseIndex{0};
  bool valid{false};
};

/**
 * Hand-off between the solver thread (writer) and the control thread (reader).
 *
 * The reader consumes a pending plan only at a phase boundary, so a plan is never swapped in
 * mid-phase and the synthesized reference stays continuous. A plan that is superseded before it is
 * consumed is dropped: the newest plan is always the right one.
 *
 * SINGLE READER. Exactly one thread may call hasPending()/takePending(). Each accessor is
 * individually locked, but hasPending() followed by takePending() is a check-then-act across two
 * separate critical sections: with a second reader the two could interleave, so one caller would
 * consume a plan the other had already claimed. The control thread is that single reader. Adding a
 * second one requires a combined take-if-pending operation, not just more locking here.
 */
class PlanBuffer {
 public:
  void publish(const PhasePlan& plan);
  bool hasPending() const;

  /**
   * Removes and returns the pending plan.
   *
   * UNDEFINED WHEN EMPTY in the sense that the value is not meaningful: with nothing ever
   * published it returns a default-constructed plan (valid == false), and called twice without an
   * intervening publish it returns the same already-consumed plan again, with no signal that it is
   * stale. Callers must gate on hasPending(); the `valid` flag is not a substitute.
   */
  PhasePlan takePending();

  PhasePlan current() const;
  void setCurrent(const PhasePlan& plan);

 private:
  mutable std::mutex mutex_;
  PhasePlan pending_;
  PhasePlan current_;
  bool hasPending_{false};
};

}  // namespace opti_pessi_bridge
