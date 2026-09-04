#pragma once

#include <cstddef>

#include "opti_pessi_wbc_bridge/LegIndexing.h"

namespace opti_pessi_bridge {

/**
 * Maps real elapsed time onto the Opti-Pessi plan's discrete contact phases.
 *
 * The OCP's "time" axis is the knot index, and the real duration of a phase is the decision
 * variable u(RobotU::DT). This class is the only place that conversion lives.
 */
class PhaseClock {
 public:
  void reset(double phaseDuration);

  /** Advances by period seconds. Returns true exactly when a phase boundary was crossed. */
  bool advance(double period);

  /** Sets the duration of the CURRENT phase, keeping elapsed time. Used when a new plan latches. */
  void setPhaseDuration(double phaseDuration);

  double tau() const { return tau_; }
  double phaseDuration() const { return phaseDuration_; }
  int phaseIndex() const { return phaseIndex_; }
  int parity() const { return phaseIndex_ % 2; }
  size_t modeNumber() const { return modeNumberForParity(parity()); }

  /** tau / phaseDuration, clamped to [0, 1]. Drives the swing arc. */
  double progress() const;

 private:
  double tau_{0.0};
  double phaseDuration_{0.25};
  int phaseIndex_{0};
};

}  // namespace opti_pessi_bridge
