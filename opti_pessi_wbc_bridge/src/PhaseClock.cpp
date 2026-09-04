#include "opti_pessi_wbc_bridge/PhaseClock.h"

#include <algorithm>
#include <cmath>

namespace opti_pessi_bridge {

namespace {
constexpr double kMinPhaseDuration = 1e-6;
}

void PhaseClock::reset(double phaseDuration) {
  tau_ = 0.0;
  phaseDuration_ = std::max(phaseDuration, kMinPhaseDuration);
  phaseIndex_ = 0;
}

bool PhaseClock::advance(double period) {
  tau_ += period;
  if (tau_ < phaseDuration_) {
    return false;
  }
  // Carry the overshoot into the new phase so time is never lost or double counted.
  //
  // The crossing count is computed arithmetically rather than by subtracting once, so that an
  // advance spanning MORE than one phase lands on the correct phase index instead of silently
  // dropping the phases in between. That matters because parity() -- and therefore the stance pair
  // the whole-body controller is told to stand on -- is derived from phaseIndex_: skipping a phase
  // would hand the controller the wrong two feet with no diagnostic.
  //
  // This also self-corrects the state left behind when setPhaseDuration() shrinks the current phase
  // below the already-elapsed tau_: the next advance() rolls over as many phases as the new
  // duration implies. phaseDuration_ is floored at kMinPhaseDuration, so the division is safe.
  const double crossings = std::floor(tau_ / phaseDuration_);
  tau_ -= crossings * phaseDuration_;
  phaseIndex_ += static_cast<int>(crossings);
  return true;
}

void PhaseClock::setPhaseDuration(double phaseDuration) {
  phaseDuration_ = std::max(phaseDuration, kMinPhaseDuration);
}

double PhaseClock::progress() const {
  return std::min(1.0, std::max(0.0, tau_ / phaseDuration_));
}

}  // namespace opti_pessi_bridge
