#include "opti_pessi_wbc_bridge/PlanBuffer.h"

namespace opti_pessi_bridge {

void PlanBuffer::publish(const PhasePlan& plan) {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = plan;
  hasPending_ = true;
}

bool PlanBuffer::hasPending() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return hasPending_;
}

PhasePlan PlanBuffer::takePending() {
  std::lock_guard<std::mutex> lock(mutex_);
  hasPending_ = false;
  return pending_;
}

PhasePlan PlanBuffer::current() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return current_;
}

void PlanBuffer::setCurrent(const PhasePlan& plan) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_ = plan;
}

}  // namespace opti_pessi_bridge
