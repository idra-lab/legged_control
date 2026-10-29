#pragma once
#include <array>
#include <vector>
#include <algorithm>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>


namespace legged {

// Indeces of the feet in OCS2: {LF=0, RF=1, LH=2, RH=3}
class GaitPlanner{

public:
    using Pair = std::array<size_t, 2>; // {p0, p1}

    GaitPlanner(double off_x, double off_y, double nominal_step_duration)
        : step_duration_(nominal_step_duration) {
            hip_offsets_[0] = { off_x, off_y}; // LF
            hip_offsets_[1] = { off_x, -off_y}; // RF
            hip_offsets_[2] = { -off_x, off_y}; // LH
            hip_offsets_[3] = { -off_x, -off_y}; // RH
        }

    // Current support feet (p0 and p1 of x0) and swing feet
    const Pair& stancePair() const { return trot_pairs_[stance_]; }
    const Pair& swingPair() const { return trot_pairs_[1 - stance_]; }

    // FIX: added (was used but not defined). Pair placed at node k (k=0 -> swing feet now)
    const Pair& paramPair(int k) const { return trot_pairs_[(1 - stance_ + k) % 2]; }


    // Offset of the pair that will go into the paramters at node k (k=0 -> swing feet now)
    std::vector<double> hipOffsetParams(int k) const {
        const Pair& pair = paramPair(k);
        // FIX: second foot used y twice, now [x, y]
        return {hip_offsets_[pair[0]][0], hip_offsets_[pair[0]][1],
                hip_offsets_[pair[1]][0], hip_offsets_[pair[1]][1]};
    }


    // To be called at each cycle of update(): true if switch support feet
    bool advance(double dt){
        t_phase_ += dt;
        if (t_phase_ < step_duration_) return false;
        t_phase_ = 0.0;
        stance_ = 1 - stance_; // FIX: stance -> stance_
        step_index_++;
        return true;
    }

    void setStepDuration(double T){ step_duration_ = T; }
    // WARM START: number of support switches so far (the MPC shifts the previous solution only when it changes)
    size_t stepIndex() const { return step_index_; }
    double timeLeft() const { return std::max(step_duration_ - t_phase_, 0.0); }



    // Mode number for the WBC
    size_t mode() const {
        // FIX: types/functions live in ocs2::legged_robot
        ocs2::legged_robot::contact_flag_t contact{false, false, false, false};
        contact[stancePair()[0]] = true;
        contact[stancePair()[1]] = true;
        return ocs2::legged_robot::stanceLeg2ModeNumber(contact);
    }


private:
    // In the python there was: [["FR", "RL"], ["FL", "RR"]]
    // FIX: removed const, otherwise the copy assignment (gait_local = *gait_planner_) is deleted
    std::array<Pair, 2> trot_pairs_{{{1, 2}, {0, 3}}};
    std::array<std::array<double, 2>, 4> hip_offsets_;
    int stance_ = 1; // Support on LF-RH, hence params at node 0 are RF-LH
    double t_phase_ = 0.0;
    double step_duration_;
    size_t step_index_ = 0;

}; // FIX: missing ';' at the end of the class

} // namespace legged
