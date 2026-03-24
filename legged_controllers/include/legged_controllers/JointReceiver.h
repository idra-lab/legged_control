/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>

namespace ocs2 {
namespace legged_robot {

class JointReceiver {
 public:
  JointReceiver(::ros::NodeHandle nodeHandle);
  std::vector<float> getJointPositions() { return receivedPos_; }
  std::vector<float> getJointVelocities() { return receivedVel_; }
  std::vector<float> getJointEfforts() { return receivedEff_; }
  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

 private:
  
  ros::Subscriber jointSubscriber_;
  std::shared_ptr<bool> jointPtr_;

  std::mutex receivedJointMutex_;
  std::vector<float> receivedPos_ = std::vector<float>(13);
  std::vector<float> receivedVel_ = std::vector<float>(13);
  std::vector<float> receivedEff_ = std::vector<float>(13);
};

}  // namespace legged_robot
}  // namespace ocs2
