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

#include "legged_controllers/JointReceiver.h"

namespace ocs2 {
namespace legged_robot {
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
JointReceiver::JointReceiver(ros::NodeHandle nodeHandle)
{
  jointSubscriber_ = nodeHandle.subscribe("/joints_backup", 1, &JointReceiver::jointCallback, this);                                                    
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void JointReceiver::jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(receivedJointMutex_);
    // assuming the joint state message has already 12 elements allocated
    for (std::size_t i(0); i < 13; ++i){
	  receivedPos_[i] = msg->position[i];
	  receivedVel_[i] = msg->velocity[i]; // this is normally zero
	  receivedEff_[i] = msg->effort[i]; // this is normally fixed


  }
}

}  // namespace legged_robot
}  // end of namespace ocs2
