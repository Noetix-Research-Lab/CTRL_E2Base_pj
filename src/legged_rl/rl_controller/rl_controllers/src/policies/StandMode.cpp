#include "rl_controllers/AcController.h"

namespace legged
{
namespace
{
bool isLowDampingJoint(const std::string& jointName)
{
  return jointName == "l_leg_ankle_pitch_joint" || jointName == "r_leg_ankle_pitch_joint" ||
         jointName == "l_leg_ankle_roll_joint" || jointName == "r_leg_ankle_roll_joint";
}

void setTransitionCommand(const std::string& jointName,
                          HybridJointHandle& joint, scalar_t position)
{
  if (jointName == "waist_roll_joint" || jointName == "waist_pitch_joint") {
    joint.setCommand(position, 0, 200, 5, 0);
  } else if (isLowDampingJoint(jointName)) {
    joint.setCommand(position, 0, 50, 1, 0);
  } else {
    joint.setCommand(position, 0, 50, 5, 0);
  }
}
}  // namespace

void RLControllerBase::handleStandMode()
{
  const scalar_t percent = standTransitionPercent();
  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    const scalar_t desired =
        currentJointAngles_[i] * (1 - percent) + standJointAngles_[i] * percent;
    setTransitionCommand(jointNames_[i], hybridJointHandles_[i], desired);
  }
}

void AcController::handleStandMode()
{
  leavePolicyMode();
  endCsvPlaybackSession();
  RLControllerBase::handleStandMode();
}
}  // namespace legged
