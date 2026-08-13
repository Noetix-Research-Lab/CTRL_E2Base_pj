#include "rl_controllers/AcController.h"

namespace legged
{
void RLControllerBase::handleDefaultMode()
{
  for (HybridJointHandle& joint : hybridJointHandles_) {
    joint.setCommand(0, 0, 0, 0.1, 0);
  }
}

void AcController::handleDefaultMode()
{
  leavePolicyMode();
  RLControllerBase::handleDefaultMode();
}
}  // namespace legged
