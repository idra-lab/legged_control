#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>  // completes pinocchio::ModelTpl so model.names is usable below.

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/ModelSettings.h>

#include "opti_pessi_wbc_bridge/LegIndexing.h"

using namespace opti_pessi_bridge;

namespace {
// Set by the CMake-provided compile definition; see CMakeLists.
const std::string kUrdfPath = OPTI_PESSI_BRIDGE_TEST_URDF;
}  // namespace

TEST(ModelOrdering, pinocchioJointOrderIsLfLhRfRh) {
  const auto interface = ocs2::centroidal_model::createPinocchioInterface(
      kUrdfPath, ocs2::legged_robot::ModelSettings().jointNames);
  const auto& model = interface.getModel();

  // Actuated joints follow the 6-dof floating base, so joint names start at index 2 in Pinocchio's
  // names vector (index 0 is "universe", index 1 is the root joint).
  std::vector<std::string> actuated;
  for (size_t i = 2; i < model.names.size(); ++i) {
    actuated.push_back(model.names[i]);
  }
  ASSERT_EQ(actuated.size(), 12u);

  // Joint block b covers actuated[3b .. 3b+2]. Assert each block belongs to the expected leg.
  const std::vector<std::string> expectedLegPrefixPerBlock{"LF", "LH", "RF", "RH"};
  for (size_t b = 0; b < 4; ++b) {
    for (size_t j = 0; j < 3; ++j) {
      const std::string& name = actuated[3 * b + j];
      EXPECT_EQ(name.substr(0, 2), expectedLegPrefixPerBlock[b])
          << "joint block " << b << " entry " << j << " is " << name;
    }
  }

  // And that agrees with jointBlockOf.
  EXPECT_EQ(actuated[3 * jointBlockOf(opti_pessi::Foot::FL)].substr(0, 2), "LF");
  EXPECT_EQ(actuated[3 * jointBlockOf(opti_pessi::Foot::RL)].substr(0, 2), "LH");
  EXPECT_EQ(actuated[3 * jointBlockOf(opti_pessi::Foot::FR)].substr(0, 2), "RF");
  EXPECT_EQ(actuated[3 * jointBlockOf(opti_pessi::Foot::RR)].substr(0, 2), "RH");
}

TEST(ModelOrdering, contactFrameOrderIsLfRfLhRh) {
  const ocs2::legged_robot::ModelSettings settings;
  ASSERT_EQ(settings.contactNames3DoF.size(), 4u);
  EXPECT_EQ(settings.contactNames3DoF[contactIndexOf(opti_pessi::Foot::FL)], "LF_FOOT");
  EXPECT_EQ(settings.contactNames3DoF[contactIndexOf(opti_pessi::Foot::FR)], "RF_FOOT");
  EXPECT_EQ(settings.contactNames3DoF[contactIndexOf(opti_pessi::Foot::RL)], "LH_FOOT");
  EXPECT_EQ(settings.contactNames3DoF[contactIndexOf(opti_pessi::Foot::RR)], "RH_FOOT");
}
