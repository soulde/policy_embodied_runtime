#include <array>
#include <cstdint>

#include <gtest/gtest.h>

#include "policy_runtime/protocol/cia402/state_machine.hpp"

namespace {

using policy_runtime::AxisRequest;
using policy_runtime::Cia402StateMachine;
using policy_runtime::DriveState;

struct StateCase {
  std::uint16_t status_word;
  DriveState expected;
};

constexpr std::array<StateCase, 9> kStateCases{{
    {0x0000, DriveState::not_ready_to_switch_on},
    {0x0040, DriveState::switch_on_disabled},
    {0x0021, DriveState::ready_to_switch_on},
    {0x0023, DriveState::switched_on},
    {0x0027, DriveState::operation_enabled},
    {0x0007, DriveState::quick_stop_active},
    {0x000F, DriveState::fault_reaction_active},
    {0x0008, DriveState::fault},
    {0x006F, DriveState::unknown},
}};

struct TransitionCase {
  std::uint16_t status_word;
  AxisRequest request;
  std::uint16_t expected_control_word;
  DriveState expected_state;
};

constexpr std::array<TransitionCase, 17> kTransitionCases{{
    {0x0040, AxisRequest::enable, 0x0006, DriveState::switch_on_disabled},
    {0x0021, AxisRequest::enable, 0x0007, DriveState::ready_to_switch_on},
    {0x0023, AxisRequest::enable, 0x000F, DriveState::switched_on},
    {0x0027, AxisRequest::enable, 0x000F, DriveState::operation_enabled},
    {0x0007, AxisRequest::enable, 0x000F, DriveState::quick_stop_active},
    {0x0000, AxisRequest::enable, 0x0000, DriveState::not_ready_to_switch_on},
    {0x000F, AxisRequest::enable, 0x0000, DriveState::fault_reaction_active},
    {0x0008, AxisRequest::enable, 0x0000, DriveState::fault},
    {0x0027, AxisRequest::quick_stop, 0x0002, DriveState::operation_enabled},
    {0x0007, AxisRequest::quick_stop, 0x0002, DriveState::quick_stop_active},
    {0x0040, AxisRequest::quick_stop, 0x0000, DriveState::switch_on_disabled},
    {0x0027, AxisRequest::disable, 0x0000, DriveState::operation_enabled},
    {0x0023, AxisRequest::disable, 0x0000, DriveState::switched_on},
    {0x0008, AxisRequest::fault_reset, 0x0080, DriveState::fault},
    {0x0027, AxisRequest::fault_reset, 0x0000, DriveState::operation_enabled},
    {0x000F, AxisRequest::fault_reset, 0x0000, DriveState::fault_reaction_active},
    {0x006F, AxisRequest::enable, 0x0000, DriveState::unknown},
}};

}  // namespace

TEST(Cia402StateMachineTest, DecodesEveryDriveStateWithUnrelatedBitsSet) {
  for (const auto& test_case : kStateCases) {
    EXPECT_EQ(Cia402StateMachine::decode(test_case.status_word | 0x3F90U),
              test_case.expected)
        << "status word: " << test_case.status_word;
  }
}

TEST(Cia402StateMachineTest, GeneratesSafeControlWordsForEveryRequestPath) {
  Cia402StateMachine state_machine;
  for (const auto& test_case : kTransitionCases) {
    const auto output = state_machine.update(test_case.status_word, test_case.request);
    EXPECT_EQ(output.state, test_case.expected_state)
        << "status word: " << test_case.status_word;
    EXPECT_EQ(output.control_word, test_case.expected_control_word)
        << "status word: " << test_case.status_word;
  }
}

TEST(Cia402StateMachineTest, PulsesFaultResetOnlyAfterTheRequestEdge) {
  Cia402StateMachine state_machine;

  EXPECT_EQ(state_machine.update(0x0008, AxisRequest::fault_reset).control_word, 0x0080);
  EXPECT_EQ(state_machine.update(0x0008, AxisRequest::fault_reset).control_word, 0x0000);
  EXPECT_EQ(state_machine.update(0x0027, AxisRequest::disable).control_word, 0x0000);
  EXPECT_EQ(state_machine.update(0x0008, AxisRequest::fault_reset).control_word, 0x0080);
}
