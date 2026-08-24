#pragma once

#include <string>
#include <string_view>

#include "policy_runtime/common/result.hpp"
#include "policy_runtime/protocol/rpc/messages.hpp"

namespace policy_runtime::rpc {

Result<void> validate_action_payload(const nlohmann::json& action);
Result<void> validate_payload(std::string_view type,
                              const nlohmann::json& payload);
Result<MessageEnvelope> decode_envelope(std::string_view text);
Result<MessageEnvelope> decode_envelope_for_dispatch(std::string_view text);
std::string encode_envelope(const MessageEnvelope& envelope);

}  // namespace policy_runtime::rpc
