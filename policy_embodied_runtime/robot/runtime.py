"""Robot-layer runtime orchestration."""

from __future__ import annotations

from typing import Any

from policy_embodied_runtime.adapters.base import BaseEmbodimentAdapter
from policy_embodied_runtime.robot.session_manager import SessionManager
from policy_embodied_runtime.robot.data import RobotAction, RobotData, RobotObservation
from policy_embodied_runtime.robot.devices.rpc import RpcActionActuator, RpcObservationSensor
from policy_embodied_runtime.robot.policy import Policy
from policy_embodied_runtime.postprocess.pipeline import build_postprocessors, run_postprocessors
from policy_embodied_runtime.preprocess.pipeline import build_preprocessors, run_preprocessors
from policy_embodied_runtime.safety.checks import validate_required_fields
from policy_embodied_runtime.protocol.embodiment_profile import EmbodimentProfile
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


class EmbodiedRobotRuntime:
    """Coordinate embodiment mapping, policy inference, and action mapping.

    This layer intentionally works with canonical robot observations/actions.
    Protocol envelopes and ZMQ socket concerns stay outside this class.
    """

    def __init__(
        self,
        *,
        policy_profile: PolicyProfile,
        embodiment_profile: EmbodimentProfile,
        policy: Policy,
        adapter: BaseEmbodimentAdapter,
        session_manager: SessionManager | None = None,
    ) -> None:
        self._policy_profile = policy_profile
        self._embodiment_profile = embodiment_profile
        self._policy = policy
        self._adapter = adapter
        self._session_manager = session_manager or SessionManager()
        self._policy_preprocessors = build_preprocessors(policy_profile.preprocess)
        self._policy_postprocessors = build_postprocessors(policy_profile.postprocess)
        self._adapter.setup(embodiment_profile, policy_profile)

    def reset(self, session_id: str) -> dict[str, bool]:
        """Reset robot-layer session state."""
        self._session_manager.reset(session_id)
        self._policy.reset(session_id)
        self._adapter.reset(session_id)
        return {"ok": True}

    def infer(self, session_id: str, obs_msg: dict[str, Any]) -> dict[str, Any]:
        """Infer one embodiment action from one embodiment observation."""
        rpc_sensor = RpcObservationSensor(obs_msg)
        robot_data = RobotData()
        rpc_sensor.read(robot_data)
        robot_data.observation = self._to_canonical_observation(
            robot_data.sensors.get(rpc_sensor.field_name, {})
        )
        robot_data.observation = self._preprocess(robot_data.observation)

        session_ctx = self._session_manager.get_or_create(session_id)
        self._policy.infer(robot_data, session_ctx)
        if robot_data.action is None:
            raise RuntimeError("model inference did not produce an action")
        robot_data.set_action(self._postprocess(robot_data.action))

        session_ctx.step_id += 1
        rpc_actuator = RpcActionActuator()
        rpc_actuator.write(robot_data)
        return self._adapter.from_canonical_action(rpc_actuator.response)

    def _to_canonical_observation(self, obs_msg: dict[str, Any]) -> RobotObservation:
        self._adapter.validate_input(obs_msg)
        canonical_obs = self._adapter.to_canonical_obs(obs_msg)
        required_fields = [
            field.name
            for field in self._policy_profile.canonical_observation_schema
            if not field.optional
        ]
        validate_required_fields(canonical_obs, required_fields)
        return RobotObservation(canonical_obs)

    def _preprocess(self, observation: RobotObservation) -> RobotObservation:
        return RobotObservation(run_preprocessors(observation.as_dict(), self._policy_preprocessors))

    def _postprocess(self, action: RobotAction) -> RobotAction:
        return RobotAction(run_postprocessors(action.as_dict(), self._policy_postprocessors))
