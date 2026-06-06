from policy_embodied_runtime.postprocess.base import BasePostprocessor
from policy_embodied_runtime.postprocess.noop import NoOpPostprocessor
from policy_embodied_runtime.postprocess.pipeline import build_postprocessors, run_postprocessors


def test_postprocess_defaults_to_noop_processor() -> None:
    postprocessors = build_postprocessors([])

    assert len(postprocessors) == 1
    assert isinstance(postprocessors[0], NoOpPostprocessor)
    assert isinstance(postprocessors[0], BasePostprocessor)


def test_postprocess_leaves_actions_unchanged() -> None:
    action = {
        "joint_position_delta": {
            "values": [0.5] * 7,
            "joint_names": [f"joint_{index}" for index in range(1, 8)],
            "unit": "rad",
        },
        "gripper_command": {"value": 0.2, "unit": "m"},
    }

    processed = run_postprocessors(action, build_postprocessors([]))

    assert processed == action
