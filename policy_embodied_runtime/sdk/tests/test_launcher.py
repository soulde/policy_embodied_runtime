from policy_embodied_runtime.sdk.policy_client.launcher import launch_local_server
from policy_embodied_runtime.sdk.policy_client.types import EndpointConfig


def test_launch_local_server_builds_subprocess_command(monkeypatch) -> None:
    recorded = {}

    class DummyProcess:
        def poll(self):
            return None

        def terminate(self):
            recorded["terminated"] = True

        def wait(self, timeout):
            recorded["wait_timeout"] = timeout

    def fake_popen(command, stdout, stderr):
        recorded["command"] = command
        recorded["stdout"] = stdout
        recorded["stderr"] = stderr
        return DummyProcess()

    monkeypatch.setattr("policy_embodied_runtime.sdk.policy_client.launcher.subprocess.Popen", fake_popen)

    handle = launch_local_server(
        EndpointConfig(
            endpoint="embodied-policy-runtime",
        ),
        policy_profile="policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json",
        embodiment_profile="policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json",
    )
    assert recorded["command"][1:3] == ["-m", "policy_embodied_runtime.server.apps.zmq_server"]
    assert recorded["command"][3:5] == ["--endpoint", "embodied-policy-runtime"]
    assert recorded["command"][5:9] == [
        "--policy-profile",
        "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json",
        "--embodiment-profile",
        "policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json",
    ]
    handle.terminate()
    assert recorded["terminated"] is True
