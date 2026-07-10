from pathlib import Path

from bxi_slam_manager.runtime import (
    MapArtifacts,
    RuntimeMode,
    RuntimeRequest,
    RuntimeSupervisor,
)


class FakeProcesses:
    def __init__(self) -> None:
        self.events: list[tuple[str, object]] = []
        self.failures: dict[str, int] = {}

    def stop_pipeline(self) -> None:
        self.events.append(("stop", None))

    def start_new_mapping(self) -> None:
        self.events.append(("start_new_mapping", None))

    def start_navigation(self, artifacts: MapArtifacts) -> None:
        self.events.append(("start_navigation", artifacts))

    def start_extend_mapping(self, artifacts: MapArtifacts) -> None:
        self.events.append(("start_extend_mapping", artifacts))

    def prepare_extend_mapping(self, artifacts: MapArtifacts) -> None:
        self.events.append(("prepare_extend_mapping", artifacts))

    def poll_failed_processes(self) -> dict[str, int]:
        failures, self.failures = self.failures, {}
        return failures


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def artifacts() -> MapArtifacts:
    return MapArtifacts(
        map_id="map-a",
        pcd_path=Path("/maps/map-a/map.pcd"),
        yaml_path=Path("/maps/map-a/map.yaml"),
    )


def test_boot_is_quiet_and_starts_no_algorithm_pipeline() -> None:
    processes = FakeProcesses()

    runtime = RuntimeSupervisor(processes)

    assert runtime.snapshot.current_mode is RuntimeMode.idle
    assert runtime.snapshot.desired_mode is RuntimeMode.idle
    assert processes.events == []


def test_mode_change_stops_previous_pipeline_before_starting_next() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)

    result = runtime.apply(
        RuntimeRequest(
            request_id="req-1",
            mode=RuntimeMode.navigation,
            artifacts=artifacts(),
        )
    )

    assert result.accepted
    assert processes.events == [
        ("stop", None),
        ("start_navigation", artifacts()),
    ]
    assert runtime.snapshot.current_mode is RuntimeMode.localizing
    assert runtime.snapshot.active_map_id == "map-a"
    assert not runtime.snapshot.localized


def test_duplicate_request_id_is_idempotent() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    request = RuntimeRequest(
        request_id="same-request",
        mode=RuntimeMode.new_mapping,
    )

    first = runtime.apply(request)
    second = runtime.apply(request)

    assert first.transition_id == second.transition_id
    assert processes.events == [("stop", None), ("start_new_mapping", None)]


def test_navigation_requires_a_complete_3d_map_bundle() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())

    result = runtime.apply(
        RuntimeRequest(request_id="bad-map", mode=RuntimeMode.navigation)
    )

    assert not result.accepted
    assert "artifacts" in result.message
    assert runtime.snapshot.current_mode is RuntimeMode.idle


def test_production_runtime_rejects_modes_while_lidar_is_unhealthy() -> None:
    runtime = RuntimeSupervisor(FakeProcesses(), require_driver_health=True)

    result = runtime.apply(
        RuntimeRequest(request_id="no-lidar", mode=RuntimeMode.new_mapping)
    )

    assert not result.accepted
    assert "LiDAR" in result.message


def test_lidar_dropout_stops_an_active_pipeline() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes, require_driver_health=True)
    runtime.update_driver_health(True)
    runtime.apply(RuntimeRequest('map', RuntimeMode.new_mapping))
    processes.events.clear()

    runtime.update_driver_health(False)

    assert processes.events == [('stop', None)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert 'LiDAR' in runtime.snapshot.last_error


def test_localization_health_controls_navigation_readiness() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())
    runtime.apply(
        RuntimeRequest(
            request_id="nav",
            mode=RuntimeMode.navigation,
            artifacts=artifacts(),
        )
    )

    runtime.update_localization(
        localized=True,
        fitness_score=0.18,
        inlier_ratio=0.72,
    )

    assert runtime.snapshot.current_mode is RuntimeMode.navigation
    assert runtime.snapshot.localized
    assert runtime.snapshot.fitness_score == 0.18
    assert runtime.snapshot.inlier_ratio == 0.72


def test_localized_navigation_switches_to_extend_without_losing_gicp() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    map_artifacts = artifacts()
    runtime.apply(RuntimeRequest('nav-first', RuntimeMode.navigation, map_artifacts))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()

    result = runtime.apply(
        RuntimeRequest('extend', RuntimeMode.extend_mapping, map_artifacts)
    )

    assert result.accepted
    assert processes.events == [('prepare_extend_mapping', map_artifacts)]
    assert runtime.snapshot.current_mode is RuntimeMode.extend_mapping
    assert runtime.snapshot.localized


def test_stale_localization_status_relocks_navigation() -> None:
    processes = FakeProcesses()
    clock = ManualClock()
    runtime = RuntimeSupervisor(
        processes,
        localization_timeout_s=3.0,
        clock=clock,
    )
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()

    clock.now = 3.1
    runtime.check_health()

    assert processes.events == [('stop', None)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert 'stale' in runtime.snapshot.last_error.lower()


def test_unexpected_child_exit_stops_remaining_pipeline() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()
    processes.failures = {'gicp': 2}

    runtime.check_health()

    assert processes.events == [('stop', None)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert 'gicp' in runtime.snapshot.last_error


def test_idle_ignores_queued_localization_messages() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())

    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )

    assert runtime.snapshot.current_mode is RuntimeMode.idle
    assert not runtime.snapshot.localized


def test_nonfinite_localization_metrics_are_not_exposed_to_json_clients() -> None:
    runtime = RuntimeSupervisor(FakeProcesses())
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))

    runtime.update_localization(
        localized=False, fitness_score=float('inf'), inlier_ratio=float('nan')
    )

    assert runtime.snapshot.fitness_score == 0.0
    assert runtime.snapshot.inlier_ratio == 0.0


def test_lost_fix_stops_an_inflight_navigation_pipeline() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    runtime.update_localization(
        localized=True, fitness_score=0.1, inlier_ratio=0.8
    )
    processes.events.clear()

    runtime.update_localization(
        localized=False, fitness_score=2.5, inlier_ratio=0.1
    )

    assert processes.events == [('stop', None)]
    assert runtime.snapshot.current_mode is RuntimeMode.error
    assert not runtime.snapshot.localized
    assert 'lost' in runtime.snapshot.last_error.lower()


def test_unlocalized_updates_are_expected_during_initial_alignment() -> None:
    processes = FakeProcesses()
    runtime = RuntimeSupervisor(processes)
    runtime.apply(RuntimeRequest('nav', RuntimeMode.navigation, artifacts()))
    processes.events.clear()

    runtime.update_localization(
        localized=False, fitness_score=2.5, inlier_ratio=0.1
    )

    assert processes.events == []
    assert runtime.snapshot.current_mode is RuntimeMode.localizing
