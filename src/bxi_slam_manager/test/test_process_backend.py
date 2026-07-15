from pathlib import Path

from bxi_slam_manager.process_backend import ProcessRegistry, RosLaunchBackend


class FakeChild:
    def __init__(self, pid: int, return_code=None) -> None:
        self.pid = pid
        self.return_code = return_code

    def poll(self):
        return self.return_code


def test_process_registry_persists_groups_for_crash_recovery(tmp_path: Path) -> None:
    registry = ProcessRegistry(tmp_path / "slam-processes.json")

    registry.replace({"point_lio": 101, "gicp": 102})

    assert registry.load() == {"point_lio": 101, "gicp": 102}


def test_process_registry_clear_removes_stale_state(tmp_path: Path) -> None:
    registry = ProcessRegistry(tmp_path / "slam-processes.json")
    registry.replace({"nav2": 103})

    registry.replace({})

    assert registry.load() == {}
    assert not registry.path.exists()


def test_backend_reports_unexpected_child_exit(tmp_path: Path) -> None:
    backend = RosLaunchBackend(tmp_path / "slam-processes.json")
    backend._children = {
        "point_lio": FakeChild(101, None),
        "gicp": FakeChild(102, 2),
    }

    assert backend.poll_failed_processes() == {"gicp": 2}
    assert set(backend._children) == {"point_lio"}
    assert ProcessRegistry(tmp_path / "slam-processes.json").load() == {
        "point_lio": 101
    }


def test_backend_cleans_registered_groups_on_supervisor_restart(
    tmp_path: Path, monkeypatch
) -> None:
    path = tmp_path / "slam-processes.json"
    ProcessRegistry(path).replace({"point_lio": 501, "nav2": 502})
    terminated = []
    monkeypatch.setattr(
        RosLaunchBackend,
        "_terminate_stale_group",
        staticmethod(terminated.append),
    )
    monkeypatch.setattr("bxi_slam_manager.process_backend.os.name", "posix")

    RosLaunchBackend(path)

    assert set(terminated) == {501, 502}
    assert not path.exists()
