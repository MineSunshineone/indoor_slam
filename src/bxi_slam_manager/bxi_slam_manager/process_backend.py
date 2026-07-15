"""Child-process ownership for on-demand Point-LIO, GICP and Nav2 stacks."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from collections.abc import Sequence
from pathlib import Path

from .runtime import MapArtifacts


class ProcessRegistry:
    """Small atomic PID registry used to clean process groups after a crash."""

    def __init__(self, path: Path) -> None:
        self.path = path

    def load(self) -> dict[str, int]:
        try:
            payload = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        if not isinstance(payload, dict):
            return {}
        return {
            str(name): int(pid)
            for name, pid in payload.items()
            if isinstance(name, str)
            and isinstance(pid, int)
            and not isinstance(pid, bool)
            and pid > 1
        }

    def replace(self, groups: dict[str, int]) -> None:
        if not groups:
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(f".{self.path.name}.{os.getpid()}.tmp")
        temporary.write_text(
            json.dumps(groups, sort_keys=True), encoding="utf-8"
        )
        os.replace(temporary, self.path)


def _default_registry_path() -> Path:
    configured = os.environ.get("BXI_SLAM_PROCESS_REGISTRY")
    if configured:
        return Path(configured)
    if os.name == "posix" and hasattr(os, "geteuid") and os.geteuid() == 0:
        return Path("/run/bxi/slam-processes.json")
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR")
    if runtime_dir:
        return Path(runtime_dir) / "bxi-slam-processes.json"
    uid = os.getuid() if hasattr(os, "getuid") else os.getpid()
    return Path("/tmp") / f"bxi-slam-processes-{uid}.json"


class RosLaunchBackend:
    """Starts ROS launches without a shell and stops them as process groups."""

    def __init__(self, registry_path: Path | None = None) -> None:
        self._lock = threading.RLock()
        self._children: dict[str, subprocess.Popen[bytes]] = {}
        self._registry = ProcessRegistry(registry_path or _default_registry_path())
        self._cleanup_stale_groups()

    def stop_pipeline(self) -> None:
        with self._lock:
            children, self._children = list(self._children.values()), {}
        for child in reversed(children):
            self._stop_child(child)
        with self._lock:
            self._persist_children()

    def poll_failed_processes(self) -> dict[str, int]:
        failures: dict[str, int] = {}
        with self._lock:
            for name, child in list(self._children.items()):
                return_code = child.poll()
                if return_code is not None:
                    failures[name] = int(return_code)
                    self._children.pop(name, None)
            if failures:
                self._persist_children()
        return failures

    def start_new_mapping(self) -> None:
        self._start(
            "point_lio",
            [
                "ros2", "launch", "point_lio",
                "point_lio_with_mapping_control.launch.py", "rviz:=False",
            ]
        )

    def start_navigation(self, artifacts: MapArtifacts) -> None:
        self._start_point_lio()
        self._start_gicp(artifacts)
        self._start(
            "nav2",
            [
                "ros2", "launch", "nav", "indoor_navigation_launch.py",
                f"map:={artifacts.yaml_path}", "autostart:=true", "rviz:=false",
            ]
        )

    def start_extend_mapping(self, artifacts: MapArtifacts) -> None:
        self.start_new_mapping()
        self._start_gicp(artifacts)

    def prepare_extend_mapping(self, _artifacts: MapArtifacts) -> None:
        """Keep Point-LIO/GICP and remove Nav2's static /map publisher."""
        self._stop_named("nav2")

    def _start_point_lio(self) -> None:
        self._start(
            "point_lio",
            [
                "ros2", "launch", "point_lio",
                "point_lio_with_mapping_control.launch.py", "rviz:=False",
            ]
        )

    def _start_gicp(self, artifacts: MapArtifacts) -> None:
        self._start(
            "gicp",
            [
                "ros2", "launch", "small_gicp_relocalization",
                "small_gicp_relocalization_launch.py",
                f"prior_pcd_file:={artifacts.pcd_path}",
            ]
        )

    def _start(self, name: str, command: Sequence[str]) -> None:
        self._stop_named(name)
        child = subprocess.Popen(
            list(command),
            env={**os.environ},
            start_new_session=True,
        )
        with self._lock:
            self._children[name] = child
            self._persist_children()

    def _stop_named(self, name: str) -> None:
        with self._lock:
            child = self._children.pop(name, None)
        if child is not None:
            self._stop_child(child)
        with self._lock:
            self._persist_children()

    @staticmethod
    def _stop_child(child: subprocess.Popen[bytes]) -> None:
        if child.poll() is not None:
            return
        try:
            if os.name == "posix":
                os.killpg(child.pid, signal.SIGINT)
            else:
                child.terminate()
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            if os.name == "posix":
                os.killpg(child.pid, signal.SIGKILL)
            else:
                child.kill()
            child.wait(timeout=2)

    def _persist_children(self) -> None:
        try:
            self._registry.replace(
                {name: child.pid for name, child in self._children.items()}
            )
        except OSError:
            # Registry recovery is defense-in-depth; never prevent an immediate
            # stop/start operation because the runtime directory is read-only.
            pass

    def _cleanup_stale_groups(self) -> None:
        stale = self._registry.load()
        try:
            self._registry.replace({})
        except OSError:
            pass
        if os.name != "posix":
            return
        for process_group in stale.values():
            self._terminate_stale_group(process_group)

    @staticmethod
    def _terminate_stale_group(process_group: int) -> None:
        try:
            os.killpg(process_group, signal.SIGINT)
        except ProcessLookupError:
            return
        except PermissionError:
            return
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.killpg(process_group, 0)
            except ProcessLookupError:
                return
            except PermissionError:
                return
            time.sleep(0.05)
        try:
            os.killpg(process_group, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
