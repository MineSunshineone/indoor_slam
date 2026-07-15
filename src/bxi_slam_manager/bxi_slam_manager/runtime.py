"""Pure, testable state machine for the SLAM process supervisor."""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, replace
from enum import Enum
from pathlib import Path
from typing import Callable, Protocol


class RuntimeMode(str, Enum):
    """Externally visible and transitional runtime modes."""

    idle = "idle"
    new_mapping = "new_mapping"
    localizing = "localizing"
    navigation = "navigation"
    extend_mapping = "extend_mapping"
    error = "error"


@dataclass(frozen=True)
class MapArtifacts:
    """The immutable 3D and 2D inputs required to activate a map."""

    map_id: str
    pcd_path: Path
    yaml_path: Path

    def validate(self) -> None:
        if not self.map_id.strip():
            raise ValueError("map artifacts require a map_id")
        if self.pcd_path.suffix.lower() != ".pcd":
            raise ValueError("map artifacts require a PCD file")
        if self.yaml_path.suffix.lower() not in {".yaml", ".yml"}:
            raise ValueError("map artifacts require a Nav2 YAML file")


@dataclass(frozen=True)
class RuntimeRequest:
    request_id: str
    mode: RuntimeMode
    artifacts: MapArtifacts | None = None


@dataclass(frozen=True)
class RuntimeResult:
    accepted: bool
    transition_id: str
    message: str


@dataclass(frozen=True)
class RuntimeSnapshot:
    current_mode: RuntimeMode = RuntimeMode.idle
    desired_mode: RuntimeMode = RuntimeMode.idle
    transition_id: str = ""
    active_map_id: str = ""
    localized: bool = False
    fitness_score: float = 0.0
    inlier_ratio: float = 0.0
    driver_healthy: bool = False
    last_error: str = ""


class ProcessBackend(Protocol):
    """Owns every on-demand SLAM/Nav2 child process."""

    def stop_pipeline(self) -> None: ...

    def start_new_mapping(self) -> None: ...

    def start_navigation(self, artifacts: MapArtifacts) -> None: ...

    def start_extend_mapping(self, artifacts: MapArtifacts) -> None: ...

    def prepare_extend_mapping(self, artifacts: MapArtifacts) -> None: ...

    def poll_failed_processes(self) -> dict[str, int]: ...


class RuntimeSupervisor:
    """Serializes mode transitions and makes repeated requests idempotent."""

    def __init__(
        self,
        processes: ProcessBackend,
        *,
        require_driver_health: bool = False,
        localization_timeout_s: float = 3.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._processes = processes
        self._require_driver_health = require_driver_health
        self._localization_timeout_s = max(0.5, localization_timeout_s)
        self._clock = clock
        self._lock = threading.RLock()
        self._snapshot = RuntimeSnapshot()
        self._results: dict[str, RuntimeResult] = {}
        self._last_localization_update_at: float | None = None

    @property
    def snapshot(self) -> RuntimeSnapshot:
        with self._lock:
            return self._snapshot

    def apply(self, request: RuntimeRequest) -> RuntimeResult:
        with self._lock:
            cached = self._results.get(request.request_id)
            if cached is not None:
                return cached
            validation_error = self._validate_request(request)
            if validation_error:
                result = RuntimeResult(False, "", validation_error)
                self._results[request.request_id] = result
                return result

            transition_id = uuid.uuid4().hex
            try:
                self._transition(request, transition_id)
            except Exception as exc:
                try:
                    self._processes.stop_pipeline()
                finally:
                    self._snapshot = RuntimeSnapshot(
                        current_mode=RuntimeMode.error,
                        desired_mode=request.mode,
                        transition_id=transition_id,
                        last_error=str(exc),
                    )
                result = RuntimeResult(False, transition_id, str(exc))
            else:
                result = RuntimeResult(True, transition_id, "accepted")
            self._results[request.request_id] = result
            return result

    def update_localization(
        self,
        *,
        localized: bool,
        fitness_score: float,
        inlier_ratio: float,
    ) -> None:
        with self._lock:
            if self._snapshot.desired_mode not in {
                RuntimeMode.navigation,
                RuntimeMode.extend_mapping,
            }:
                return
            self._last_localization_update_at = self._clock()
            fitness_score = (
                float(fitness_score) if math.isfinite(fitness_score) else 0.0
            )
            inlier_ratio = (
                float(inlier_ratio) if math.isfinite(inlier_ratio) else 0.0
            )
            if (
                self._snapshot.localized
                and not localized
                and self._snapshot.current_mode in {
                    RuntimeMode.navigation,
                    RuntimeMode.extend_mapping,
                }
            ):
                self._processes.stop_pipeline()
                self._snapshot = replace(
                    self._snapshot,
                    current_mode=RuntimeMode.error,
                    localized=False,
                    fitness_score=fitness_score,
                    inlier_ratio=inlier_ratio,
                    last_error="3D localization fix was lost",
                )
                self._last_localization_update_at = None
                return
            current = self._snapshot.current_mode
            if localized and current is RuntimeMode.localizing:
                current = self._snapshot.desired_mode
            self._snapshot = replace(
                self._snapshot,
                current_mode=current,
                localized=localized,
                fitness_score=fitness_score,
                inlier_ratio=inlier_ratio,
                last_error="" if localized else self._snapshot.last_error,
            )

    def check_health(self) -> None:
        """Relock stale localization and fail closed on child-process exit."""
        with self._lock:
            failures = self._processes.poll_failed_processes()
            if failures and self._snapshot.current_mode not in {
                RuntimeMode.idle,
                RuntimeMode.error,
            }:
                details = ", ".join(
                    f"{name} exited with code {code}"
                    for name, code in sorted(failures.items())
                )
                self._processes.stop_pipeline()
                self._snapshot = replace(
                    self._snapshot,
                    current_mode=RuntimeMode.error,
                    localized=False,
                    last_error=details,
                )
                self._last_localization_update_at = None
                return

            last_update = self._last_localization_update_at
            if (
                self._snapshot.localized
                and last_update is not None
                and self._clock() - last_update > self._localization_timeout_s
            ):
                self._processes.stop_pipeline()
                self._snapshot = replace(
                    self._snapshot,
                    current_mode=RuntimeMode.error,
                    localized=False,
                    last_error="3D localization status is stale",
                )
                self._last_localization_update_at = None

    def update_driver_health(self, healthy: bool) -> None:
        with self._lock:
            if (
                self._require_driver_health
                and not healthy
                and self._snapshot.current_mode
                not in {RuntimeMode.idle, RuntimeMode.error}
            ):
                self._processes.stop_pipeline()
                self._snapshot = replace(
                    self._snapshot,
                    current_mode=RuntimeMode.error,
                    driver_healthy=False,
                    localized=False,
                    last_error="LiDAR data stream was lost",
                )
                self._last_localization_update_at = None
            else:
                self._snapshot = replace(self._snapshot, driver_healthy=healthy)

    def _validate_request(self, request: RuntimeRequest) -> str:
        if not request.request_id.strip():
            return "request_id is required"
        if (
            self._require_driver_health
            and request.mode is not RuntimeMode.idle
            and not self._snapshot.driver_healthy
        ):
            return "LiDAR data stream is unhealthy"
        needs_map = request.mode in {
            RuntimeMode.navigation,
            RuntimeMode.extend_mapping,
        }
        if needs_map and request.artifacts is None:
            return "map artifacts are required for this mode"
        if request.artifacts is not None:
            try:
                request.artifacts.validate()
            except ValueError as exc:
                return str(exc)
        if request.mode is RuntimeMode.localizing or request.mode is RuntimeMode.error:
            return f"{request.mode.value} is an internal mode"
        return ""

    def _transition(self, request: RuntimeRequest, transition_id: str) -> None:
        can_reuse_localization = (
            request.mode is RuntimeMode.extend_mapping
            and request.artifacts is not None
            and self._snapshot.current_mode is RuntimeMode.navigation
            and self._snapshot.localized
            and self._snapshot.active_map_id == request.artifacts.map_id
        )
        if can_reuse_localization:
            self._processes.prepare_extend_mapping(request.artifacts)
            self._snapshot = replace(
                self._snapshot,
                current_mode=RuntimeMode.extend_mapping,
                desired_mode=RuntimeMode.extend_mapping,
                transition_id=transition_id,
                last_error="",
            )
            return

        self._processes.stop_pipeline()
        self._last_localization_update_at = None
        active_map_id = ""
        current_mode = request.mode
        if request.mode is RuntimeMode.new_mapping:
            self._processes.start_new_mapping()
        elif request.mode is RuntimeMode.navigation:
            assert request.artifacts is not None
            self._processes.start_navigation(request.artifacts)
            active_map_id = request.artifacts.map_id
            current_mode = RuntimeMode.localizing
        elif request.mode is RuntimeMode.extend_mapping:
            assert request.artifacts is not None
            self._processes.start_extend_mapping(request.artifacts)
            active_map_id = request.artifacts.map_id
            current_mode = RuntimeMode.localizing

        self._snapshot = RuntimeSnapshot(
            current_mode=current_mode,
            desired_mode=request.mode,
            transition_id=transition_id,
            active_map_id=active_map_id,
            driver_healthy=self._snapshot.driver_healthy,
        )
