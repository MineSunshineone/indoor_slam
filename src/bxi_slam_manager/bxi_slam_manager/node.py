"""ROS 2 adapter for the pure runtime supervisor."""

from __future__ import annotations

import time
from pathlib import Path

import rclpy
from bxi_nav_interfaces.msg import RelocalizationStatus, RuntimeStatus
from bxi_nav_interfaces.srv import SetRuntimeMode
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from .process_backend import RosLaunchBackend
from .runtime import MapArtifacts, RuntimeMode, RuntimeRequest, RuntimeSupervisor


_MODE_BY_VALUE = {
    SetRuntimeMode.Request.MODE_IDLE: RuntimeMode.idle,
    SetRuntimeMode.Request.MODE_NEW_MAPPING: RuntimeMode.new_mapping,
    SetRuntimeMode.Request.MODE_NAVIGATION: RuntimeMode.navigation,
    SetRuntimeMode.Request.MODE_EXTEND_MAPPING: RuntimeMode.extend_mapping,
}


class SlamManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("bxi_slam_manager")
        self._runtime = RuntimeSupervisor(
            RosLaunchBackend(),
            require_driver_health=True,
            localization_timeout_s=3.0,
        )
        self._last_lidar_at = 0.0
        self._localization_epoch_ns = 0
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(
            RuntimeStatus, "/slam/runtime/status", status_qos
        )
        self.create_service(
            SetRuntimeMode, "/slam/runtime/set_mode", self._on_set_mode
        )
        self.create_subscription(
            RelocalizationStatus,
            "/nav/relocalization_status",
            self._on_relocalization_status,
            status_qos,
        )
        self.create_subscription(
            CustomMsg,
            "/livox/lidar",
            self._on_lidar,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0, self._health_tick)
        self._publish_status()

    def _on_set_mode(self, request, response):
        mode = _MODE_BY_VALUE.get(request.mode)
        if mode is None:
            response.accepted = False
            response.message = "unsupported runtime mode"
            return response
        artifacts = None
        if request.map_id or request.pcd_path or request.yaml_path:
            artifacts = MapArtifacts(
                map_id=request.map_id,
                pcd_path=Path(request.pcd_path),
                yaml_path=Path(request.yaml_path),
            )
        result = self._runtime.apply(
            RuntimeRequest(
                request_id=request.request_id,
                mode=mode,
                artifacts=artifacts,
            )
        )
        if result.accepted and mode in {
            RuntimeMode.navigation,
            RuntimeMode.extend_mapping,
        }:
            self._localization_epoch_ns = self.get_clock().now().nanoseconds
        response.accepted = result.accepted
        response.transition_id = result.transition_id
        response.message = result.message
        self._publish_status()
        return response

    def _on_relocalization_status(self, msg: RelocalizationStatus) -> None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )
        if stamp_ns <= self._localization_epoch_ns:
            return
        self._runtime.update_localization(
            localized=bool(msg.localized),
            fitness_score=float(msg.fitness_score),
            inlier_ratio=float(msg.inlier_ratio),
        )
        self._publish_status()

    def _on_lidar(self, _msg: CustomMsg) -> None:
        self._last_lidar_at = time.monotonic()

    def _health_tick(self) -> None:
        healthy = (
            self._last_lidar_at > 0.0
            and time.monotonic() - self._last_lidar_at < 2.5
        )
        self._runtime.update_driver_health(healthy)
        self._runtime.check_health()
        self._publish_status()

    def _publish_status(self) -> None:
        snapshot = self._runtime.snapshot
        msg = RuntimeStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_mode = snapshot.current_mode.value
        msg.desired_mode = snapshot.desired_mode.value
        msg.transition_id = snapshot.transition_id
        msg.active_map_id = snapshot.active_map_id
        msg.localized = snapshot.localized
        msg.fitness_score = snapshot.fitness_score
        msg.inlier_ratio = snapshot.inlier_ratio
        msg.driver_healthy = snapshot.driver_healthy
        msg.last_error = snapshot.last_error
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._runtime.apply(
            RuntimeRequest(
                request_id=f"shutdown-{time.monotonic_ns()}",
                mode=RuntimeMode.idle,
            )
        )
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SlamManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
