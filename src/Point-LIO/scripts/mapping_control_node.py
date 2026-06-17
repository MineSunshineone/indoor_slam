#!/usr/bin/env python3
"""面向 App 对接的 HTTP + ROS 2 建图控制节点。

该节点订阅 Point-LIO 发布的配准点云，并增量生成 Nav2 风格的 2D 栅格地图。
App 通过 HTTP 接口控制地图累积流程：开始、暂停/继续、取消、保存、查询状态。
"""

from __future__ import annotations

import argparse
import json
import re
import threading
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np


# 地图名会直接用于生成文件名，只允许安全字符，避免路径穿越和奇怪文件名。
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def sanitize_map_name(map_name: str) -> str:
    # 清理并校验地图名，确保后续可以安全拼成 .pgm/.yaml 文件路径。
    cleaned = map_name.strip()
    if not MAP_NAME_RE.fullmatch(cleaned):
        raise ValueError("map_name must be 1-64 chars: A-Z, a-z, 0-9, _ or -")
    return cleaned


@dataclass
class MappingConfig:
    """建图输出和点云投影参数。"""

    # 保存 .pgm 和 .yaml 的目录。
    output_dir: Path
    # 栅格分辨率，单位 m/cell。
    resolution: float = 0.10
    # 输出地图的物理宽度和高度，单位 m。
    size_x: float = 60.0
    size_y: float = 60.0
    # 输出地图左下角在 Point-LIO 世界坐标中的位置。
    origin_x: float = -30.0
    origin_y: float = -30.0
    # 落在该高度范围内的点会被当作障碍物证据。
    occupied_z_min: float = -0.8
    occupied_z_max: float = 0.3
    # 落在该高度范围内的点会被当作可通行/空闲证据。
    free_z_min: float = -1.30
    free_z_max: float = -0.35
    # 障碍物膨胀格数，用于给 Nav2 留出安全边界。
    occupied_dilation: int = 1


class MappingSession:
    """单次 App 建图任务的线程安全内存状态。"""

    def __init__(self, config: MappingConfig) -> None:
        self.config = config
        # 根据物理尺寸和分辨率换算固定大小的二维栅格。
        self.width = int(round(config.size_x / config.resolution))
        self.height = int(round(config.size_y / config.resolution))
        # HTTP 请求线程和 ROS 点云回调线程会共享同一个 session，因此这里必须加锁。
        self.lock = threading.RLock()
        # 状态取值：idle/mapping/paused/saving/saved/cancelled。
        self.state = "idle"
        self.current_map_name = ""
        self.last_error = ""
        # 分别累计障碍物证据和空闲证据；保存时再融合成 Nav2 占据栅格。
        self.occ_counts = np.zeros((self.height, self.width), dtype=np.uint32)
        self.free_counts = np.zeros((self.height, self.width), dtype=np.uint32)

    def start(self, map_name: str) -> dict[str, Any]:
        with self.lock:
            # 开始新地图时清空未保存的旧累积结果，避免两次建图混在一起。
            self.current_map_name = sanitize_map_name(map_name)
            self._clear_counts()
            self.state = "mapping"
            self.last_error = ""
            return self.status()

    def pause(self, paused: bool) -> dict[str, Any]:
        with self.lock:
            if not self.current_map_name or self.state in {"idle", "cancelled"}:
                raise RuntimeError("no active mapping task")
            # 暂停只影响地图累积；Point-LIO 仍继续发布里程计、点云和 TF。
            self.state = "paused" if paused else "mapping"
            return self.status()

    def cancel(self) -> dict[str, Any]:
        with self.lock:
            # 取消表示放弃当前地图，因此清空所有未保存证据。
            self._clear_counts()
            self.state = "cancelled"
            return self.status()

    def ingest_xyz(self, xyz: np.ndarray) -> None:
        with self.lock:
            # 在 idle/paused/saved 等状态下点云仍可能到达；只有 mapping 状态才累积。
            if self.state != "mapping" or xyz.size == 0:
                return

            # 丢弃 NaN/Inf 点，避免索引计算报错或污染地图。
            xyz = xyz[np.isfinite(xyz).all(axis=1)]
            if xyz.size == 0:
                return

            cfg = self.config
            # 将米制 x/y 坐标转换为固定输出栅格里的列/行索引。
            ix = np.floor((xyz[:, 0] - cfg.origin_x) / cfg.resolution).astype(np.int64)
            iy = np.floor((xyz[:, 1] - cfg.origin_y) / cfg.resolution).astype(np.int64)
            # 只保留落在输出地图范围内的点。
            keep = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
            ix, iy, z = ix[keep], iy[keep], xyz[keep, 2]

            # 通过高度带判断某个栅格收到的是障碍物证据还是空闲证据。
            occ = (z >= cfg.occupied_z_min) & (z <= cfg.occupied_z_max)
            free = (z >= cfg.free_z_min) & (z <= cfg.free_z_max)
            # np.add.at 支持重复索引累加，适合一帧点云内多个点落到同一格。
            np.add.at(self.occ_counts, (iy[occ], ix[occ]), 1)
            np.add.at(self.free_counts, (iy[free], ix[free]), 1)

    def save(self, map_name: str | None = None) -> dict[str, Any]:
        with self.lock:
            # 保存时允许重新指定地图名，但仍必须满足安全文件名规则。
            if map_name is not None:
                self.current_map_name = sanitize_map_name(map_name)
            if not self.current_map_name:
                raise RuntimeError("map_name is required before saving")

            self.state = "saving"
            self.config.output_dir.mkdir(parents=True, exist_ok=True)
            pgm_path = self._pgm_path()
            yaml_path = self._yaml_path()
            grid = self._build_grid()
            self._write_pgm(pgm_path, grid)
            self._write_yaml(yaml_path, pgm_path.name)
            self.state = "saved"
            result = self.status()
            result["success"] = True
            return result

    def status(self) -> dict[str, Any]:
        # 返回给 App 的状态字段保持简单稳定，便于前端直接展示。
        return {
            "state": self.state,
            "current_map_name": self.current_map_name,
            "map_pgm_path": str(self._pgm_path()) if self.current_map_name else "",
            "map_yaml_path": str(self._yaml_path()) if self.current_map_name else "",
            "resolution": self.config.resolution,
            "width": self.width,
            "height": self.height,
            "last_error": self.last_error,
        }

    def _clear_counts(self) -> None:
        # 只清空累积证据，不改变地图参数。
        self.occ_counts.fill(0)
        self.free_counts.fill(0)

    def _pgm_path(self) -> Path:
        return self.config.output_dir / f"{self.current_map_name}.pgm"

    def _yaml_path(self) -> Path:
        return self.config.output_dir / f"{self.current_map_name}.yaml"

    def _build_grid(self) -> np.ndarray:
        # Nav2/ROS PGM 约定：0 表示占用，254 表示空闲，205 通常作为未知灰度。
        occ = self.occ_counts > 0
        free = self.free_counts > 0
        grid = np.full((self.height, self.width), 205, dtype=np.uint8)
        grid[free] = 254
        # 对障碍物做曼哈顿邻域膨胀，给导航避障留下安全边界。
        for _ in range(max(self.config.occupied_dilation, 0)):
            expanded = occ.copy()
            expanded[1:, :] |= occ[:-1, :]
            expanded[:-1, :] |= occ[1:, :]
            expanded[:, 1:] |= occ[:, :-1]
            expanded[:, :-1] |= occ[:, 1:]
            occ = expanded
        grid[occ] = 0
        return grid

    def _write_pgm(self, pgm_path: Path, grid: np.ndarray) -> None:
        # ROS 地图原点在左下角，而 PGM 图像原点在左上角，因此写入前上下翻转。
        image = np.flipud(grid)
        header = f"P5\n{self.width} {self.height}\n255\n".encode("ascii")
        pgm_path.write_bytes(header + image.tobytes())

    def _write_yaml(self, yaml_path: Path, image_name: str) -> None:
        # 生成 Nav2 map_server 可直接加载的 YAML 元数据。
        cfg = self.config
        yaml_path.write_text(
            "\n".join(
                [
                    f"image: {image_name}",
                    "mode: trinary",
                    f"resolution: {cfg.resolution:.6f}",
                    f"origin: [{cfg.origin_x:.6f}, {cfg.origin_y:.6f}, 0.000000]",
                    "negate: 0",
                    "occupied_thresh: 0.65",
                    "free_thresh: 0.25",
                    "",
                ]
            ),
            encoding="utf-8",
        )


class MappingHttpServer(ThreadingHTTPServer):
    """携带 MappingSession 的 HTTP 服务器。"""

    def __init__(self, server_address: tuple[str, int], session: MappingSession) -> None:
        super().__init__(server_address, MappingRequestHandler)
        self.session = session


class MappingRequestHandler(BaseHTTPRequestHandler):
    """处理 App 调用的建图 HTTP 接口。"""

    server: MappingHttpServer

    def do_GET(self) -> None:
        # App 轮询这个接口，用来展示当前状态和当前地图名。
        if self.path == "/mapping/status":
            self._send_json(200, self.server.session.status())
            return
        self._send_json(404, {"success": False, "message": "not found"})

    def do_POST(self) -> None:
        try:
            body = self._read_json()
            # HTTP 层只做路由；真正的持久状态和状态转换都放在 MappingSession。
            if self.path == "/mapping/start":
                result = self.server.session.start(str(body.get("map_name", "")))
            elif self.path == "/mapping/pause":
                result = self.server.session.pause(bool(body.get("paused", True)))
            elif self.path == "/mapping/cancel":
                result = self.server.sessio  .cancel()
            elif self.path == "/mapping/save":
                map_name = body.get("map_name")
                result = self.server.session.save(str(map_name) if map_name is not None else None)
            else:
                self._send_json(404, {"success": False, "message": "not found"})
                return
            result.setdefault("success", True)
            self._send_json(200, result)
        except Exception as exc:
            self.server.session.last_error = str(exc)
            self._send_json(400, {"success": False, "message": str(exc)})

    def log_message(self, fmt: str, *args: Any) -> None:
        # 关闭 BaseHTTPRequestHandler 默认访问日志，避免 ROS 控制台刷屏。
        return

    def _read_json(self) -> dict[str, Any]:
        # 支持无请求体的 POST，例如 /mapping/cancel。
        length = int(self.headers.get("Content-Length", "0"))
        if length == 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw)

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        # 所有接口统一返回 JSON，ensure_ascii=False 方便中文错误信息直出。
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def run_http_server(session: MappingSession, host: str, port: int) -> MappingHttpServer:
    # HTTP 服务运行在守护线程里，让 ROS executor 占用主线程。
    server = MappingHttpServer((host, port), session)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def run_ros(args: argparse.Namespace, session: MappingSession) -> None:
    # ROS 相关依赖只在实际启动 ROS 模式时导入，便于单元测试直接加载纯逻辑。
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    class MappingControlNode(Node):
        """订阅 Point-LIO 点云，并把点云数据送入 MappingSession。"""

        def __init__(self) -> None:
            super().__init__("mapping_control_node")
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, qos)
            self.get_logger().info(
                f"mapping control HTTP: http://{args.host}:{args.port}, "
                f"cloud_topic={args.cloud_topic}"
            )

        def on_cloud(self, msg: PointCloud2) -> None:
            # 将 ROS PointCloud2 转成 Nx3 numpy 数组，再交给纯 Python 状态逻辑处理。
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
            session.ingest_xyz(pts)

    rclpy.init()
    node = MappingControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    # 这些参数也会由 launch 文件透传，便于现场按地图范围和高度带调参。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8088, type=int)
    parser.add_argument("--output-dir", default="src/bxi_nav/maps", type=Path)
    parser.add_argument("--resolution", default=0.10, type=float)
    parser.add_argument("--size-x", default=60.0, type=float)
    parser.add_argument("--size-y", default=60.0, type=float)
    parser.add_argument("--origin-x", default=-30.0, type=float)
    parser.add_argument("--origin-y", default=-30.0, type=float)
    parser.add_argument("--occupied-z-min", default=-0.8, type=float)
    parser.add_argument("--occupied-z-max", default=0.3, type=float)
    parser.add_argument("--free-z-min", default=-1.30, type=float)
    parser.add_argument("--free-z-max", default=-0.35, type=float)
    parser.add_argument("--occupied-dilation", default=1, type=int)
    parser.add_argument("--no-ros", action="store_true", help="start only HTTP server")
    # ROS2 launch 会自动追加 --ros-args/-r 等参数；这里忽略未知参数，留给 rclpy 处理。
    args, _ = parser.parse_known_args(argv)
    return args


def main() -> None:
    args = parse_args()
    # 同一个 MappingSession 同时被 ROS 点云回调线程和 HTTP 请求线程共享。
    session = MappingSession(
        MappingConfig(
            output_dir=args.output_dir,
            resolution=args.resolution,
            size_x=args.size_x,
            size_y=args.size_y,
            origin_x=args.origin_x,
            origin_y=args.origin_y,
            occupied_z_min=args.occupied_z_min,
            occupied_z_max=args.occupied_z_max,
            free_z_min=args.free_z_min,
            free_z_max=args.free_z_max,
            occupied_dilation=args.occupied_dilation,
        )
    )
    server = run_http_server(session, args.host, args.port)
    if args.no_ros:
        try:
            print(f"mapping control HTTP: http://{args.host}:{args.port}")
            threading.Event().wait()
        except KeyboardInterrupt:
            server.shutdown()
        return
    run_ros(args, session)


if __name__ == "__main__":
    main()
