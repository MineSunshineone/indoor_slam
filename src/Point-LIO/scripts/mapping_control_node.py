#!/usr/bin/env python3
"""面向 App 对接的 ROS 2 建图控制节点。

该节点订阅 Point-LIO 发布的配准点云，并增量生成 Nav2 风格的 2D 栅格地图。
App 通过 ROS 2 action/service/topic 控制地图累积流程：开始、暂停/继续、取消、保存、查询状态。
"""

from __future__ import annotations

import argparse
import array
import json
import re
import subprocess
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable

import numpy as np


# 地图名会直接用于生成文件名，只允许安全字符，避免路径穿越和奇怪文件名。
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")

def sanitize_map_name(map_name: str) -> str:
    # 清理并校验地图名，确保后续可以安全拼成 .pgm/.yaml 文件路径。
    cleaned = map_name.strip()
    if not MAP_NAME_RE.fullmatch(cleaned):
        raise ValueError("map_name must be 1-64 chars: A-Z, a-z, 0-9, _ or -")
    return cleaned


def apply_rigid_transform(
    xyz: np.ndarray,
    *,
    translation: tuple[float, float, float],
    quaternion: tuple[float, float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    """Apply a map<-cloud rigid transform to an Nx3 point array."""
    qx, qy, qz, qw = quaternion
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("transform quaternion is invalid")
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),
             2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz),
             2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw),
             1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    transformed = np.asarray(xyz, dtype=np.float64) @ rotation.T + matrix[:3, 3]
    return transformed, matrix


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
    # 使用外部 PCD 转图程序生成导航地图和重定位点云。
    pcd2pgm_enabled: bool = True
    pcd2pgm_executable: Path = Path("pcd2pgm_headless")
    pcd2pgm_config: Path = Path("scans_nav2_map.cfg")
    pcd_input_path: Path = Path("src/Point-LIO/PCD/scans.pcd")
    relocalization_pcd_path: Path = Path("maps/PCD/scans.pcd")
    map_store_root: Path = Path("/var/lib/bxi/maps")
    pcd_merge_executable: Path = Path("merge_pcd_maps")


class MappingSession:
    """单次 App 建图任务的线程安全内存状态。"""

    def __init__(
        self,
        config: MappingConfig,
        command_runner: Callable[[list[str], Path], None] | None = None,
    ) -> None:
        self.config = config
        self.command_runner = command_runner or self._run_command
        # 根据物理尺寸和分辨率换算固定大小的二维栅格。
        self.width = int(round(config.size_x / config.resolution))
        self.height = int(round(config.size_y / config.resolution))
        # HTTP 请求线程和 ROS 点云回调线程会共享同一个 session，因此这里必须加锁。
        self.lock = threading.RLock()
        # 状态取值：idle/mapping/paused/saving/saved/cancelled。
        self.state = "idle"
        self.current_map_name = ""
        self.session_id = ""
        self.base_map_id = ""
        self.last_error = ""
        self.last_map_pgm_path = ""
        self.last_map_yaml_path = ""
        self.last_relocalization_pcd_path = ""
        self.map_from_cloud = np.eye(4, dtype=np.float64)
        self.base_cells: np.ndarray | None = None
        # 分别累计障碍物证据和空闲证据；保存时再融合成 Nav2 占据栅格。
        self.occ_counts = np.zeros((self.height, self.width), dtype=np.uint32)
        self.free_counts = np.zeros((self.height, self.width), dtype=np.uint32)

    def start(
        self,
        map_name: str,
        session_id: str = "",
        base_map_id: str = "",
    ) -> dict[str, Any]:
        with self.lock:
            # 开始新地图时清空未保存的旧累积结果，避免两次建图混在一起。
            self.current_map_name = sanitize_map_name(map_name)
            self.session_id = session_id.strip()
            self.base_map_id = base_map_id.strip()
            if self.base_map_id:
                sanitize_map_name(self.base_map_id)
                self._seed_from_parent_grid()
            else:
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
            if self.base_map_id:
                self._expand_to_include(xyz)
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

    def _expand_to_include(self, xyz: np.ndarray) -> None:
        cfg = self.config
        resolution = cfg.resolution
        current_max_x = cfg.origin_x + self.width * resolution
        current_max_y = cfg.origin_y + self.height * resolution
        min_x = min(cfg.origin_x, float(np.min(xyz[:, 0])))
        min_y = min(cfg.origin_y, float(np.min(xyz[:, 1])))
        max_x = max(current_max_x, float(np.max(xyz[:, 0])) + resolution)
        max_y = max(current_max_y, float(np.max(xyz[:, 1])) + resolution)
        new_origin_x = np.floor(min_x / resolution) * resolution
        new_origin_y = np.floor(min_y / resolution) * resolution
        new_width = int(np.ceil((max_x - new_origin_x) / resolution))
        new_height = int(np.ceil((max_y - new_origin_y) / resolution))
        if (
            new_width == self.width
            and new_height == self.height
            and np.isclose(new_origin_x, cfg.origin_x)
            and np.isclose(new_origin_y, cfg.origin_y)
        ):
            return
        x_offset = int(round((cfg.origin_x - new_origin_x) / resolution))
        y_offset = int(round((cfg.origin_y - new_origin_y) / resolution))
        old_slice = (
            slice(y_offset, y_offset + self.height),
            slice(x_offset, x_offset + self.width),
        )
        new_occ = np.zeros((new_height, new_width), dtype=np.uint32)
        new_free = np.zeros((new_height, new_width), dtype=np.uint32)
        new_occ[old_slice] = self.occ_counts
        new_free[old_slice] = self.free_counts
        self.occ_counts = new_occ
        self.free_counts = new_free
        if self.base_cells is not None:
            new_base = np.full((new_height, new_width), -1, dtype=np.int8)
            new_base[old_slice] = self.base_cells
            self.base_cells = new_base
        self.width = new_width
        self.height = new_height
        cfg.origin_x = float(new_origin_x)
        cfg.origin_y = float(new_origin_y)
        cfg.size_x = new_width * resolution
        cfg.size_y = new_height * resolution

    def set_map_from_cloud_transform(self, matrix: np.ndarray) -> None:
        with self.lock:
            if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                raise ValueError("map-from-cloud transform must be a finite 4x4 matrix")
            self.map_from_cloud = matrix.copy()

    def _seed_from_parent_grid(self) -> None:
        record_path = self.config.map_store_root / f"{self.base_map_id}.json"
        try:
            record = json.loads(record_path.read_text(encoding="utf-8"))
            grid = record["grid"]
            width = int(grid["width"])
            height = int(grid["height"])
            data = np.asarray(grid["data"], dtype=np.int16).reshape(height, width)
            resolution = float(grid["resolution"])
            origin_x = float(grid.get("origin_x", 0.0))
            origin_y = float(grid.get("origin_y", 0.0))
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"failed to load parent 2D grid {self.base_map_id}: {exc}"
            ) from exc
        if width <= 0 or height <= 0 or resolution <= 0:
            raise RuntimeError("parent 2D grid dimensions are invalid")
        self.width = width
        self.height = height
        self.config.resolution = resolution
        self.config.origin_x = origin_x
        self.config.origin_y = origin_y
        self.occ_counts = np.zeros((height, width), dtype=np.uint32)
        self.free_counts = np.zeros((height, width), dtype=np.uint32)
        self.base_cells = data.astype(np.int8, copy=True)

    def save(self, map_name: str | None = None) -> dict[str, Any]:
        with self.lock:
            # 保存时允许重新指定地图名，但仍必须满足安全文件名规则。
            if map_name is not None:
                self.current_map_name = sanitize_map_name(map_name)
            if not self.current_map_name:
                raise RuntimeError("map_name is required before saving")

            self.state = "saving"
            self.config.output_dir.mkdir(parents=True, exist_ok=True)
            try:
                if self.config.pcd2pgm_enabled:
                    paths = self._run_pcd2pgm_conversion()
                else:
                    paths = self._write_accumulated_grid()
            except Exception as exc:
                self.state = "error"
                self.last_error = str(exc)
                raise
            self.last_map_pgm_path = str(paths["map_pgm_path"])
            self.last_map_yaml_path = str(paths["map_yaml_path"])
            self.last_relocalization_pcd_path = str(paths["relocalization_pcd_path"])
            self.state = "saved"
            self.last_error = ""
            result = self.status()
            result["success"] = True
            return result

    def status(self) -> dict[str, Any]:
        # 返回给 App 的状态字段保持简单稳定，便于前端直接展示。
        with self.lock:
            cells_known, coverage_percent = self._grid_metrics()
            return {
                "state": self.state,
                "current_map_name": self.current_map_name,
                "map_pgm_path": self.last_map_pgm_path or (
                    str(self._pgm_path()) if self.current_map_name else ""
                ),
                "map_yaml_path": self.last_map_yaml_path or (
                    str(self._yaml_path()) if self.current_map_name else ""
                ),
                "relocalization_pcd_path": self.last_relocalization_pcd_path,
                "cells_known": cells_known,
                "coverage_percent": coverage_percent,
                "resolution": self.config.resolution,
                "width": self.width,
                "height": self.height,
                "last_error": self.last_error,
                "session_id": self.session_id,
                "base_map_id": self.base_map_id,
            }

    def _clear_counts(self) -> None:
        # 只清空累积证据，不改变地图参数。
        self.occ_counts.fill(0)
        self.free_counts.fill(0)
        self.base_cells = None

    def _pgm_path(self) -> Path:
        return self.config.output_dir / f"{self.current_map_name}.pgm"

    def _yaml_path(self) -> Path:
        return self.config.output_dir / f"{self.current_map_name}.yaml"

    def _write_accumulated_grid(self) -> dict[str, Path]:
        pgm_path = self._pgm_path()
        yaml_path = self._yaml_path()
        grid = self._build_grid()
        self._write_pgm(pgm_path, grid)
        self._write_yaml(yaml_path, pgm_path.name)
        return {
            "map_pgm_path": pgm_path,
            "map_yaml_path": yaml_path,
            "relocalization_pcd_path": Path(""),
        }

    def _run_pcd2pgm_conversion(self) -> dict[str, Path]:
        cfg = self.config
        executable = self._resolve_path(cfg.pcd2pgm_executable)
        config_path = self._resolve_path(cfg.pcd2pgm_config)
        pcd_input = self._resolve_path(cfg.pcd_input_path)
        output_root = self._resolve_path(cfg.output_dir)
        session_name = self.session_id or self.current_map_name
        session_name = sanitize_map_name(session_name)
        output_dir = output_root / ".staging" / session_name
        relocalization_pcd = output_dir / "map.pcd"
        output_dir.mkdir(parents=True, exist_ok=True)

        if self.base_map_id:
            base_pcd = cfg.map_store_root / self.base_map_id / "map.pcd"
            if not base_pcd.is_file():
                raise FileNotFoundError(f"parent PCD not found: {base_pcd}")
            merge_executable = self._resolve_path(cfg.pcd_merge_executable)
            if not merge_executable.is_file():
                raise FileNotFoundError(
                    f"PCD merge executable not found: {merge_executable}")
            merged_pcd = output_dir / "merged.pcd"
            transform_csv = ",".join(
                f"{value:.12g}" for value in self.map_from_cloud.reshape(-1)
            )
            self.command_runner(
                [
                    str(merge_executable),
                    "--base", str(base_pcd),
                    "--increment", str(pcd_input),
                    "--output", str(merged_pcd),
                    "--transform", transform_csv,
                    "--leaf", "0.10",
                ],
                output_dir,
            )
            pcd_input = merged_pcd

        if not pcd_input.exists():
            raise FileNotFoundError(f"Point-LIO PCD not found: {pcd_input}")
        if not config_path.exists():
            raise FileNotFoundError(f"pcd2pgm config not found: {config_path}")
        if not executable.exists():
            raise FileNotFoundError(f"pcd2pgm executable not found: {executable}")

        cmd = [
            str(executable),
            "--headless",
            str(pcd_input),
            "--config",
            str(config_path),
            "--save-point-cloud",
            str(relocalization_pcd),
        ]
        self.command_runner(cmd, output_dir)

        output_stem = self._pcd2pgm_output_stem(config_path)
        pgm_path = output_dir / f"{output_stem}.pgm"
        yaml_path = output_dir / f"{output_stem}.yaml"
        missing = [
            str(path)
            for path in (pgm_path, yaml_path, relocalization_pcd)
            if not path.exists()
        ]
        if missing:
            raise RuntimeError("pcd2pgm did not create expected output: " + ", ".join(missing))
        return {
            "map_pgm_path": pgm_path,
            "map_yaml_path": yaml_path,
            "relocalization_pcd_path": relocalization_pcd,
        }

    def _pcd2pgm_output_stem(self, config_path: Path) -> str:
        for raw_line in config_path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if key.strip() == "output_stem":
                stem = value.strip()
                if stem:
                    return sanitize_map_name(stem)
        return self.current_map_name

    def _resolve_path(self, path: Path) -> Path:
        return path if path.is_absolute() else (Path.cwd() / path)

    def _run_command(self, cmd: list[str], cwd: Path) -> None:
        try:
            subprocess.run(cmd, cwd=cwd, check=True, text=True, capture_output=True)
        except subprocess.CalledProcessError as exc:
            details = (exc.stderr or exc.stdout or str(exc)).strip()
            raise RuntimeError(f"pcd2pgm conversion failed: {details}") from exc

    def _build_grid(self) -> np.ndarray:
        # Nav2/ROS PGM 约定：0 表示占用，254 表示空闲，205 通常作为未知灰度。
        occ = self.occ_counts > 0
        free = self.free_counts > 0
        if self.base_cells is None:
            grid = np.full((self.height, self.width), 205, dtype=np.uint8)
        else:
            grid = np.full(self.base_cells.shape, 205, dtype=np.uint8)
            grid[(self.base_cells >= 0) & (self.base_cells < 65)] = 254
            grid[self.base_cells >= 65] = 0
        grid[free & (grid != 0)] = 254
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

    def occupancy_cells(self) -> np.ndarray:
        """当前累积栅格转成 Nav2 OccupancyGrid 语义 (int8): -1 未知 / 0 空闲 / 100 占用。

        行 0 对应 origin_y（行主序，y 向上递增），与 OccupancyGrid.data 布局一致，
        无需上下翻转（PGM 才需要翻）。
        """
        with self.lock:
            grid = self._build_grid()
        cells = np.full(grid.shape, -1, dtype=np.int8)
        cells[grid == 254] = 0
        cells[grid == 0] = 100
        return cells

    def _grid_metrics(self) -> tuple[int, float]:
        # 统计最终栅格中已知区域比例，和保存出来的 PGM 语义保持一致。
        grid = self._build_grid()
        cells_known = int(np.count_nonzero(grid != 205))
        total_cells = self.width * self.height
        coverage_percent = (cells_known / total_cells * 100.0) if total_cells else 0.0
        return cells_known, round(float(coverage_percent), 2)

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
                result = self.server.session.cancel()
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
    from bxi_nav_interfaces.action import BuildMap
    from bxi_nav_interfaces.msg import MappingStatus
    from bxi_nav_interfaces.srv import SaveMap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_srvs.srv import SetBool, Trigger
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class MappingControlNode(Node):
        """订阅 Point-LIO 点云，并提供 App 建图 action/service/topic。"""

        def __init__(self) -> None:
            super().__init__("mapping_control_node")
            self.callback_group = ReentrantCallbackGroup()
            self.goal_active = False
            self.active_goal_handle = None
            self.completed_build_result: dict[str, Any] | None = None
            self.build_done = threading.Event()
            self.build_lock = threading.RLock()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.last_tf_warning_at = 0.0
            cloud_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=5,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            status_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.status_pub = self.create_publisher(MappingStatus, "/mapping/status", status_qos)
            # 实时建图栅格 → /map（App 建图画面靠它，接口契约 §6.1/§8 要求
            # reliable + transient_local，volatile 会被网关的订阅静默不兼容）。
            # 仅在建图会话活跃时发布（1Hz，随 status 定时器），会话结束即销毁
            # 发布器——把 latched 帧从 DDS 撤掉，避免与 nav2 map_server 的静态
            # 先验图在 /map 上双 latch 打架。
            self.map_qos = status_qos
            self.map_pub = None
            self.create_subscription(PointCloud2, args.cloud_topic, self.on_cloud, cloud_qos)
            self.build_server = ActionServer(self, BuildMap, "/mapping/build",
                execute_callback=self.execute_build,
                goal_callback=self.on_build_goal,
                cancel_callback=self.on_build_cancel,
                callback_group=self.callback_group,
            )
            self.create_service(SetBool, "/mapping/pause", self.on_pause, callback_group=self.callback_group)
            self.create_service(SaveMap, "/mapping/save", self.on_save, callback_group=self.callback_group)
            self.point_lio_start_client = self.create_client(Trigger, "/point_lio/mapping/start",
                callback_group=self.callback_group)
            self.point_lio_pause_client = self.create_client(SetBool, "/point_lio/mapping/pause",
                callback_group=self.callback_group)
            self.point_lio_save_client = self.create_client(Trigger, "/point_lio/mapping/save",
                callback_group=self.callback_group)
            self.point_lio_cancel_client = self.create_client(Trigger, "/point_lio/mapping/cancel",
                callback_group=self.callback_group)
            self.create_timer(args.status_period, self.publish_status, callback_group=self.callback_group)
            self.get_logger().info(
                "mapping control ready: "
                f"build_action=/mapping/build, pause_service=/mapping/pause, "
                f"save_service=/mapping/save, status_topic=/mapping/status, "
                f"cloud_topic={args.cloud_topic}"
            )
            self.publish_status()

        def on_cloud(self, msg: PointCloud2) -> None:
            # 将 ROS PointCloud2 转成 Nx3 numpy 数组，再交给纯 Python 状态逻辑处理。
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
            if session.base_map_id:
                source_frame = msg.header.frame_id or "odom"
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "map", source_frame, Time())
                    t = transform.transform.translation
                    q = transform.transform.rotation
                    pts, matrix = apply_rigid_transform(
                        pts,
                        translation=(t.x, t.y, t.z),
                        quaternion=(q.x, q.y, q.z, q.w),
                    )
                    session.set_map_from_cloud_transform(matrix)
                except (TransformException, ValueError) as exc:
                    now = time.monotonic()
                    if now - self.last_tf_warning_at > 5.0:
                        self.get_logger().warning(
                            f"waiting for map<-{source_frame} transform: {exc}")
                        self.last_tf_warning_at = now
                    return
            session.ingest_xyz(pts)

        def on_build_goal(self, goal_request: BuildMap.Goal) -> GoalResponse:
            # 同一时间只允许一个建图会话，避免多个 App 客户端互相覆盖地图名和累计数据。
            try:
                sanitize_map_name(goal_request.map_name)
            except ValueError as exc:
                self.get_logger().error(f"reject mapping goal: {exc}")
                return GoalResponse.REJECT
            with self.build_lock:
                if self.goal_active:
                    self.get_logger().warn("reject mapping goal: another build goal is active")
                    return GoalResponse.REJECT
                self.goal_active = True
                return GoalResponse.ACCEPT

        def on_build_cancel(self, goal_handle) -> CancelResponse:
            # 取消 action 表示丢弃未保存数据；execute_build 会完成真正的清理和返回。
            return CancelResponse.ACCEPT

        def execute_build(self, goal_handle) -> BuildMap.Result:
            # build action 是一次建图会话：start 后持续 feedback，直到 save 成功或 action cancel。
            started_at = time.monotonic()
            with self.build_lock:
                self.active_goal_handle = goal_handle
                self.completed_build_result = None
                self.build_done.clear()

            try:
                self._call_point_lio_trigger(self.point_lio_start_client,
                    "start Point-LIO mapping accumulation")
                session.start(
                    goal_handle.request.map_name,
                    goal_handle.request.session_id,
                    goal_handle.request.base_map_id,
                )
            except Exception as exc:
                session.last_error = str(exc)
                self.get_logger().error(f"mapping build failed to start: {exc}")
                goal_handle.abort()
                self.publish_status()
                self._clear_active_goal()
                return self._build_action_result(False, str(exc), "", "", "")

            self.publish_status()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    try:
                        self._call_point_lio_trigger(self.point_lio_cancel_client,
                            "cancel Point-LIO mapping accumulation")
                    except Exception as exc:
                        session.last_error = str(exc)
                        self.get_logger().error(f"Point-LIO mapping cancel failed: {exc}")
                    session.cancel()
                    goal_handle.canceled()
                    self.publish_status()
                    result = self._build_action_result(
                        False, "mapping cancelled", "", "", "")
                    self._clear_active_goal()
                    return result

                with self.build_lock:
                    completed = self.completed_build_result
                if completed is not None:
                    goal_handle.succeed()
                    self.publish_status()
                    result = self._build_action_result(
                        bool(completed.get("success", False)),
                        "map saved",
                        str(completed.get("map_pgm_path", "")),
                        str(completed.get("map_yaml_path", "")),
                        str(completed.get("relocalization_pcd_path", "")),
                    )
                    self._clear_active_goal()
                    return result

                goal_handle.publish_feedback(self._build_feedback(started_at))
                self.publish_status()
                self.build_done.wait(args.status_period)

            goal_handle.abort()
            self._clear_active_goal()
            return self._build_action_result(False, "rclpy shutdown", "", "", "")

        def on_pause(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
            # true 暂停累计，false 继续累计；Point-LIO 本体仍照常运行。
            try:
                current = session.status()
                if not current["current_map_name"] or current["state"] in {"idle", "cancelled"}:
                    raise RuntimeError("no active mapping task")
                self._call_point_lio_set_bool(self.point_lio_pause_client, request.data,
                    "pause Point-LIO mapping accumulation" if request.data
                    else "resume Point-LIO mapping accumulation",
                )
                session.pause(request.data)
                response.success = True
                response.message = "paused" if request.data else "mapping"
            except Exception as exc:
                session.last_error = str(exc)
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f"mapping pause failed: {exc}")
            self.publish_status()
            return response

        def on_save(self, request: SaveMap.Request, response: SaveMap.Response) -> SaveMap.Response:
            # 保存成功后唤醒 build action，让 action result 返回最终文件路径。
            try:
                map_name = request.map_name.strip() or None
                if map_name is not None:
                    sanitize_map_name(map_name)
                self._call_point_lio_trigger(self.point_lio_save_client, "save Point-LIO PCD map")
                result = session.save(map_name)
                response.success = True
                response.message = "map saved"
                response.map_pgm_path = str(result["map_pgm_path"])
                response.map_yaml_path = str(result["map_yaml_path"])
                response.map_pcd_path = str(result["relocalization_pcd_path"])
                with self.build_lock:
                    self.completed_build_result = result
                    self.build_done.set()
            except Exception as exc:
                session.last_error = str(exc)
                response.success = False
                response.message = str(exc)
                response.map_pgm_path = ""
                response.map_yaml_path = ""
                response.map_pcd_path = ""
                self.get_logger().error(f"mapping save failed: {exc}")
            self.publish_status()
            return response

        def publish_status(self) -> None:
            data = session.status()
            status_msg = MappingStatus()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.header.frame_id = "map"
            status_msg.state = str(data["state"])
            status_msg.current_map_name = str(data["current_map_name"])
            status_msg.coverage_percent = float(data["coverage_percent"])
            status_msg.resolution = float(data["resolution"])
            status_msg.width = int(data["width"])
            status_msg.height = int(data["height"])
            status_msg.last_error = str(data["last_error"])
            status_msg.session_id = str(data["session_id"])
            status_msg.base_map_id = str(data["base_map_id"])
            self.status_pub.publish(status_msg)
            self.publish_live_map(str(data["state"]))

        def publish_live_map(self, state: str) -> None:
            # 建图/暂停/保存中把累积栅格发出去；其余状态销毁发布器撤掉 latch。
            if state in ("mapping", "paused", "saving"):
                if self.map_pub is None:
                    self.map_pub = self.create_publisher(
                        OccupancyGrid, args.map_topic, self.map_qos)
                msg = OccupancyGrid()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.info.resolution = float(session.config.resolution)
                msg.info.width = session.width
                msg.info.height = session.height
                msg.info.origin.position.x = float(session.config.origin_x)
                msg.info.origin.position.y = float(session.config.origin_y)
                msg.info.origin.orientation.w = 1.0
                # array('b') 直接吃 numpy int8 字节，绕过 36 万格的 Python 列表转换。
                msg.data = array.array("b", session.occupancy_cells().tobytes())
                self.map_pub.publish(msg)
            elif self.map_pub is not None:
                self.destroy_publisher(self.map_pub)
                self.map_pub = None

        def _call_point_lio_trigger(self, client, action_name: str) -> str:
            return self._call_point_lio_service(client, Trigger.Request(), action_name)

        def _call_point_lio_set_bool(self, client, value: bool, action_name: str) -> str:
            request = SetBool.Request()
            request.data = value
            return self._call_point_lio_service(client, request, action_name)

        def _call_point_lio_service(self, client, request, action_name: str) -> str:
            deadline = time.monotonic() + 10.0
            while rclpy.ok() and not client.wait_for_service(timeout_sec=0.1):
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout waiting to {action_name}")

            future = client.call_async(request)
            while rclpy.ok() and not future.done():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout trying to {action_name}")
                time.sleep(0.02)

            if not future.done():
                raise RuntimeError(f"ROS shutdown while trying to {action_name}")
            result = future.result()
            if result is None:
                raise RuntimeError(f"failed to {action_name}: empty response")
            if not result.success:
                message = result.message or "service returned success=false"
                raise RuntimeError(f"failed to {action_name}: {message}")
            return result.message

        def _build_feedback(self, started_at: float) -> BuildMap.Feedback:
            data = session.status()
            feedback = BuildMap.Feedback()
            feedback.state = str(data["state"])
            feedback.cells_known = int(data["cells_known"])
            feedback.coverage_percent = float(data["coverage_percent"])
            feedback.elapsed_sec = float(time.monotonic() - started_at)
            feedback.resolution = float(data["resolution"])
            feedback.width = int(data["width"])
            feedback.height = int(data["height"])
            return feedback

        def _build_action_result(
            self,
            success: bool,
            message: str,
            pgm_path: str,
            yaml_path: str,
            pcd_path: str,
        ) -> BuildMap.Result:
            result = BuildMap.Result()
            result.success = success
            result.message = message
            result.map_pgm_path = pgm_path
            result.map_yaml_path = yaml_path
            result.map_pcd_path = pcd_path
            return result

        def _clear_active_goal(self) -> None:
            with self.build_lock:
                self.goal_active = False
                self.active_goal_handle = None
                self.completed_build_result = None
                self.build_done.clear()

    rclpy.init()
    node = MappingControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    # 这些参数也会由 launch 文件透传，便于现场按地图范围和高度带调参。
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cloud-topic", default="/cloud_registered")
    parser.add_argument("--map-topic", default="/map",
                        help="建图期间实时占用栅格的发布话题 (latched, 1Hz)")
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
    parser.add_argument("--status-period", default=1.0, type=float)
    parser.add_argument("--pcd2pgm-executable", default="pcd2pgm_headless", type=Path)
    parser.add_argument("--pcd2pgm-config", default="scans_nav2_map.cfg", type=Path)
    parser.add_argument("--pcd-input-path", default="src/Point-LIO/PCD/scans.pcd", type=Path)
    parser.add_argument("--map-store-root", default="/var/lib/bxi/maps", type=Path)
    parser.add_argument("--pcd-merge-executable", default="merge_pcd_maps", type=Path)
    parser.add_argument(
        "--relocalization-pcd-path",
        default="maps/PCD/scans.pcd",
        type=Path,
    )
    parser.add_argument("--disable-pcd2pgm", action="store_true")
    parser.add_argument("--no-ros", action="store_true", help="start only HTTP server")
    parser.add_argument("--enable-http", action="store_true", help="also start deprecated HTTP server")
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
            pcd2pgm_enabled=not args.disable_pcd2pgm,
            pcd2pgm_executable=args.pcd2pgm_executable,
            pcd2pgm_config=args.pcd2pgm_config,
            pcd_input_path=args.pcd_input_path,
            relocalization_pcd_path=args.relocalization_pcd_path,
            map_store_root=args.map_store_root,
            pcd_merge_executable=args.pcd_merge_executable,
        )
    )
    if args.no_ros:
        server = run_http_server(session, args.host, args.port)
        try:
            print(f"mapping control HTTP: http://{args.host}:{args.port}")
            threading.Event().wait()
        except KeyboardInterrupt:
            server.shutdown()
        return
    server = run_http_server(session, args.host, args.port) if args.enable_http else None
    run_ros(args, session)
    if server is not None:
        server.shutdown()


if __name__ == "__main__":
    main()
