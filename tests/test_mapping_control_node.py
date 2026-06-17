import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "Point-LIO"
    / "scripts"
    / "mapping_control_node.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("mapping_control_node", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_map_name_is_sanitized_for_file_paths():
    module = load_module()

    assert module.sanitize_map_name(" floor-1_A ") == "floor-1_A"

    with pytest.raises(ValueError, match="map_name"):
        module.sanitize_map_name("../floor")

    with pytest.raises(ValueError, match="map_name"):
        module.sanitize_map_name("")


def test_start_pause_resume_and_cancel_control_accumulation(tmp_path):
    module = load_module()
    session = module.MappingSession(
        module.MappingConfig(
            output_dir=tmp_path,
            resolution=1.0,
            size_x=4.0,
            size_y=4.0,
            origin_x=0.0,
            origin_y=0.0,
            occupied_z_min=0.2,
            occupied_z_max=1.0,
            free_z_min=-0.2,
            free_z_max=0.1,
            occupied_dilation=0,
        )
    )

    session.start("floor_1")
    session.ingest_xyz(np.array([[1.2, 1.2, 0.5], [2.2, 1.2, 0.0]], dtype=float))
    assert session.status()["state"] == "mapping"
    assert int(session.occ_counts.sum()) == 1
    assert int(session.free_counts.sum()) == 1

    session.pause(True)
    session.ingest_xyz(np.array([[1.2, 1.2, 0.5]], dtype=float))
    assert session.status()["state"] == "paused"
    assert int(session.occ_counts.sum()) == 1

    session.pause(False)
    session.ingest_xyz(np.array([[1.2, 1.2, 0.5]], dtype=float))
    assert session.status()["state"] == "mapping"
    assert int(session.occ_counts.sum()) == 2

    session.cancel()
    assert session.status()["state"] == "cancelled"
    assert int(session.occ_counts.sum()) == 0
    assert int(session.free_counts.sum()) == 0


def test_save_writes_nav2_map_files_and_status(tmp_path):
    module = load_module()
    session = module.MappingSession(
        module.MappingConfig(
            output_dir=tmp_path,
            resolution=0.5,
            size_x=2.0,
            size_y=2.0,
            origin_x=-1.0,
            origin_y=-1.0,
            occupied_z_min=0.2,
            occupied_z_max=1.0,
            free_z_min=-0.2,
            free_z_max=0.1,
            occupied_dilation=0,
        )
    )

    session.start("floor_2")
    session.ingest_xyz(np.array([[-0.5, -0.5, 0.0], [0.25, 0.25, 0.5]], dtype=float))
    result = session.save()

    pgm_path = tmp_path / "floor_2.pgm"
    yaml_path = tmp_path / "floor_2.yaml"
    assert result["success"] is True
    assert result["current_map_name"] == "floor_2"
    assert result["map_pgm_path"] == str(pgm_path)
    assert result["map_yaml_path"] == str(yaml_path)
    assert pgm_path.read_bytes().startswith(b"P5\n4 4\n255\n")
    assert "image: floor_2.pgm" in yaml_path.read_text()
    assert "resolution: 0.500000" in yaml_path.read_text()
    assert "origin: [-1.000000, -1.000000, 0.000000]" in yaml_path.read_text()
    assert session.status()["state"] == "saved"


def test_parse_args_accepts_ros_launch_arguments():
    module = load_module()

    args = module.parse_args(
        [
            "--cloud-topic",
            "/cloud_registered",
            "--host",
            "0.0.0.0",
            "--port",
            "8088",
            "--ros-args",
            "-r",
            "__node:=mapping_control_node",
            "-r",
            "__ns:=/",
        ]
    )

    assert args.cloud_topic == "/cloud_registered"
    assert args.host == "0.0.0.0"
    assert args.port == 8088
