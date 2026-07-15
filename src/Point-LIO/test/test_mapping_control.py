import importlib.util
from pathlib import Path
import sys
from unittest.mock import Mock

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "mapping_control_node.py"
SPEC = importlib.util.spec_from_file_location("mapping_control_node", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_apply_rigid_transform_moves_cloud_into_parent_map_frame():
    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)
    half = np.sqrt(0.5)

    transformed, matrix = MODULE.apply_rigid_transform(
        points,
        translation=(10.0, 2.0, 0.0),
        quaternion=(0.0, 0.0, half, half),
    )

    np.testing.assert_allclose(transformed, [[10.0, 3.0, 0.0]], atol=1e-6)
    np.testing.assert_allclose(matrix @ [1.0, 0.0, 0.0, 1.0], [10, 3, 0, 1])


def test_parent_grid_seeds_live_occupancy(tmp_path):
    root = tmp_path / "maps"
    root.mkdir()
    (root / "parent.json").write_text(
        '{"grid":{"width":2,"height":1,"resolution":0.1,'
        '"origin_x":1.0,"origin_y":2.0,"data":[100,0]}}',
        encoding="utf-8",
    )
    session = MODULE.MappingSession(
        MODULE.MappingConfig(output_dir=tmp_path, map_store_root=root)
    )

    session.start("child", "session1", "parent")

    assert session.width == 2
    assert session.height == 1
    assert session.occupancy_cells().tolist() == [[100, 0]]


def test_extend_grid_grows_when_aligned_scan_exceeds_parent_bounds(tmp_path):
    root = tmp_path / "maps"
    root.mkdir()
    (root / "parent.json").write_text(
        '{"grid":{"width":1,"height":1,"resolution":1.0,'
        '"origin_x":0.0,"origin_y":0.0,"data":[0]}}',
        encoding="utf-8",
    )
    session = MODULE.MappingSession(
        MODULE.MappingConfig(
            output_dir=tmp_path,
            map_store_root=root,
            occupied_z_min=-1,
            occupied_z_max=1,
            occupied_dilation=0,
        )
    )
    session.start("child", "session1", "parent")

    session.ingest_xyz(np.array([[3.2, 0.2, 0.0]]))

    assert session.width >= 4
    assert 100 in session.occupancy_cells()


def test_cli_defaults_are_anchored_to_indoor_root(monkeypatch, tmp_path):
    indoor_root = tmp_path / "indoor"
    monkeypatch.setenv("BXI_INDOOR_SLAM_ROOT", str(indoor_root))

    args = MODULE.parse_args([])

    assert args.output_dir == indoor_root / "src" / "bxi_nav" / "maps"
    assert args.pcd2pgm_executable == indoor_root / "pcd2pgm_headless"
    assert args.pcd2pgm_config == indoor_root / "scans_nav2_map.cfg"
    assert args.pcd_input_path == indoor_root / "src" / "Point-LIO" / "PCD" / "scans.pcd"
    assert args.relocalization_pcd_path == indoor_root / "maps" / "PCD" / "scans.pcd"
    assert args.pcd_merge_executable == indoor_root / "merge_pcd_maps"
    assert args.robot_frame == "body_raw"


def test_cancel_clears_stale_session_status(tmp_path):
    session = MODULE.MappingSession(
        MODULE.MappingConfig(
            output_dir=tmp_path,
            occupied_z_min=-1,
            occupied_z_max=1,
            occupied_dilation=0,
        )
    )
    session.start("old_map", "old_session", "")
    session.ingest_xyz(np.array([[0.0, 0.0, 0.0]]))
    session.last_error = "old error"
    session.last_map_pgm_path = "/tmp/old.pgm"
    session.last_map_yaml_path = "/tmp/old.yaml"
    session.last_relocalization_pcd_path = "/tmp/old.pcd"

    status = session.cancel()

    assert status["state"] == "cancelled"
    assert status["current_map_name"] == ""
    assert status["map_pgm_path"] == ""
    assert status["map_yaml_path"] == ""
    assert status["relocalization_pcd_path"] == ""
    assert status["last_error"] == ""
    assert status["session_id"] == ""
    assert status["base_map_id"] == ""
    assert status["cells_known"] == 0


def test_clear_radius_removes_cells_inside_circle_and_keeps_outside(tmp_path):
    session = MODULE.MappingSession(
        MODULE.MappingConfig(
            output_dir=tmp_path,
            resolution=1.0,
            size_x=6.0,
            size_y=6.0,
            origin_x=0.0,
            origin_y=0.0,
            occupied_z_min=-1.0,
            occupied_z_max=1.0,
            occupied_dilation=0,
        )
    )
    session.start("map")
    session.ingest_xyz(np.array([[2.5, 2.5, 0.0], [5.5, 5.5, 0.0]]))

    cleared = session.clear_radius(2.5, 2.5, 1.1)

    assert cleared == 5
    assert session.occ_counts[2, 2] == 0
    assert session.occ_counts[5, 5] == 1


def test_clear_radius_removes_inherited_parent_cells(tmp_path):
    root = tmp_path / "maps"
    root.mkdir()
    (root / "parent.json").write_text(
        '{"grid":{"width":2,"height":1,"resolution":1.0,'
        '"origin_x":0.0,"origin_y":0.0,"data":[100,0]}}',
        encoding="utf-8",
    )
    session = MODULE.MappingSession(
        MODULE.MappingConfig(output_dir=tmp_path, map_store_root=root)
    )
    session.start("child", "session1", "parent")

    session.clear_radius(0.5, 0.5, 0.6)

    assert session.occupancy_cells().tolist() == [[-1, 0]]


def test_ros_shutdown_finishes_build_without_touching_action_handle():
    clear_active_goal = Mock()
    result_factory = Mock(return_value="shutdown-result")
    goal_handle = Mock()
    goal_handle.abort.side_effect = AssertionError("action context is already invalid")

    result = MODULE.finish_build_after_ros_shutdown(
        clear_active_goal=clear_active_goal,
        result_factory=result_factory,
    )

    assert result == "shutdown-result"
    clear_active_goal.assert_called_once_with()
    result_factory.assert_called_once_with(
        False, "rclpy shutdown", "", "", ""
    )
    goal_handle.abort.assert_not_called()


def test_ros_cleanup_tolerates_keyboard_interrupt_in_each_step():
    executor = Mock()
    node = Mock()
    ros = Mock()
    executor.shutdown.side_effect = KeyboardInterrupt
    node.destroy_node.side_effect = KeyboardInterrupt
    ros.ok.return_value = True
    ros.shutdown.side_effect = KeyboardInterrupt

    MODULE.best_effort_ros_cleanup(executor, node, ros)

    executor.shutdown.assert_called_once_with()
    node.destroy_node.assert_called_once_with()
    ros.shutdown.assert_called_once_with()
