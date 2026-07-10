import importlib.util
from pathlib import Path
import sys

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
