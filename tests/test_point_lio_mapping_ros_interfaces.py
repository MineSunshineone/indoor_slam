import importlib.util
import sys
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = REPO_ROOT / "src" / "Point-LIO" / "scripts" / "mapping_control_node.py"
LAUNCH_PATH = (
    REPO_ROOT / "src" / "Point-LIO" / "launch" / "point_lio_with_mapping_control.launch.py"
)
POINT_LIO_PACKAGE_XML = REPO_ROOT / "src" / "Point-LIO" / "package.xml"


def load_mapping_control_module():
    spec = importlib.util.spec_from_file_location("mapping_control_node", NODE_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_mapping_session_reports_known_cell_coverage():
    module = load_mapping_control_module()
    session = module.MappingSession(
        module.MappingConfig(
            output_dir=Path("/tmp/maps"),
            resolution=1.0,
            size_x=2.0,
            size_y=2.0,
            origin_x=-1.0,
            origin_y=-1.0,
            occupied_dilation=0,
        )
    )

    session.start("floor_1")
    session.ingest_xyz(np.array([[0.0, 0.0, -0.5], [0.0, 0.0, 0.0]], dtype=float))

    status = session.status()
    assert status["cells_known"] == 1
    assert status["coverage_percent"] == 25.0


def test_mapping_control_node_uses_custom_ros_interfaces():
    text = NODE_PATH.read_text()

    assert "from rclpy.action import ActionServer" in text
    assert "from bxi_nav_interfaces.action import BuildMap" in text
    assert "from bxi_nav_interfaces.msg import MappingStatus" in text
    assert "from bxi_nav_interfaces.srv import SaveMap" in text
    assert 'ActionServer(self, BuildMap, "/mapping/build"' in text
    assert 'self.create_service(SetBool, "/mapping/pause"' in text
    assert 'self.create_service(SaveMap, "/mapping/save"' in text
    assert 'self.create_publisher(MappingStatus, "/mapping/status"' in text


def test_point_lio_declares_runtime_dependencies_for_mapping_interfaces():
    package_xml = POINT_LIO_PACKAGE_XML.read_text()
    launch_text = LAUNCH_PATH.read_text()

    assert "<depend>bxi_nav_interfaces</depend>" in package_xml
    assert "<depend>std_srvs</depend>" in package_xml
    assert "--status-period" in launch_text
    assert "mapping_http_host" not in launch_text
    assert "mapping_http_port" not in launch_text
