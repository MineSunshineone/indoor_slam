from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
PKG = REPO_ROOT / "src" / "bxi_nav_interfaces"


def test_bxi_nav_interfaces_package_exists_with_expected_idl_files():
    expected_files = [
        "srv/SetInitialPose.srv",
        "srv/SaveMap.srv",
        "srv/ClearTerrain.srv",
        "action/NavGoto.action",
        "action/BuildMap.action",
        "action/FollowWaypoints.action",
        "msg/NavPose.msg",
        "msg/NavStatus.msg",
        "msg/MappingStatus.msg",
    ]

    for relative in expected_files:
        assert (PKG / relative).exists(), relative


def test_bxi_nav_interfaces_build_files_generate_rosidl_interfaces():
    cmake = (PKG / "CMakeLists.txt").read_text()
    package_xml = (PKG / "package.xml").read_text()

    assert "rosidl_generate_interfaces" in cmake
    assert "action/BuildMap.action" in cmake
    assert "srv/SaveMap.srv" in cmake
    assert "<member_of_group>rosidl_interface_packages</member_of_group>" in package_xml


def test_build_map_action_matches_app_contract():
    text = (PKG / "action" / "BuildMap.action").read_text()

    assert "string map_name" in text
    assert "string map_pgm_path" in text
    assert "string map_yaml_path" in text
    assert "float32 coverage_percent" in text
