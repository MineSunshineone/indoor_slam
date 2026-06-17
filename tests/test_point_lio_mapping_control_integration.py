from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
POINT_LIO = REPO_ROOT / "src" / "Point-LIO"


def test_mapping_control_script_is_installed_by_point_lio_package():
    cmake = (POINT_LIO / "CMakeLists.txt").read_text()

    assert "scripts/mapping_control_node.py" in cmake
    assert "DESTINATION lib/${PROJECT_NAME}" in cmake


def test_combined_launch_starts_point_lio_and_mapping_control():
    launch_path = POINT_LIO / "launch" / "point_lio_with_mapping_control.launch.py"
    launch_text = launch_path.read_text()

    assert 'executable="pointlio_mapping"' in launch_text
    assert 'executable="mapping_control_node.py"' in launch_text
    assert '"--cloud-topic"' in launch_text
    assert '"--output-dir"' in launch_text
    assert '"--port"' in launch_text


def test_start_script_uses_combined_point_lio_launch():
    start_script = (REPO_ROOT / "start.sh").read_text()

    assert "ros2 launch point_lio point_lio_with_mapping_control.launch.py" in start_script
