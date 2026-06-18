from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
NAV_SRC = REPO_ROOT / "src" / "bxi_nav" / "src" / "main.cpp"
NAV_CMAKE = REPO_ROOT / "src" / "bxi_nav" / "CMakeLists.txt"
NAV_PACKAGE = REPO_ROOT / "src" / "bxi_nav" / "package.xml"
NAV_LAUNCH = REPO_ROOT / "src" / "bxi_nav" / "launch" / "indoor_navigation_launch.py"
START_SH = REPO_ROOT / "start.sh"


def test_nav_gateway_exposes_app_facing_interfaces():
    source = NAV_SRC.read_text()

    assert 'ActionServer<NavGoto>' in source
    assert 'ActionServer<FollowWaypoints>' in source
    assert 'create_service<SetInitialPose>(' in source
    assert '"/nav/init"' in source
    assert 'create_service<std_srvs::srv::SetBool>(' in source
    assert '"/nav/pause"' in source
    assert 'create_service<ClearTerrain>(' in source
    assert '"/mapping/clear_terrain"' in source
    assert 'create_publisher<NavPose>("/nav/pose"' in source
    assert 'create_publisher<NavStatus>("/nav/status"' in source


def test_nav_gateway_bridges_to_nav2_and_internal_topics():
    source = NAV_SRC.read_text()

    assert 'create_client<NavigateToPose>(this, "/navigate_to_pose"' in source
    assert 'create_client<NavigateThroughPoses>(this, "/navigate_through_poses"' in source
    assert 'create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose"' in source
    assert 'create_publisher<std_msgs::msg::Float32>("/map_clearing"' in source


def test_nav_build_and_launch_include_gateway_dependencies():
    cmake = NAV_CMAKE.read_text()
    package_xml = NAV_PACKAGE.read_text()
    launch = NAV_LAUNCH.read_text()

    assert "bxi_nav_interfaces" in cmake
    assert "rclcpp_action" in cmake
    assert "<depend>bxi_nav_interfaces</depend>" in package_xml
    assert "<depend>rclcpp_action</depend>" in package_xml
    assert "executable='indoor_nav_goal'" in launch or 'executable="indoor_nav_goal"' in launch


def test_start_script_runs_rosbridge_websocket():
    start_script = START_SH.read_text()

    assert "rosbridge_server" in start_script
    assert "rosbridge_websocket_launch.xml" in start_script
