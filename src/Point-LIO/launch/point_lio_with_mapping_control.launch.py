import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static"), ("path", "/debug/path")]
    indoor_root = os.environ.get("BXI_INDOOR_SLAM_ROOT", "/opt/bxi/indoor")
    point_lio_source_dir = os.environ.get(
        "BXI_POINT_LIO_SOURCE_DIR",
        os.path.join(indoor_root, "src", "Point-LIO"),
    )

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    mapping_control = LaunchConfiguration("mapping_control")
    mapping_cloud_topic = LaunchConfiguration("mapping_cloud_topic")
    mapping_output_dir = LaunchConfiguration("mapping_output_dir")
    mapping_resolution = LaunchConfiguration("mapping_resolution")
    mapping_size_x = LaunchConfiguration("mapping_size_x")
    mapping_size_y = LaunchConfiguration("mapping_size_y")
    mapping_origin_x = LaunchConfiguration("mapping_origin_x")
    mapping_origin_y = LaunchConfiguration("mapping_origin_y")
    mapping_status_period = LaunchConfiguration("mapping_status_period")
    pcd2pgm_executable = LaunchConfiguration("pcd2pgm_executable")
    pcd2pgm_config = LaunchConfiguration("pcd2pgm_config")
    mapping_pcd_input_path = LaunchConfiguration("mapping_pcd_input_path")
    relocalization_pcd_path = LaunchConfiguration("relocalization_pcd_path")
    map_store_root = LaunchConfiguration("map_store_root")
    pcd_merge_executable = LaunchConfiguration("pcd_merge_executable")

    point_lio_dir = get_package_share_directory("point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for Point-LIO and mapping control nodes",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )
    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )
    declare_mapping_control = DeclareLaunchArgument(
        "mapping_control",
        default_value="True",
        description="Whether to start the mapping control node",
    )
    declare_mapping_cloud_topic = DeclareLaunchArgument(
        "mapping_cloud_topic",
        default_value="/cloud_registered",
        description="Point cloud topic accumulated by the mapping control node",
    )
    declare_mapping_output_dir = DeclareLaunchArgument(
        "mapping_output_dir",
        default_value=os.path.join(indoor_root, "src", "bxi_nav", "maps"),
        description="Directory where saved .pgm and .yaml maps are written",
    )
    declare_mapping_resolution = DeclareLaunchArgument(
        "mapping_resolution",
        default_value="0.10",
        description="Saved occupancy map resolution in meters per cell",
    )
    declare_mapping_size_x = DeclareLaunchArgument(
        "mapping_size_x",
        default_value="60.0",
        description="Saved occupancy map width in meters",
    )
    declare_mapping_size_y = DeclareLaunchArgument(
        "mapping_size_y",
        default_value="60.0",
        description="Saved occupancy map height in meters",
    )
    declare_mapping_origin_x = DeclareLaunchArgument(
        "mapping_origin_x",
        default_value="-30.0",
        description="Saved occupancy map origin x in meters",
    )
    declare_mapping_origin_y = DeclareLaunchArgument(
        "mapping_origin_y",
        default_value="-30.0",
        description="Saved occupancy map origin y in meters",
    )
    declare_mapping_status_period = DeclareLaunchArgument(
        "mapping_status_period",
        default_value="1.0",
        description="Seconds between /mapping/status publications",
    )
    declare_pcd2pgm_executable = DeclareLaunchArgument(
        "pcd2pgm_executable",
        default_value=os.path.join(indoor_root, "pcd2pgm_headless"),
        description="Executable used to convert Point-LIO PCD into Nav2 map files",
    )
    declare_pcd2pgm_config = DeclareLaunchArgument(
        "pcd2pgm_config",
        default_value=os.path.join(indoor_root, "scans_nav2_map.cfg"),
        description="pcd2pgm_headless config file",
    )
    declare_mapping_pcd_input_path = DeclareLaunchArgument(
        "mapping_pcd_input_path",
        default_value=os.path.join(point_lio_source_dir, "PCD", "scans.pcd"),
        description="Point-LIO PCD file converted when /mapping/save is called",
    )
    declare_relocalization_pcd_path = DeclareLaunchArgument(
        "relocalization_pcd_path",
        default_value=os.path.join(indoor_root, "maps", "PCD", "scans.pcd"),
        description="Edited PCD output used by small_gicp relocalization",
    )
    declare_map_store_root = DeclareLaunchArgument(
        "map_store_root",
        default_value="/var/lib/bxi/maps",
        description="Robot-side versioned map bundle root",
    )
    declare_pcd_merge_executable = DeclareLaunchArgument(
        "pcd_merge_executable",
        default_value=os.path.join(
            get_package_prefix("point_lio"), "lib", "point_lio", "merge_pcd_maps"
        ),
        description="Executable that merges a parent PCD with an aligned increment",
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[point_lio_cfg_dir],
        remappings=remappings,
        output="screen",
    )

    start_mapping_control_node = Node(
        condition=IfCondition(mapping_control),
        package="point_lio",
        executable="mapping_control_node.py",
        namespace=namespace,
        name="mapping_control_node",
        output="screen",
        arguments=[
            "--cloud-topic",
            mapping_cloud_topic,
            "--output-dir",
            mapping_output_dir,
            "--resolution",
            mapping_resolution,
            "--size-x",
            mapping_size_x,
            "--size-y",
            mapping_size_y,
            "--origin-x",
            mapping_origin_x,
            "--origin-y",
            mapping_origin_y,
            "--status-period",
            mapping_status_period,
            "--pcd2pgm-executable",
            pcd2pgm_executable,
            "--pcd2pgm-config",
            pcd2pgm_config,
            "--pcd-input-path",
            mapping_pcd_input_path,
            "--relocalization-pcd-path",
            relocalization_pcd_path,
            "--map-store-root",
            map_store_root,
            "--pcd-merge-executable",
            pcd_merge_executable,
        ],
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox"]),
            ".rviz",
        ],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_rviz,
            declare_point_lio_cfg_dir,
            declare_mapping_control,
            declare_mapping_cloud_topic,
            declare_mapping_output_dir,
            declare_mapping_resolution,
            declare_mapping_size_x,
            declare_mapping_size_y,
            declare_mapping_origin_x,
            declare_mapping_origin_y,
            declare_mapping_status_period,
            declare_pcd2pgm_executable,
            declare_pcd2pgm_config,
            declare_mapping_pcd_input_path,
            declare_relocalization_pcd_path,
            declare_map_store_root,
            declare_pcd_merge_executable,
            start_point_lio_node,
            start_mapping_control_node,
            start_rviz_node,
        ]
    )
