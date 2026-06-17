from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    mapping_control = LaunchConfiguration("mapping_control")
    mapping_cloud_topic = LaunchConfiguration("mapping_cloud_topic")
    mapping_http_host = LaunchConfiguration("mapping_http_host")
    mapping_http_port = LaunchConfiguration("mapping_http_port")
    mapping_output_dir = LaunchConfiguration("mapping_output_dir")
    mapping_resolution = LaunchConfiguration("mapping_resolution")
    mapping_size_x = LaunchConfiguration("mapping_size_x")
    mapping_size_y = LaunchConfiguration("mapping_size_y")
    mapping_origin_x = LaunchConfiguration("mapping_origin_x")
    mapping_origin_y = LaunchConfiguration("mapping_origin_y")

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
        description="Whether to start the HTTP mapping control node",
    )
    declare_mapping_cloud_topic = DeclareLaunchArgument(
        "mapping_cloud_topic",
        default_value="/cloud_registered",
        description="Point cloud topic accumulated by the mapping control node",
    )
    declare_mapping_http_host = DeclareLaunchArgument(
        "mapping_http_host",
        default_value="0.0.0.0",
        description="HTTP bind host for App mapping control",
    )
    declare_mapping_http_port = DeclareLaunchArgument(
        "mapping_http_port",
        default_value="8088",
        description="HTTP bind port for App mapping control",
    )
    declare_mapping_output_dir = DeclareLaunchArgument(
        "mapping_output_dir",
        default_value="src/bxi_nav/maps",
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
            "--host",
            mapping_http_host,
            "--port",
            mapping_http_port,
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
            declare_mapping_http_host,
            declare_mapping_http_port,
            declare_mapping_output_dir,
            declare_mapping_resolution,
            declare_mapping_size_x,
            declare_mapping_size_y,
            declare_mapping_origin_x,
            declare_mapping_origin_y,
            start_point_lio_node,
            start_mapping_control_node,
            start_rviz_node,
        ]
    )
