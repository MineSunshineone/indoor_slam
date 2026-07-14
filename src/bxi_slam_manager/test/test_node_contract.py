import ast
from pathlib import Path


NODE_PATH = (
    Path(__file__).resolve().parents[1]
    / "bxi_slam_manager"
    / "node.py"
)


def _assigned_self_attributes_for_call(method_name: str) -> set[str]:
    tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
    assigned: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not isinstance(node.value, ast.Call):
            continue
        func = node.value.func
        if not (
            isinstance(func, ast.Attribute)
            and isinstance(func.value, ast.Name)
            and func.value.id == "self"
            and func.attr == method_name
        ):
            continue
        for target in node.targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                assigned.add(target.attr)
    return assigned


def _assigned_self_attributes_for_self_call(method_name: str) -> set[str]:
    tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
    assigned: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not isinstance(node.value, ast.Call):
            continue
        func = node.value.func
        if not (
            isinstance(func, ast.Attribute)
            and isinstance(func.value, ast.Name)
            and func.value.id == "self"
            and func.attr == method_name
        ):
            continue
        for target in node.targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            ):
                assigned.add(target.attr)
    return assigned


def _function_node(name: str) -> ast.FunctionDef:
    tree = ast.parse(NODE_PATH.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"function {name!r} not found")


def _called_self_methods(function_name: str) -> set[str]:
    function = _function_node(function_name)
    calls: set[str] = set()
    for node in ast.walk(function):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (
            isinstance(func, ast.Attribute)
            and isinstance(func.value, ast.Name)
            and func.value.id == "self"
        ):
            calls.add(func.attr)
    return calls


def test_ros_entity_handles_are_kept_alive_on_node_instance() -> None:
    """rclpy entities must have explicit long-lived handles on the node."""

    assert {
        "_set_mode_srv",
        "_relocalization_sub",
        "_lidar_sub",
    } <= (
        _assigned_self_attributes_for_call("create_subscription")
        | _assigned_self_attributes_for_call("create_service")
        | _assigned_self_attributes_for_self_call("_create_lidar_subscription")
    )
    assert "_health_timer" in _assigned_self_attributes_for_call("create_timer")


def test_lidar_subscription_uses_reliable_qos_and_can_be_recreated() -> None:
    """The Livox publisher is reliable; the manager must not stay stuck after a stream reset."""

    source = ast.unparse(_function_node("_create_lidar_subscription"))
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "_refresh_lidar_subscription_if_needed" in _called_self_methods("_health_tick")


def test_lidar_subscription_refresh_destroys_and_recreates_handle() -> None:
    """Refreshing the LiDAR subscription must replace the long-lived handle."""

    calls = _called_self_methods("_refresh_lidar_subscription_if_needed")
    assert "destroy_subscription" in calls
    assert "_create_lidar_subscription" in calls
