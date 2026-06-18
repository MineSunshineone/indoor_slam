# Indoor SLAM App 对接接口文档

本文档只列出当前系统已有的 ROS 2 接口，供 App 侧或中间层对接时查阅。

## 1. 基础信息

| 项目 | 值 |
| --- | --- |
| ROS 版本 | ROS 2 Humble |
| 默认 ROS_DOMAIN_ID | `37` |
| 主坐标系 | `map` |
| 里程计坐标系 | `odom` |
| 机器人坐标系 | `base_link` |
| 默认地图 | `src/bxi_nav/maps/maps.yaml` |

启动完整系统：

```bash
./start.sh
```

手动启动导航：

```bash
source install/setup.bash
ros2 launch nav indoor_navigation_launch.py
```

## 2. 地图数据

| 项目 | 值 |
| --- | --- |
| Topic | `/map` |
| 类型 | `nav_msgs/msg/OccupancyGrid` |
| 发布源 | `nav2_map_server` |
| 坐标系 | `map` |
| 用途 | 静态 2D 栅格地图 |

查看地图信息：

```bash
ros2 topic echo --once /map --field info
```

`OccupancyGrid` 关键字段：

| 字段 | 说明 |
| --- | --- |
| `header.frame_id` | 地图坐标系，通常为 `map` |
| `info.resolution` | 分辨率，单位 m/cell |
| `info.width` | 地图宽度，单位 cell |
| `info.height` | 地图高度，单位 cell |
| `info.origin` | 地图左下角在 `map` 坐标系下的位姿 |
| `data` | 一维栅格数组，长度为 `width * height` |

栅格值：

| 值 | 含义 |
| --- | --- |
| `-1` | 未知 |
| `0` | 空闲 |
| `100` | 占用 / 障碍 |

坐标换算：

```text
index = y_cell * width + x_cell
map_x = origin.position.x + x_cell * resolution
map_y = origin.position.y + y_cell * resolution
```

## 3. 发送导航目标

| 项目 | 值 |
| --- | --- |
| Action | `/navigate_to_pose` |
| 类型 | `nav2_msgs/action/NavigateToPose` |
| 坐标系 | `map` |
| 用途 | 发送单点导航目标 |

命令行示例：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.2, y: 0.5, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  },
  behavior_tree: ''
}" --feedback
```

目标方向使用四元数。只设置 yaw 时：

```text
orientation.x = 0
orientation.y = 0
orientation.z = sin(yaw / 2)
orientation.w = cos(yaw / 2)
```

## 4. 取消导航

| 项目 | 值 |
| --- | --- |
| Action | `/navigate_to_pose` |
| 操作 | cancel goal |
| 用途 | 取消当前导航目标 |

命令行示例：

```bash
ros2 action cancel /navigate_to_pose
```

## 5. 导航状态

导航状态来自 `/navigate_to_pose` action 的 goal status、feedback 和 result。

查看 action 信息：

```bash
ros2 action info /navigate_to_pose
```

发送目标并查看反馈：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 1.2, y: 0.5, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    }
  }
}" --feedback
```

常用反馈字段：

| 字段 | 说明 |
| --- | --- |
| `current_pose` | 当前位姿 |
| `navigation_time` | 已导航时间 |
| `estimated_time_remaining` | 预计剩余时间 |
| `distance_remaining` | 剩余距离 |
| `number_of_recoveries` | 恢复行为次数 |

## 6. 机器人当前位置

| 项目 | 值 |
| --- | --- |
| Topic | `/aft_mapped_to_init` |
| 类型 | `nav_msgs/msg/Odometry` |
| 发布源 | Point-LIO |
| 用途 | 机器人当前位置和姿态 |

命令行示例：

```bash
ros2 topic echo --once /aft_mapped_to_init
```

常用字段：

| 字段 | 说明 |
| --- | --- |
| `pose.pose.position.x` | x 坐标，单位 m |
| `pose.pose.position.y` | y 坐标，单位 m |
| `pose.pose.position.z` | z 坐标，单位 m |
| `pose.pose.orientation` | 姿态四元数 |
| `twist.twist` | 速度信息 |

## 7. 初始化定位

| 项目 | 值 |
| --- | --- |
| Topic | `/initialpose` |
| 类型 | `geometry_msgs/msg/PoseWithCovarianceStamped` |
| 坐标系 | `map` |
| 用途 | 给重定位节点设置初始位姿 |

命令行示例：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
"{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0, 0, 0, 0, 0,
                 0, 0.25, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0,
                 0, 0, 0, 0, 0, 0.0685]
  }
}"
```

## 8. 清除局部地形点云

| 项目 | 值 |
| --- | --- |
| Topic | `/map_clearing` |
| 类型 | `std_msgs/msg/Float32` |
| 单位 | m |
| 用途 | 发布清除半径，触发地形点云清理 |

命令行示例：

```bash
ros2 topic pub --once /map_clearing std_msgs/msg/Float32 "{data: 8.0}"
```

## 9. 点云、路径与调试数据

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Point-LIO 配准点云 |
| `/terrain_map` | `sensor_msgs/msg/PointCloud2` | 地形 / 障碍点云 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 点云转 2D 激光，供 Nav2 局部代价地图使用 |
| `/path` | `nav_msgs/msg/Path` | Point-LIO 轨迹 |
| `/plan` | `nav_msgs/msg/Path` | Nav2 全局规划路径 |
| `/local_plan` | `nav_msgs/msg/Path` | 本项目局部 A* 路径调试输出 |
| `/free_paths` | `sensor_msgs/msg/PointCloud2` | 可通行候选路径点云 |

## 10. 建图控制

建图控制由 `point_lio` 包内的 `mapping_control_node.py` 提供。该节点订阅 Point-LIO 的 `/cloud_registered` 点云，并在 App 发送控制话题后累积生成 2D 栅格地图；暂停时 Point-LIO 仍继续运行，但不再累积地图；取消会清空当前未保存地图；保存会生成 Nav2 可直接加载的 `.pgm` 和 `.yaml`。

推荐启动方式：使用集成 launch 同时启动 Point-LIO 和建图控制节点。

```bash
source install/setup.bash
ros2 launch point_lio point_lio_with_mapping_control.launch.py
```

默认输出目录：

```text
src/bxi_nav/maps
```

可选参数：

```bash
ros2 launch point_lio point_lio_with_mapping_control.launch.py \
  mapping_http_host:=0.0.0.0 \
  mapping_http_port:=8088 \
  mapping_cloud_topic:=/cloud_registered \
  mapping_output_dir:=src/bxi_nav/maps \
  mapping_resolution:=0.10 \
  mapping_size_x:=60.0 \
  mapping_size_y:=60.0 \
  mapping_origin_x:=-30.0 \
  mapping_origin_y:=-30.0 \
  mapping_status_period:=1.0
```

地图名限制：`map_name` 只能包含英文字母、数字、下划线和短横线，长度 1-64，例如 `floor_1`。

### 10.1 ROS Topic 对接（推荐）

| Topic | 类型 | 方向 | QoS | 用途 |
| --- | --- | --- | --- | --- |
| `/mapping/start` | `std_msgs/msg/String` | App 发布 | Reliable / Volatile / KeepLast(10) | 开始建图；`data` 填地图名 |
| `/mapping/pause` | `std_msgs/msg/Bool` | App 发布 | Reliable / Volatile / KeepLast(10) | `true` 暂停累积，`false` 继续累积 |
| `/mapping/cancel` | `std_msgs/msg/Empty` | App 发布 | Reliable / Volatile / KeepLast(10) | 取消当前建图并清空未保存数据 |
| `/mapping/save` | `std_msgs/msg/String` | App 发布 | Reliable / Volatile / KeepLast(10) | 保存地图；`data` 为空时使用当前地图名 |
| `/mapping/status` | `std_msgs/msg/String` | App 订阅 | Reliable / Transient Local / KeepLast(1) | JSON 状态，包含当前地图名和保存路径 |

点云输入 `/cloud_registered` 使用 SensorData 风格 QoS：Best Effort / Volatile / KeepLast(5)，用于匹配 Point-LIO 高频点云发布。

开始建图：

```bash
ros2 topic pub --once /mapping/start std_msgs/msg/String "{data: 'floor_1'}"
```

暂停建图：

```bash
ros2 topic pub --once /mapping/pause std_msgs/msg/Bool "{data: true}"
```

继续建图：

```bash
ros2 topic pub --once /mapping/pause std_msgs/msg/Bool "{data: false}"
```

取消建图：

```bash
ros2 topic pub --once /mapping/cancel std_msgs/msg/Empty "{}"
```

保存地图：

```bash
ros2 topic pub --once /mapping/save std_msgs/msg/String "{data: 'floor_1'}"
```

如果保存时不想改名，可以发布空字符串，节点会使用 `/mapping/start` 时设置的当前地图名：

```bash
ros2 topic pub --once /mapping/save std_msgs/msg/String "{data: ''}"
```

查看状态：

```bash
ros2 topic echo /mapping/status
```

`/mapping/status` 的 `data` 是 JSON 字符串，示例：

```json
{
  "state": "mapping",
  "current_map_name": "floor_1",
  "map_pgm_path": "src/bxi_nav/maps/floor_1.pgm",
  "map_yaml_path": "src/bxi_nav/maps/floor_1.yaml",
  "resolution": 0.1,
  "width": 600,
  "height": 600,
  "last_error": ""
}
```

状态含义：

| 状态 | 含义 |
| --- | --- |
| `idle` | 控制节点已启动，但未开始建图 |
| `mapping` | 正在累积地图 |
| `paused` | 已暂停地图累积 |
| `saving` | 正在保存地图 |
| `saved` | 地图已保存 |
| `cancelled` | 当前建图已取消，未保存数据已清空 |
| `error` | 出现错误，查看 `last_error` |

### 10.2 HTTP 兼容接口：开始建图

| 项目 | 值 |
| --- | --- |
| HTTP | `POST /mapping/start` |
| 用途 | 清空旧累积数据，设置当前地图名，并开始累积新地图 |

请求示例：

```bash
curl -X POST http://<机器人IP>:8088/mapping/start \
  -H "Content-Type: application/json" \
  -d '{"map_name":"floor_1"}'
```

返回示例：

```json
{
  "success": true,
  "state": "mapping",
  "current_map_name": "floor_1",
  "map_pgm_path": "src/bxi_nav/maps/floor_1.pgm",
  "map_yaml_path": "src/bxi_nav/maps/floor_1.yaml"
}
```

### 10.3 HTTP 兼容接口：暂停或继续建图

| 项目 | 值 |
| --- | --- |
| HTTP | `POST /mapping/pause` |
| 用途 | 暂停或继续地图累积；不停止 Point-LIO 定位 |

暂停：

```bash
curl -X POST http://<机器人IP>:8088/mapping/pause \
  -H "Content-Type: application/json" \
  -d '{"paused":true}'
```

继续：

```bash
curl -X POST http://<机器人IP>:8088/mapping/pause \
  -H "Content-Type: application/json" \
  -d '{"paused":false}'
```

### 10.4 HTTP 兼容接口：取消建图

| 项目 | 值 |
| --- | --- |
| HTTP | `POST /mapping/cancel` |
| 用途 | 清空当前未保存地图数据，不生成地图文件 |

命令示例：

```bash
curl -X POST http://<机器人IP>:8088/mapping/cancel
```

### 10.5 HTTP 兼容接口：保存地图

| 项目 | 值 |
| --- | --- |
| HTTP | `POST /mapping/save` |
| 用途 | 将当前累积地图保存为 Nav2 `.pgm` 和 `.yaml` |

命令示例：

```bash
curl -X POST http://<机器人IP>:8088/mapping/save \
  -H "Content-Type: application/json" \
  -d '{"map_name":"floor_1"}'
```

输出文件：

```text
src/bxi_nav/maps/floor_1.pgm
src/bxi_nav/maps/floor_1.yaml
```

注意：这个接口保存的是 App/导航使用的 **2D 栅格地图**，不是 Point-LIO 原始 PCD 点云地图。Point-LIO 自带的 PCD 保存由 `pcd_save.pcd_save_en` 控制，通常在 Point-LIO 正常退出时写入：

```text
src/Point-LIO/PCD/scans.pcd
```

如果需要保留 PCD，请结束建图时让 Point-LIO 正常退出，例如在 Point-LIO 终端按 `Ctrl+C`，等待保存完成后再关闭终端。

保存后可用该地图启动导航：

```bash
source install/setup.bash
ros2 launch nav indoor_navigation_launch.py map:=$(pwd)/src/bxi_nav/maps/floor_1.yaml
```

### 10.6 HTTP 兼容接口：查询建图状态

| 项目 | 值 |
| --- | --- |
| HTTP | `GET /mapping/status` |
| 用途 | 查询当前建图状态和当前地图名 |

命令示例：

```bash
curl http://<机器人IP>:8088/mapping/status
```

返回示例：

```json
{
  "state": "mapping",
  "current_map_name": "floor_1",
  "map_pgm_path": "src/bxi_nav/maps/floor_1.pgm",
  "map_yaml_path": "src/bxi_nav/maps/floor_1.yaml",
  "resolution": 0.1,
  "width": 600,
  "height": 600,
  "last_error": ""
}
```

状态含义：

| 状态 | 含义 |
| --- | --- |
| `idle` | 控制节点已启动，但未开始建图 |
| `mapping` | 正在累积地图 |
| `paused` | 已暂停地图累积 |
| `saving` | 正在保存地图 |
| `saved` | 地图已保存 |
| `cancelled` | 当前建图已取消，未保存数据已清空 |
| `error` | 出现错误，查看 `last_error` |

### 10.7 建图保存完整流程

1. 启动集成建图程序：

```bash
source install/setup.bash
ros2 launch point_lio point_lio_with_mapping_control.launch.py
```

也可以使用一键启动：

```bash
./start.sh
```

2. 确认建图控制话题在线：

```bash
ros2 topic list | grep /mapping
ros2 topic echo /mapping/status
```

3. 开始记录新地图：

```bash
ros2 topic pub --once /mapping/start std_msgs/msg/String "{data: 'floor_1'}"
```

4. 推动机器人完成建图路线。期间可以暂停或继续地图累积：

```bash
ros2 topic pub --once /mapping/pause std_msgs/msg/Bool "{data: true}"

ros2 topic pub --once /mapping/pause std_msgs/msg/Bool "{data: false}"
```

5. 保存 2D 导航地图：

```bash
ros2 topic pub --once /mapping/save std_msgs/msg/String "{data: 'floor_1'}"
```

6. 检查输出文件：

```bash
ls src/bxi_nav/maps/floor_1.*
```

期望看到：

```text
src/bxi_nav/maps/floor_1.pgm
src/bxi_nav/maps/floor_1.yaml
```

7. 使用新地图启动导航：

```bash
source install/setup.bash
ros2 launch nav indoor_navigation_launch.py map:=$(pwd)/src/bxi_nav/maps/floor_1.yaml
```

8. 如果还需要保存 Point-LIO PCD 点云地图，正常退出 Point-LIO，等待生成：

```text
src/Point-LIO/PCD/scans.pcd
```

当前版本中，`/mapping/save` 生成的 `floor_1.pgm/floor_1.yaml` 和 Point-LIO 的 `scans.pcd` 是两套保存逻辑，文件名不会自动同步。

## 11. 对接检查

联调前检查：

```bash
source install/setup.bash
ros2 node list
ros2 topic list
ros2 action list
ros2 topic echo --once /map --field info
ros2 topic echo --once /aft_mapped_to_init
ros2 action info /navigate_to_pose
ros2 topic list | grep /mapping
ros2 topic echo /mapping/status
```

关键条件：

| 检查项 | 期望 |
| --- | --- |
| `/cloud_registered` | 持续发布 |
| `/aft_mapped_to_init` | 持续发布 |
| `/map` | 可读取地图信息 |
| `/navigate_to_pose` | action 存在 |
| `/mapping/start` | topic 存在，类型为 `std_msgs/msg/String` |
| `/mapping/pause` | topic 存在，类型为 `std_msgs/msg/Bool` |
| `/mapping/cancel` | topic 存在，类型为 `std_msgs/msg/Empty` |
| `/mapping/save` | topic 存在，类型为 `std_msgs/msg/String` |
| `/mapping/status` | 持续发布 JSON 状态 |
| TF | `map -> odom -> base_link` 连通 |

## 11. 当前说明

1. 本文档只列当前 ROS 2 接口。
2. `src/bxi_nav/src/main.cpp` 当前只是占位订阅节点，不是 App 对接服务。
3. 当前导航目标发送依赖 Nav2 标准 action `/navigate_to_pose`。
