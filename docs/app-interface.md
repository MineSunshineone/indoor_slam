# Indoor SLAM App 对接接口文档

本文档列出 App 侧对接所需的全部 ROS 2 接口。导航和建图分别收敛在 `/nav` 和 `/mapping` 两个命名空间下，不再使用 HTTP。

## 1. 基础信息

| 项目 | 值 |
| --- | --- |
| ROS 版本 | ROS 2 Humble |
| 默认 ROS_DOMAIN_ID | `37` |
| 主坐标系 | `map` |
| 里程计坐标系 | `odom` |
| 机器人坐标系 | `base_link` |
| 默认地图 | `src/bxi_nav/maps/maps.yaml` |
| 自定义接口包 | `bxi_nav_interfaces` |

启动完整系统：

```bash
./start.sh
```

手动启动导航：

```bash
source install/setup.bash
ros2 launch nav indoor_navigation_launch.py
```

## 2. App 接入方式

App 不需要安装 ROS 或 DDS。系统在机器人上运行 `rosbridge_server`，App 通过 websocket + JSON 访问所有接口。

| 项目 | 值 |
| --- | --- |
| 地址 | `ws://<机器人IP>:9090` |
| 协议 | rosbridge v2（JSON） |
| 可访问 | topic / service / action |

JSON 操作对应关系：

| 接口种类 | 调用方式 | rosbridge op |
| --- | --- | --- |
| service | 调用并等回执 | `call_service` |
| topic | 订阅推送 | `subscribe` |
| action | 发目标、收反馈 | `send_action_goal` |
| action | 取消目标 | `cancel_action_goal` |

## 3. 命名空间约定

| 命名空间 | 用途 |
| --- | --- |
| `/nav` | 导航相关 |
| `/mapping` | 建图相关 |
| 顶层 | 地图数据和标准 topic（`/map`、`/scan` 等） |

Nav2 内部仍使用标准接口名（`/navigate_to_pose`、`/initialpose` 等）。`/nav` 下的接口由网关节点转译，App 只对接 `/nav` 即可，不直接接触 Nav2 原始接口。

## 4. 导航接口

### 4.1 接口列表

| 操作 | 名称 | 类型 | 种类 |
| --- | --- | --- | --- |
| 设初始位姿 | `/nav/init` | `bxi_nav_interfaces/srv/SetInitialPose` | service |
| 发导航目标 | `/nav/goto` | `bxi_nav_interfaces/action/NavGoto` | action |
| 暂停 / 继续 | `/nav/pause` | `std_srvs/srv/SetBool` | service |
| 取消 | 取消 `/nav/goto` 目标 | — | action cancel |
| 多点巡航 | `/nav/follow_waypoints` | `bxi_nav_interfaces/action/FollowWaypoints` | action |
| 当前位姿 | `/nav/pose` | `bxi_nav_interfaces/msg/NavPose` | topic |
| 导航状态 | `/nav/status` | `bxi_nav_interfaces/msg/NavStatus` | topic |

### 4.2 设初始位姿

App 给 x/y/yaw，网关内部转四元数后发布到 `/initialpose`。

```bash
ros2 service call /nav/init bxi_nav_interfaces/srv/SetInitialPose \
  "{x: 0.0, y: 0.0, yaw: 0.0}"
```

### 4.3 发导航目标

目标方向用 yaw（弧度），网关内部转四元数。发出后通过 action feedback 持续收到剩余距离、剩余时间。

```bash
ros2 action send_goal /nav/goto bxi_nav_interfaces/action/NavGoto \
  "{x: 1.2, y: 0.5, yaw: 1.57}" --feedback
```

NavGoto 状态流转：

```
send goal -> navigating -> succeeded / canceled / failed
                 |
            pause(true) -> paused -> pause(false) -> navigating
```

### 4.4 暂停与继续

```bash
ros2 service call /nav/pause std_srvs/srv/SetBool "{data: true}"   # 暂停
ros2 service call /nav/pause std_srvs/srv/SetBool "{data: false}"  # 继续
```

### 4.5 取消导航

取消当前 `/nav/goto` 目标。rosbridge 用 `cancel_action_goal`，命令行用：

```bash
ros2 action send_goal ... 后 Ctrl+C，或在客户端发 cancel
```

### 4.6 当前位姿

| 字段 | 说明 |
| --- | --- |
| `x` / `y` | 平面坐标，单位 m |
| `yaw` | 朝向，单位 rad |
| `twist` | 速度信息 |

```bash
ros2 topic echo --once /nav/pose
```

### 4.7 导航状态

| 字段 | 说明 |
| --- | --- |
| `state` | `idle` / `navigating` / `paused` / `succeeded` / `canceled` / `failed` |
| `distance_remaining` | 剩余距离 |
| `estimated_time_remaining` | 预计剩余时间 |
| `navigation_time` | 已导航时间 |
| `number_of_recoveries` | 恢复行为次数 |

`/nav/status` 为 latched topic，订阅后立即收到最新状态，之后变化时推送。

## 5. 建图接口

### 5.1 接口列表

| 操作 | 名称 | 类型 | 种类 |
| --- | --- | --- | --- |
| 开始建图 | `/mapping/build` | `bxi_nav_interfaces/action/BuildMap` | action |
| 暂停 / 继续累积 | `/mapping/pause` | `std_srvs/srv/SetBool` | service |
| 保存并完成 | `/mapping/save` | `bxi_nav_interfaces/srv/SaveMap` | service |
| 取消 | 取消 `/mapping/build` 目标 | — | action cancel |
| 清除地形点云 | `/mapping/clear_terrain` | `bxi_nav_interfaces/srv/ClearTerrain` | service |
| 建图状态 | `/mapping/status` | `bxi_nav_interfaces/msg/MappingStatus` | topic |

地图名限制：只能包含英文字母、数字、下划线和短横线，长度 1-64，例如 `floor_1`。

### 5.2 开始建图

发 `/mapping/build` 目标即开始累积，goal 持续活跃表示会话进行中。建图进度通过 action feedback 推送。

```bash
ros2 action send_goal /mapping/build bxi_nav_interfaces/action/BuildMap \
  "{map_name: 'floor_1'}" --feedback
```

feedback 字段：

| 字段 | 说明 |
| --- | --- |
| `state` | `mapping` / `paused` |
| `cells_known` | 已知栅格数 |
| `coverage_percent` | 覆盖率 |
| `elapsed_sec` | 已建图时长 |
| `resolution` / `width` / `height` | 当前地图参数 |

### 5.3 暂停与继续累积

暂停时 Point-LIO 继续运行，仅停止地图累积。不结束会话。

```bash
ros2 service call /mapping/pause std_srvs/srv/SetBool "{data: true}"   # 暂停
ros2 service call /mapping/pause std_srvs/srv/SetBool "{data: false}"  # 继续
```

### 5.4 保存并完成

写出 Nav2 可加载的 `.pgm` 和 `.yaml`，同时让 `/mapping/build` 成功结束，result 返回文件路径。

```bash
ros2 service call /mapping/save bxi_nav_interfaces/srv/SaveMap \
  "{map_name: 'floor_1'}"
```

输出文件：

```text
src/bxi_nav/maps/floor_1.pgm
src/bxi_nav/maps/floor_1.yaml
```

### 5.5 取消建图

取消 `/mapping/build` 目标。清空未保存数据，不生成文件。已通过 save 写出的文件不受影响。

### 5.6 清除地形点云

```bash
ros2 service call /mapping/clear_terrain bxi_nav_interfaces/srv/ClearTerrain \
  "{radius: 8.0}"
```

### 5.7 建图状态

供未持有 build 目标的客户端查看。

| 字段 | 说明 |
| --- | --- |
| `state` | `idle` / `mapping` / `paused` / `saving` / `saved` / `cancelled` / `error` |
| `current_map_name` | 当前地图名 |
| `coverage_percent` | 覆盖率 |
| `resolution` / `width` / `height` | 地图参数 |
| `last_error` | 错误信息 |

### 5.8 会话生命周期

```
send goal(/mapping/build, map_name)
        |
        v
     mapping  <----- pause(false) -----+
        |                              |
   pause(true)                         |
        v                              |
     paused  ------- pause(false) -----+

  /mapping/save  -> 写文件，build 成功结束（带路径）
  cancel goal    -> 丢弃未保存数据，build 结束，不出文件
```

### 5.9 PCD 点云地图

`/mapping/save` 保存的是 2D 栅格地图，不是 Point-LIO 原始 PCD。PCD 由 Point-LIO 的 `pcd_save.pcd_save_en` 控制，通常在 Point-LIO 正常退出时写入：

```text
src/Point-LIO/PCD/scans.pcd
```

如需保留 PCD，结束建图时让 Point-LIO 正常退出（终端按 `Ctrl+C`），等待保存完成再关闭。两套文件名不会自动同步。

保存后用新地图启动导航：

```bash
source install/setup.bash
ros2 launch nav indoor_navigation_launch.py map:=$(pwd)/src/bxi_nav/maps/floor_1.yaml
```

## 6. 地图与数据 topic

### 6.1 地图数据

| 项目 | 值 |
| --- | --- |
| Topic | `/map` |
| 类型 | `nav_msgs/msg/OccupancyGrid` |
| 发布源 | `nav2_map_server` |
| 坐标系 | `map` |
| QoS | reliable + transient_local |

`OccupancyGrid` 关键字段：

| 字段 | 说明 |
| --- | --- |
| `header.frame_id` | 地图坐标系，通常为 `map` |
| `info.resolution` | 分辨率，单位 m/cell |
| `info.width` / `info.height` | 地图宽 / 高，单位 cell |
| `info.origin` | 地图左下角在 `map` 下的位姿 |
| `data` | 一维栅格数组，长度 `width * height` |

栅格值：`-1` 未知，`0` 空闲，`100` 占用 / 障碍。

坐标换算：

```text
index = y_cell * width + x_cell
map_x = origin.position.x + x_cell * resolution
map_y = origin.position.y + y_cell * resolution
```

### 6.2 调试数据

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Point-LIO 配准点云 |
| `/terrain_map` | `sensor_msgs/msg/PointCloud2` | 地形 / 障碍点云 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 点云转 2D 激光，供 Nav2 局部代价地图使用 |
| `/debug/path` | `nav_msgs/msg/Path` | Point-LIO 轨迹 |
| `/debug/plan` | `nav_msgs/msg/Path` | Nav2 全局规划路径 |
| `/debug/local_plan` | `nav_msgs/msg/Path` | 局部 A* 路径调试输出 |
| `/debug/free_paths` | `sensor_msgs/msg/PointCloud2` | 可通行候选路径点云 |

## 7. 自定义接口定义

包名 `bxi_nav_interfaces`。

`srv/SetInitialPose.srv`

```
float64   x
float64   y
float64   yaw
float64[] covariance     # 可空，默认 0.25 / 0.0685
---
bool      success
string    message
```

`action/NavGoto.action`

```
# goal
float64 x
float64 y
float64 yaw
---
# result
bool    success
string  message
uint16  number_of_recoveries
float64 total_time
---
# feedback
float64 distance_remaining
float64 estimated_time_remaining
float64 navigation_time
uint16  number_of_recoveries
```

`msg/NavPose.msg`

```
std_msgs/Header     header
float64             x
float64             y
float64             yaw
geometry_msgs/Twist twist
```

`msg/NavStatus.msg`

```
std_msgs/Header header
string  state
float64 distance_remaining
float64 estimated_time_remaining
float64 navigation_time
uint16  number_of_recoveries
```

`action/BuildMap.action`

```
# goal
string map_name
---
# result
bool    success
string  message
string  map_pgm_path
string  map_yaml_path
---
# feedback
string  state
uint32  cells_known
float32 coverage_percent
float32 elapsed_sec
float32 resolution
uint32  width
uint32  height
```

`srv/SaveMap.srv`

```
string map_name          # 可空，默认用 build 的 map_name
---
bool   success
string message
string map_pgm_path
string map_yaml_path
```

`srv/ClearTerrain.srv`

```
float32 radius
---
bool    success
string  message
```

`msg/MappingStatus.msg`

```
std_msgs/Header header
string  state
string  current_map_name
float32 coverage_percent
float32 resolution
uint32  width
uint32  height
string  last_error
```

## 8. QoS 说明

ROS 2 中 QoS 不匹配会导致收不到数据且无报错，App 侧需按下表匹配。

| Topic | QoS |
| --- | --- |
| `/map` | reliable + transient_local |
| `/nav/pose` | reliable + transient_local |
| `/nav/status` | reliable + transient_local |
| `/mapping/status` | reliable + transient_local |
| `/scan` | best_effort |
| `/cloud_registered`、`/terrain_map`、`/debug/*` | best_effort |

rosbridge 订阅 latched topic 后会立即收到最近一帧，无需轮询。

## 9. 对接检查

```bash
source install/setup.bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 topic echo --once /map --field info
ros2 topic echo --once /nav/pose
ros2 action info /nav/goto
ros2 action info /mapping/build
```

关键条件：

| 检查项 | 期望 |
| --- | --- |
| `/cloud_registered` | 持续发布 |
| `/nav/pose` | 持续发布 |
| `/map` | 可读取地图信息 |
| `/nav/goto` | action 存在 |
| `/mapping/build` | action 存在 |
| rosbridge | `ws://<机器人IP>:9090` 可连接 |
| TF | `map -> odom -> base_link` 连通 |

## 10. 说明

1. 本文档只列当前 ROS 2 接口，HTTP 接口已废弃。
2. `/nav` 和 `/mapping` 下的接口由网关节点提供，对外屏蔽 Nav2 原始接口。
3. 导航底层仍为 Nav2 标准 action `/navigate_to_pose`。
