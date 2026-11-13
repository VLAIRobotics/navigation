# navigation

本仓库是 ROS 2 Humble 工作区，提供 FAST-LIO2、地面处理、高程地图和 Nav2 导航流程。

## 现有问题

更新日期：2026-11-13。依据当日启动日志、离线快照和当前源码整理；“已确认”表示已有日志或代码证据，不表示已完成修复或实车验收。

| 问题 | 状态与影响 | 后续处理 |
| --- | --- | --- |
| 全向底盘驱动无法启动 | **已确认。** 17:04:49 启动轮次使用 `chassis_model:=omni`；驱动因横移 CAN 字段、方向和比例尚未验证而退出，无法发送运动指令。导航仍继续运行，界面有路线也无法驱动车体。 | 核实实物型号及 CAN 协议，完成对应驱动支持；补充底盘启动失败和退出时的导航联动。[驱动入口](src/bringup/src/chassis_driver.cpp) |
| 当前足迹被判为碰撞，TEB 无法起步 | **已确认阻断，障碍来源待核实。** 最新日志出现 `pose_index=0`、`master_cost=254`，致命代价来自 `temporal_obstacle_layer`；随后无进展超时并重试。尚未确认是真实障碍，还是地面、车体回波等误判。 | 核对命中单元的点云来源、地面分割、TF 时序和障碍确认/清除过程；验证可通行起点与真实障碍两种情况。[局部代价地图配置](src/bringup/config/nav2_map_navigation.yaml) |
| 两轮模式下运动指令频繁被安全门拦停 | **已确认。** 较早的两轮启动轮次出现足迹碰到占用/未知区域、里程计速度超限；运动安全门额外使用 0.15 m 裕量并阻挡未知格。配置速度上限为 0.5 m/s、1.8 rad/s，人工移动时也可能触发。 | 记录首个命中单元及指令/实测速度，核对规划器与底盘的安全范围；区分实际风险与误判。[门控配置](src/bringup/launch/map_navigation.launch.py) |
| 全向模式全局膨胀半径与足迹不匹配 | **已确认。** 日志报告全局膨胀半径 0.25 m，小于全向足迹计算出的内切半径 0.285 m。 | 按实际底盘几何统一足迹、padding 与全局/局部膨胀参数，验证碰撞检查和代价分布。[导航配置](src/bringup/config/nav2_map_navigation.yaml) |
| 启动阶段 IMU 积压、断档及位姿保持 | **已确认启动期发生。** 最新轮次记录 IMU 间隔 0.106 s、累计缓冲丢弃 400 条；FAST-LIO 拒绝扫描并保持最后可信位姿。现有证据不能认定全程持续断流。 | 核对雷达/IMU 时间戳、启动积压和回调处理时序，验证恢复后输入持续新鲜。[FAST-LIO 输入处理](src/FAST_LIO_ROS2/src/laserMapping.cpp) |
| 移动时定位明显跳动或漂移 | **现象待进一步定因。** RViz 默认显示 1 Hz 的原始定位箭头及未经验证的候选箭头；正式导航 TF 另有平滑。最新轮次已接受跟踪修正最大 4.7 cm，4 份快照中正式 TF 与最后可信 GICP 的位置差约 0.5～3.2 cm；这些数值不是真值误差，也不足以解释大幅漂移。 | 对照正式 `map→base_link` TF、FAST-LIO 里程计和 GICP 校正，记录运动及回到原点的误差，再决定匹配或时序修正；原始/候选箭头作为调试显示。[RViz 配置](src/bringup/rviz/map_navigation.rviz) · [TF 平滑](src/bringup/src/map_odom_handoff_node.cpp) |
| 导航控制循环频繁超时 | **已确认。** 最新轮次出现 130 条控制周期超时告警，计算耗时记录达到约 331 ms，超过 10 Hz 的 100 ms 预算，影响指令更新和平顺性。尚不能直接认定它导致定位漂移。 | 剖析 TEB 优化、障碍处理和回调负载，验证控制周期、指令新鲜度及停车保护。[TEB 配置](src/bringup/config/nav2_map_navigation.yaml) |
| FAST-LIO 在退出清理时段错误 | **已确认出现，根因待核实。** 较早轮次收到 SIGINT 后发生 publisher 清理错误及退出码 `-11`；它发生在导航启动失败后的关闭阶段。 | 核对信号处理、执行器停止和资源销毁顺序；单独验证正常退出及异常启动后的清理。[退出处理](src/FAST_LIO_ROS2/src/laserMapping.cpp) |

已处理与加载说明：Humble 参数数组改写导致的 `list indices must be integers or slices, not str` 已修复并通过离线参数展开回归，实车导航仍需验证。约 2018 万点地图的首次转换曾耗时约 52 秒；缓存齐全且地图、转换器和参数不变时会复用，已有轮次约 3.5 秒完成地图缓存准备、约 5 秒完成地图选择。这不代表完整定位和导航已就绪。

## 获取代码

已有代码且仓库根目录包含 `src/`、`tool/` 和 `README.md` 时，不需要重复克隆。

尚未获取代码时，在目标设备终端运行：

```bash
cd /home/nvidia
git clone https://github.com/VLAIRobotics/navigation.git
cd /home/nvidia/navigation
```

本页提供[使用手册总入口](#user-manual-index)、首次使用流程和快速启动命令。

<a id="user-manual-index"></a>

## 使用手册总入口

### 按使用目的查阅

| 使用阶段 | 对应章节 |
| --- | --- |
| 准备电脑和设备：安装 Ubuntu/ROS 2、编译工作区、连接 MID360 并配置网口 | [环境准备与终端操作](docs/使用手册/环境准备与终端操作.md) |
| 检查传感器、建图并校准：确认雷达数据、保存 PCD、校准 IMU 安装角度 | [传感器建图与校准](docs/使用手册/传感器建图与校准.md) |
| 启动导航和操作 RViz：使用无图导航或已有 PCD 地图导航，查看地图图层并发送目标 | [Nav2 导航与 RViz 操作](docs/使用手册/Nav2导航与RViz操作.md) |
| 记录、排查和维护：录制数据、查看日志、处理常见问题、部署文件或查找脚本 | [数据录制与日志](docs/使用手册/数据录制与问题排查.md#recording) · [常见问题](docs/使用手册/常见问题检查与解决.md) · [系统结构与部署](docs/使用手册/系统结构与关键文件部署使用.md) · [脚本与维护工具](docs/使用手册/脚本与维护工具.md) |

### 第一次使用

| 顺序 | 操作 |
| --- | --- |
| 1. 准备 | 按[环境准备与终端操作](docs/使用手册/环境准备与终端操作.md)安装依赖并编译工作区，再按[MID360 网络配置](docs/使用手册/设备接入与启动配置.md#mid360-network)完成接线和网口设置。 |
| 2. 检查 | 按[传感器检查流程](docs/使用手册/传感器建图与校准.md#sensor-check)启动驱动，确认 `/livox/lidar`、`/livox/imu` 和 `/Odometry` 持续有数据。 |
| 3. 建图（按需） | 需要保存场地地图时，按[建图与保存地图](docs/使用手册/传感器建图与校准.md#mapping-save)生成 PCD。 |
| 4. 导航 | 选择[无图导航](docs/使用手册/Nav2导航与RViz操作.md#no-map-navigation)或[地图导航](docs/使用手册/Nav2导航与RViz操作.md#map-navigation)。首次只验证定位和规划时，使用 `enable_chassis:=false`。 |

电脑网卡地址和雷达 IP 请按现场确认；`192.168.1.5`（电脑）与 `192.168.1.194`（MID360）只是当前配置示例。

### 操作约定

| 项目 | 说明 |
| --- | --- |
| 仓库路径 | 示例路径为 `/home/nvidia/navigation`；仓库放在其他位置时，请替换命令中的路径。 |
| ROS 环境 | 启动脚本会加载环境；手动运行 `ros2` 命令前，每个新终端都要加载 `/opt/ros/humble` 和工作区 `install` 环境。 |
| 流程切换 | 同一时间只运行一条传感器、建图或导航主流程；切换前在启动终端按 `Ctrl+C` 并等待退出。 |
| 真实底盘 | 只有 CAN、急停、速度标定和测试区域都经硬件负责人确认后，才启用真实底盘。 |

## 低内存编译

内存有限的开发板构建：

```bash
cd /home/nvidia/navigation
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEEFLAGS="-j2" colcon build --executor sequential
source install/setup.bash
```

首次依赖安装、`colcon_defaults.yaml` 配置和构建问题处理见[环境准备与终端操作](docs/使用手册/环境准备与终端操作.md#build-memory)。

## L1 与 K1 启动命令

从仓库根目录运行。以下命令先关闭底盘，用于检查定位和规划；地图导航默认读取 `PCD/clean_map.pcd`。详细参数和真车操作见[Nav2 导航与 RViz 操作](docs/使用手册/Nav2导航与RViz操作.md)。

| 机器人 | 无图导航 | 已有 PCD 地图导航 |
| --- | --- | --- |
| L1（两轮） | `./tool/start_nav2.sh enable_chassis:=false` | `./tool/start_map_navigation.sh enable_chassis:=false` |
| K1（全向） | `./tool/start_nav2.sh chassis_model:=omni enable_chassis:=false` | `./tool/start_map_navigation.sh chassis_model:=omni enable_chassis:=false` |
