# navigation
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
| L1（两轮） | `./tool/start_nav2.sh ` | `./tool/start_map_navigation.sh ` |
| K1（全向） | `./tool/start_nav2.sh chassis_model:=omni ` | `./tool/start_map_navigation.sh chassis_model:=omni ` |
