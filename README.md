# L1 Navigation 运行说明

本仓库是 ROS 2 Humble 工作区。下面以 MID360 为例；换用其他雷达时，要同时选择对应的 Livox 驱动 launch 和 FAST-LIO 配置。

## 下载仓库

~~~bash
git clone https://github.com/VLAIRobotics/l1_navigation.git
cd l1_navigation
~~~

完整安装、设备接入、建图与导航步骤见[使用手册（详细操作指南）](docs/使用手册/使用手册.md)。

## 编译与环境

内存有限的开发板不要一次编译整个工作区或启用多个并行 worker，以免耗尽内存。首次使用按目标包逐步构建，命令和选择方式见[手册中的编译说明](docs/使用手册/环境准备与终端操作.md#build-memory)。

每个新终端都先加载环境：

```bash
cd l1_navigation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 无图导航

启动 MID360、FAST-LIO、Patchwork++、FastDEM 和 Nav2（Theta* 全局规划 + TEB 局部控制）：

```bash
./tool/start_nav2.sh config_file:=mid360.yaml sensor_height:=0.25 visualize:=true enable_chassis:=false
```

这条链使用 `odom` 下的滚动代价地图，不加载静态地图。当前 launch 默认会启动底盘驱动，所以只查看数据时要明确传 `enable_chassis:=false`。无显示器时将 `visualize:=true` 改成 false。只有在 CAN 接口、速度标定和急停均已确认后，才设置 `enable_chassis:=true`。

按 `Ctrl-C` 停止。

## 建图并保存 PCD

下面启动 MID360、FAST-LIO、动态障碍过滤器和 RViz。过滤器默认开启，输入为 FAST-LIO 的完整配准点云；清理后的静态 PCD 路径由 `map_file` 指定。

启动建图并明确指定两份输出路径：

```bash
./tool/start_fastlio_mapping.sh \
  config_file:=mid360.yaml \
  map_save_file:=PCD/raw_map.pcd \
  map_file:=PCD/clean_map.pcd \
  rviz:=true
```

完成采集后、退出建图前，在另一个已加载 ROS 环境的终端按需保存：

```bash
# FAST-LIO 累计的完整配准点云；路径取自启动参数 map_save_file
ros2 service call /map_save std_srvs/srv/Trigger "{}"

# 动态过滤后确认的静态地图；路径取自启动参数 map_file
ros2 service call /static_map_save std_srvs/srv/Trigger "{}"
```

服务响应中的 `success: true` 和 `message` 会确认保存结果与文件路径。过滤地图需要累计足够的稳定观测；默认动态体素连续约 10 帧未被命中后清除，历史静态体素至少观测 1 秒且机器人移动 0.5 米后归档。保存前先走完需要建图的区域，再检查过滤结果。

| 文件 | 内容 | 保存入口 |
| --- | --- | --- |
| `PCD/raw_map.pcd` | FAST-LIO 累计的完整配准点云，未经过动态障碍过滤 | `/map_save`；路径由启动参数 `map_save_file:=...` 指定 |
| `PCD/clean_map.pcd` | 时序体素确认的静态地图 | `/static_map_save`；路径由 `map_file:=...` 指定 |

## PCD 地图导航

使用清理后的 PCD 启动地图转换、MID360 定位和地图导航：

```bash
./tool/start_map_navigation.sh \
  pcd_file:=PCD/clean_map.pcd \
  config_file:=mid360.yaml \
  visualize:=true \
  enable_chassis:=false
```

脚本会在 PCD 同目录创建 `<文件名>_nav/` 缓存，生成二维占据图、2.5D 地形图和定位用 PCD。它先运行 3D-BBS/GICP 定位；定位交接成功后再启动 map server、Nav2 和 RViz。需要启用真实底盘时，确认 CAN 已配置后在命令末尾加 `enable_chassis:=true`。

## rosbag 录制与回放

建图时录制雷达原始消息、FAST-LIO 配准点云、动态过滤输出、里程计和 TF：

```bash
mkdir -p PCD/bags
ros2 bag record -o "PCD/bags/mapping_$(date +%Y%m%d_%H%M%S)" \
  /livox/lidar \
  /livox/imu \
  /Odometry \
  /cloud_registered \
  /dynamic_filter/static_cloud \
  /dynamic_filter/rejected_cloud \
  /tf \
  /tf_static
```

按 `Ctrl-C` 结束录制。`/livox/lidar` 和 `/livox/imu` 是雷达驱动原始消息；`/cloud_registered` 是 FAST-LIO 配准点云；`/dynamic_filter/rejected_cloud` 是当前帧未纳入静态地图的点；`/dynamic_filter/static_cloud` 是过滤器逐帧输出的静态点。rosbag 保存话题数据，不会自动生成 PCD；要导出 PCD，请在实时建图期间调用上面的保存服务。

查看或回放：

```bash
ros2 bag info PCD/bags/你的bag目录
ros2 bag play PCD/bags/你的bag目录
```

导航运行过程如需录制更完整的状态、代价地图、感知与日志，可用仓库脚本；`--raw` 会附加 Livox 原始话题：

```bash
./tool/record_nav2_stability.sh "PCD/nav2_$(date +%Y%m%d_%H%M%S)" --raw
```
