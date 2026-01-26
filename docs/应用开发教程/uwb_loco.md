# 使用UWB模块搭建室内定位系统

注意：本教程仅适用于XDKJY UWB 套件

## 1. 简介

超宽带（UWB, Ultra-Wideband）定位是一种基于无线电的定位方法，通过测量锚点（固定信标）与标签（安装在无人机上的设备）之间的飞行时间（ToF, Time-of-Flight）来确定距离。通过精确测量来自多个锚点的距离，系统可通过三边测量法实时计算无人机的三维位置。我们为用户提供 XDKJY UAV UWB 定位套件，作为基于视觉的方法或动作捕捉系统的替代方案。

## 2. 硬件搭建

该定位系统的搭建非常简单。对于普通用户，推荐使用最小 5m×5m 的测试场地。需要避免电磁干扰，因为 UWB 信号较易受到干扰，因此在布置场地时应尽量远离潜在的干扰源。

详细的搭建步骤请参考用户手册。简而言之，用户只需布置 UWB 锚点并进行一次在线标定即可。下图展示了场景布置示意。

## 3. 从官方镜像安装

UWB 定位相关软件包托管于我们的官方镜像站点：https://mirror.cloudkernel.cn
目前软件支持 amd64 与 arm64 架构。建议用户按照以下步骤进行安装与更新。

下载 XDKJY UAV UWB 套件的发布文件：

```bash
cd ~ && git clone https://gitee.com/cloudkernel-tech/dasa_release_files
```

下载 deb 文件并安装：

```bash
cd ~/dasa_release_files/install
bash download_deb_from_host.sh
bash install_all_deb.sh
```

安装脚本会自动将软件包部署到机载计算机中。

锁定已安装的软件包以避免后续 apt 更新造成冲突（推荐）：

```bash
sudo apt-mark hold ros-melodic-mavros ros-melodic-mavlink ros-melodic-mavros-extras \
&& ros-melodic-libmavconn ros-melodic-dasa-visualization \
&& ros-melodic-mavros-msgs ros-melodic-nlink-parser ros-melodic-uwb-localization ros-melodic-loco-driver
```

## 4. 使用说明

测量 ENU 坐标系下 x 轴的初始偏航角。用户可手持无人机，将机头旋转至 x+ 方向，并记录 QGroundControl 地面站中显示的偏航角。记该偏航角为 yaw_ned。从 NAssistant 软件中获取 Nooploop UWB 模块的锚点坐标。根据需要修改 dasa_release_files/scripts/single_case 目录下的 dasa_config.yaml 配置文件。
请注意：变量 yaw_dasa_x_axis_enu 需根据步骤 (1) 中的 yaw_ned 计算得到。

```
# param: value

# DASA x 轴在 ENU 坐标系下的初始偏航角，需通过 DASA agent 测量，单位：rad，范围 [-Pi, Pi]
# 计算公式: yaw_dasa_x_axis_enu = wrap2Pi(pi/2 - yaw_ned)
yaw_dasa_x_axis_enu: 2.2

anchor_position:
# 坐标值必须采用 x.x 格式（如 5.0），按顺序编号 anchor0–3
# 仅在 nl 模式下加载，默认使用 RFU 坐标系
- [ 0.0,       0.0,        0.0]
- [-0.13,       6.307,        0.0]
- [ 2.857,       6.391,        0.0]
- [ 2.981,       0.0,        0.0]
```

将配置文件复制到系统级安装的 uwb_localization 软件包目录。

```bash
cd ~/dasa_release_files/scripts/single_case
sudo cp dasa_config.yaml /opt/ros/melodic/share/uwb_localization/config/
```

最后，用户可以启动无人机定位所需的所有节点，包括 mavros、UWB 驱动与 uwb_localization。

```bash
cd ~/dasa_release_files/scripts/single_case
bash run_uav_default.sh
```

若一切运行正常，可在 ROS 主题 /dasa/local_position/pose 中查看定位数据，其坐标系由 UWB 锚点定义的全局框架决定。
