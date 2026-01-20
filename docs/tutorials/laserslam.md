# 使用激光SLAM实现室内定位

注意：本教程适用于配备激光扫描仪的Kerloud无人机
本教程介绍了我们如何实现室内定位的激光SLAM方法。

教程代码仅供研究项目使用，不能直接部署在产品上。

## 1. 硬件设置和环境要求

本教程所需的硬件设置是：

- Kerloud 无人机，需配置Nvidia Jetson Nano等机载电脑
- 室内激光雷达套件，包括 LD19 激光雷达 和 北醒 TF-Luna 距离传感器

经过充分测试的环境是带有 Ubuntu 18.04 的 ROS Melodic，所有必要的组件都在工厂中正确设置。

## 2. 工作原理

室内SLAM过程是通过将2D激光SLAM算法与自动驾驶仪中的EKF状态估计模块集成来实现的。
采用的2D Laser slam算法是 Google Cartographer ，它是机器人界的基准算法。Cartographer 软件包可以输出车辆在水平 2D 平面中的精确实时位置，而 TF-Luna 距离传感器则提供高度测量。

## 3. 如何运行

### 3.1 远程可视化的网络设置

远程可视化提供了从远程 PC 访问机载计算机的用户友好体验。我们首先必须将机载计算机连接到本地 wifi 网络一次，以便之后自动设置 wifi 连接。
登录路由器管理页面即可找到无人机的IP地址。以TP-Link路由器为例，用户可以访问192.168.0.1或http://tplogin.cn/查看本地网络中连接的所有计算机。机载电脑的计算机名称和密码默认均为ubuntu 。然后显示 IP 地址。

获取 IP 地址后，我们可以使用以下命令在远程 PC 和无人机机载电脑中设置主机 IP：

```bash
sudo vim /etc/hosts

# set a host name for kerloud uav
# e.g. 192.168.0.104 master_ip
<IP address> master_ip
```

然后，我们在远程 PC 中设置 ROS 环境变量，而机载电脑的环境变量则在生产中设置：

```bash
vim ~/.bashrc

export ROS_IP=`hostname -I | awk '{print $1}'`
export ROS_HOSTNAME=`hostname -I | awk '{print $1}'`
export ROS_MASTER_URI=http://master_ip:11311
```

为了验证网络连接，我们可以在远程PC上执行以下命令：

```bash
# PC side Terminal 1: setup ssh connection with the onboard computer
ssh ubuntu@master_ip
roscore

# PC side Terminal 2:
rostopic list
```

如果网络设置正确，那么 /rosout 和 /rosout_agg 主题可以在上面的第二个终端中查看，否则会出现如下错误消息：
ERROR: Unable to communicate with master!

### 3.2 构建工作空间

室内激光slam的ROS工作空间位于~/src/uav_space/catkinws_laserslam，它包含几个包：

- ldlidar_stl_ros：LD19激光扫描仪的ros驱动程序
- robots_laserslam：激光 slam 包，包含制图师的配置和启动文件
- pose_converter：将 slam 输出与距离传感器测量融合的包，并将 3D 位置提供给 mavros

要编译工作区，只需运行：

```bash
cd ~/src/uav_space/catkinws_laserslam
catkin build -j 3
```

### 3.3 用数据集进行SLAM仿真

为了方便室内 slam 包的部署，我们在 ~/src/uav_space/catkinws_laserslam/dataset/2D-laser-datasets 目录下提供了几个激光扫描数据集。用户可以为这些数据集启动激光撞击节点来熟悉这些软件工具。
执行模拟的命令如下所示：

```bash
# PC side terminal 1: launch roscore after ssh connection
ssh ubuntu@master_ip
roscore

# PC side terminal 2: launch rosbag to play a dataset
ssh ubuntu@master_ip

cd ~/src/uav_space/catkinws_laserslam \
&& cd dataset/2D-laser-datasets \
&& rosbag play floor1.bag --clock

# PC side terminal 3: launch rosbag to play a dataset
ssh ubuntu@master_ip

cd ~/src/uav_space/catkinws_laserslam \
&& source devel/setup.bash \
&& roslaunch robot_laserslam databag_sim.launch

# PC side terminal 4: remote visualization
# users have to copy the workspace from the onboard computer to the PC and build it as well
cd ~/src/uav_space/catkinws_laserslam \
source devel/setup.bash \
&& roslaunch robot_laserslam visualization.launch
```

### 3.4 室内实验

请确保 Kerloud 自动驾驶仪已正确配置为基于 SLAM 的定位，详细信息请参阅用户手册。

要启动激光 SLAM 流程，建议严格遵循以下步骤：

- 将无人机放置在四周有墙壁、光线条件良好的室内环境中
- 确保电池电压至少为 16.0V 并给机器上电
- 使用路由器建立本地wifi网络，并确保机载电脑可以自动连接到该网络
- 将无人机与QGround控制站连接
