# 基于跟踪相机的室内自主飞行实现

注意：本文只适用于XDKJY Nano VIO Indoor无人机产品
室内定位作为无人机室内自主飞行的关键技术，实现方式有光流、激光和基于视觉的SLAM方法。
区别于市场已有产品，我们基于对视觉可提供丰富信息（不限于定位）的理解，专注研究视觉方法在室内场景的应用。这里我们将为高级用户提供XDKJY vision drone。

## 如何配置

### 固件设置

机载的XDKJY飞控单元在出厂阶段已正确配置。XDKJY Nano VIO indoor可执行室内、室外任务，
我们提供的EKF2_INDOOR_EN参数可便捷切换室内、室外场景：将EKF2_INDOOR_EN置1，无人机自动切换为室内模式。

### 网络设置

将XDKJY vision drone通过Wi-Fi连接到本地网络，可以轻松实现远程操作。基于ssh工具，用户可以通过相应的IP地址访问机载电脑。

```
ssh ubuntu@<drone IP address>
```

登录所连路由器的管理页面可以查阅无人机的IP地址。以TP-Link路由器为例，用户可以访问192.168.1.1或者http://tplogin.cn来检阅本地网络所连接的全部电脑。
XDKJY vision drone默认使用的计算机名为ubuntu，IP地址显示如下：

你也可以考虑使用手机热点功能来建立需要的局域网。

## 室内半自动飞行

我们建议所有用户第一步先完成室内半自动飞行，这样有助于验证板载软件的定位精度。XDKJY vision drone采用Intel Realsense追踪摄像头T265作为板载感知，
板载软件含realsense library、mavros和visual odometry 软件包，工作环境位于~/src/catkinws_realsense。

编译工作环境：

```bash
cd ~/src/catkinws_realsense
catkin build -j 3
```

在系统上电启动后等待至少1分钟，启动定位程序：

```bash
cd ~/src/catkinws_realsense
bash run.sh
```

然后我们将在terminal中看到被提取的必要节点，需确认“Realsense is up”消息显示如下：

若Realsense摄像头启动失败，并显示“No realsense devices were found”，这是realsense library的一个已声明bug，详见：https://github.com/IntelRealSense/librealsense/issues/3657,
用户可尝试重新拔插摄像头的USB口，作为临时解决办法；
然后用户通过如下命令打印输出，确认定位结果：

```bash
rostopic echo /mavros/odometry/out
```

在低高度飞行场景下，定位功能通常需20-30秒实现稳定。用户可查阅用于飞控的local position topic来确认定位状态，具体实现方法为在Qground地面站的Mavlink控制台输入如下指令：

```
listener vehicle_local_position
```

由于网络负载情况，有时候控制台输出不完整。用户可选择尝试如下指令：

```
listener vehicle_local_position -n 20
```

只有在x, y, z位置、速度值全部确认有效时，用户才可以尝试半自动飞行(Position mode)。飞行流程与快速启动章节相同。预期中，飞机的悬停效果与室外相当、甚至更好。
补充说明下，要确保相机前方具备一些特征，不要让飞机面对一堵白墙。

## 演示视频

