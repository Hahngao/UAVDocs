# 快速启动

快速启动流程包含如下步骤：

- 软硬件工具准备
- RC接收器连接
- 固件设置
- 桨叶安装
- Position模式下进行手动飞行

## 1. 软硬件工具准备

用户在使用前需要准备好下述工具

### 1.1 硬件工具

- 一台安装windows11或者ubuntu 18.04系统的计算机
- XDKJY无人机
- 数传天线地面端
- 遥控器

### 1.2 软件工具

#### (1) Qgroundcontrol地面站软件

windows11系统：

百度云下载链接： https://pan.baidu.com/share/init?surl=NJe8LAfI1Qg6n9sEHW54-g   提取码：da2e

ubuntu 18.04系统：

https://github.com/cloudkernel-tech/qgroundcontrol/releases/download/v0.2/QGroundControl_flyingrover.AppImage

下载后用户需要执行下述指令使软件生效

```bash
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y

# Then logout and login again to enable the change to user permissions.

chmod +x ./QGroundControl_flyingrover.AppImage
./QGroundControl_flyingrover.AppImage
```

#### (2) 数传串口驱动(仅限windows11系统)

百度云下载链接: https://pan.baidu.com/s/1391Qkr-uLmmnIo6WG9F6qg 提取码: gw84

## 2. RC接收器连接

XDKJY UAV系列默认发货清单中不含RC接收机、发射机，建议用户选用支持SBUS协议的型号。
我们需要将RC接收机接入Pixhawk飞控，机身预留的接入排线如下所示：

建议用户使用魔术贴将RC接收机固定到机身，同时需注意避免接收机的天线被桨叶损坏。

## 3. 固件设置

机体内的Pixhawk飞控单元已加载基于V1.10.0版本的稳定px4固件，其参数在出厂阶段已通过实际飞行测试调优。
通常情况下，用户无需更改姿态、位置控制增益这类板载参数。px4的官方设置可参考：
https://dev.px4.io/master/en/setup/config_initial.html。
出于运输环节振动和地区差异考虑，强烈建议用户对罗盘、陀螺仪、加速度计传感器进行校准，以避免出现未知的异常。
使用Micro USB线或者数传天线将pixhawk飞控连接到一台运行Qgroundcontrol (QGC) 的电脑，可以轻松实现传感器校准。
机体设置:
XDKJY UAV系列机体默认为通用四轴无人机，不需要额外设置，如下所示：

如果用户点击QGC界面右上角的Apply按钮，则无人机出厂设置会丢失，引发不必要的飞行问题，所以这一步设置是不推荐的。

### 传感器校准

传感器可按照QGC中的提示进行校准：

- GPS模块内装有外置罗盘，建议用户在进行罗盘校准前先将GPS接入飞机，这样可以同时对板载、外置罗盘进行校准
- 陀螺仪校准期间，任何微小的动作都会导致校准失败或引入无法接受的陀螺仪偏差，因此用户在校准时需使飞机保持绝对静止
- 加速度计校准时，可将飞机置于平整桌面或竖直墙面，不建议手持进行校准

### RC校准

对于RC操作，我们可以选择右手模式（模式1，日本手）或左手模式（模式2，美国手）。点击“校准”按钮，用户可按照QGC指令轻松实现校准。

### 飞行模式设置

用户应分配一个通道（如通道5）用于飞行模式设置，而分配另一个（如通道7）用于外部控制触发。
对于XDKJY UAV系列，建议用户至少设置 STABILIZED、POSITION、OFFBOARD三种模式。
STABILIZED、POSITION模式用于通过RC接收机实现手动飞行。这里需要强调：
STABILIZED模式仅适用于经验丰富的飞手——该模式下摇杆输入信号映射到姿态基准，而POSITION模式下摇杆输入信号则映射到速度基准。
在OFFBOARD模式下，无人机接收机载计算机指令，完成如航点飞行、轨迹跟踪和视觉跟踪等自主任务。另外还有一种叫做MISSION的模式，
该模式下无人机将执行自主航点飞行任务，MISSION模式通常借助地面站进行操作。

## 4. 桨叶安装

出于运输安全考虑，发货时桨叶未安装到机身。
我们需要按照下方四旋翼配置示意图正确安装桨叶，安装时需格外注意旋转方向，使用包装中提供的便携扳手可提高效率。

注意: 在进行校准操作和其他拿在手上的调试时，用户需要切记将桨叶拆除，以免发生危险！

## 5. Position模式飞行

此处步骤只适用于室外飞行的XDKJY无人机。对于光流或者视觉定位飞行的无人机款型，请参考对应的教程说明。

Position模式是用户实现无人机飞行的最简单方式，基本操作按顺序如下：

- 确保电池电量充足，电压需要至少高于4v每cell，对于4S电池，电压需要高于16v为宜
- 将带有接口的电池可靠安装到飞机，确保飞行中不会脱落
- 确保遥控器所有开关均处于初始位置，且油门降到最低
- 电池接通上电
- 将数传天线连接到电脑，并启动QGC地面站软件对无人机进行监测，有问题地面站会给出最明确的提示信息
- 等待GPS锁星，通常如果GPS信号
