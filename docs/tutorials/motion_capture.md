# 使用Nokov度量运动捕捉系统进行定位

动作捕捉系统 （mo-cap） 是一种用于跟踪物体或人运动的技术，通常用于动画、运动分析或生物力学研究。
它通常涉及摄像头和传感器，用于捕获放置在主体上的标记的位置、方向和移动。然后处理数据以创建数字表示或动画。
Nokov 动作捕捉系统 （https://www.nokov.com/） 是该领域可用于无人机室内定位的产品之一。

## 1. 准备工作

### (1) 硬件设备

- Nokov 度量运动捕捉系统
- Kerloud 室内光流无人机
- 无人机数传地面端*1
- 地面站电脑

### (2) 操作软件

安装Qgroundcontrol 软件到地面站电脑,

Ubuntu 18.04 系统(推荐): https://github.com/cloudkernel-tech/qgroundcontrol/releases/download/v0.2.1/QGC_3.5.6_kerloud_202411.AppImage

Windows 11系统: https://github.com/cloudkernel-tech/qgroundcontrol/releases/download/v0.3/QGroundControl-pursuit-installer.exe

## 2. Nokov系统设置

请参考《ROS 与 Nokov 动作捕捉系统的通信-V2》文件,完成 XINGYING 软件的安装, VRPN软件的安装。在无人机上合适位置贴上反光球,将无人机设置成系统中的一个 Markerset,
可以通过软件查看到无人机的位姿，详细操作可以阅读《XINGYING操作说明书》。

## 3. Kerloud无人机设置

### (1) 机载ROS软件

首先参考 供电和编程界面 进入机载电脑，
参考《ROS与Nokov动作捕捉系统的通信-V2》文件中“ROS 与 XINGYING 软件的通信”，在无人机机载电脑Jetson Nano中完成vrpn软件包的安装。
保证无人机机载电脑可以自动连接到XINGYING软件的同一网络，可以ping通；之后测试启动 vrpn_client_ros可以获取对应tracker的位姿信息（详见第11页操作），例如可以在机载电脑中看到”/vrpn_client_node/Tracker2/pose”的输出，请记录好对应本机的topic名称，该输出即是机器在运动捕捉环境下的位置和姿态，我们可以用该信息输入到飞控作为定位。
对于我们Kerloud无人机，需要启动mavros节点后订阅运动捕捉的位姿输出话题，即可把运动捕捉系统的测量位姿发布给飞控。

用户可以通过topic_tools将运动捕捉发布的位姿话题转发到mavros的/mavros/vision_pose/pose话题，注意修改对应的话题名称：

```bash
#please modify the topic name from vrpn_client_node here
rosrun topic_tools relay /vrpn_client_node/<tracker_name>/pose /mavros/vision_pose/pose
```

启动mavros节点的方式为：

```bash
cd ~/src/uav_space/catkinws_offboard  #customized mavros package is inside the catkinws_offboard workspace
source devel/setup.bash
roslaunch mavros px4.launch fcu_url:=”/dev/ttyPixhawk”
```

### (2) 飞控参数设置

连接无人机和 QGC 地面站,针对运动捕捉环境,我们需要设置对应飞控的参数为:

```
EKF2_AID_MASK 24     #select vision position, vision yaw
EKF2_HGT_MODE 3      #vision height
```

注意:不同地面站版本显示的数据不一样,请确保参数数值分别为 24 和 3,可以手动输入。对于有光流配置的Kerloud无人机机型，可以勾选use optical flow选项，这样EKF2_AID_MASK为26。
重启飞控使设置生效。

## 4. 定位状态测试与室内飞行

### (1) 定位状态测试

无人机在机载电脑端需要通过三个topic来核对它的定位信息，如果设置正确，这三个定位输出的数据应该很接近。

```bash
rostopic echo /vrpn_client_node/<tracker_name>/pose   # position measurement from the motion capture system
rostopic echo /mavros/vision_pose/pose     # external localization position sent to the autopilot
rostopic echo /mavros/local_position/pose  # fused position from the autopilot
```

以上输出数据的单位需要是SI单位制

用户也可以在运动捕捉环境下手持移动无人机测试输出数据，比如往运动捕捉系统的X轴正向移动1米，则对应坐标输出X的值也会增加1。

### (2) 室内飞行

请先校准无人机加速度计、陀螺仪、磁感计等传感器，参考： https://cloudkernel.cn/kerloud-uav/quickstart-zh.html 。 只需要进行其中的传感器校准环节，其他环节均在出厂时完成。
核查无人机桨叶方向安装正确，且螺母拧紧。
启动运动捕捉系统，核查定位输出正常。
启动无人机机载电脑端的vrpn_client_ros和mavros节点，核查飞机位置和姿态正常，可以通过手持无人机验证它的显示位姿。
尝试使用POSITION模式进行半自动飞行，参考无人机用户手册快速启动章节，如果POSITION模式下可以飞行，则无人机在运动捕捉环境下可以实现正确的定位，自动飞行也没问题。
进行自动飞行前，一定要核查飞行的航路点坐标在限定范围内，如果使用提供的off_mission节点，默认的航路点坐标需要修改，其中起飞高度是5米会超过室内限制。自动飞行前软件程序一定要通过软件仿真环境测试。
