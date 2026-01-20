# Airsim环境下的硬件在环仿真

注意：本教程是我们 Kerloud DASA service 服务的组成部分

## 背景

Airsim 是微软公司于 2017 年创建的一个开源项目，它是一个支持自主无人机、无人车的仿真平台。该平台基于 Unreal Engine 构建，具有卓越的3D可视化效果和高保真物理引擎。
因此，它为高端用户提供了现有工具（如 Gazebo 和 jMAVsim）的替代解决方案，并实现了可与游戏和电影中场景相媲美的梦幻般的模拟场景。
硬件在环 (HIL) 仿真是 PX4 固件支持的另一种仿真模式。与 Offboard Control with Mavros (C++) 教程中提到的SITL仿真不同，该模式下飞控固件运行在真实的自驾仪上，
因此所有代码都是在实体上进行测试。仿真方法可以大大加快我们的软件开发，降低实验风险。
本教程为感兴趣的用户提供了一个起点，用户可以熟悉 Airsim 环境并了解其如何在硬件级别与我们的 Kerloud 自动驾驶仪进行交互。

## 先决条件

下面列出了需要的工具（已进行全面测试）：

- Kerloud autopilot ：固件版本为v1.10.0以上
- 一个 RC 接收器和一个遥控器
- 算力强大的个人电脑

推荐的 PC 硬件配置要求可参考 这里。
在我们的案例中，硬件详细信息为：CPU（Intel i7-13700KF）、显卡（Nvidia Geforce RTX 3060）、硬盘（1TB SSD）和 32GB RAM 内存。PC安装了Ubuntu 18.04 和Unreal Engine 4.27 plus。

## 实验设置

### 自驾仪设置

Kerloud 自驾仪的HIL模式可以参照 https://docs.px4.io/main/en/simulation/hitl.html 中的说明进行设置。简而言之，启用HIL模式，QGround地面站应该只配置 UDP通信。自动驾驶仪将通过自身USB 端口与 Airsim 平台交互mavlink 消息。

### Airsim软件

Airsim 平台是按照 指南 从我们 PC 中的源代码进行编译的。这里我们使用最新主分支的源代码（commit date：2022-7-21）。建议用户使用 Block 环境进行初步测试。我们使用 Epic Games Marketplace中的Landscape mountain 环境，
详情可参考 https://microsoft.github.io/AirSim/unreal_custenv/ 。环境视图如下：

将 Airsim 插件文件夹复制到自定义项目后，我们只需单击 “Play” 按钮即可开始模拟。默认的四旋翼飞行器可以像真实场景般出现。

## 演示

Airsim平台远程手动控制的演示可以在下方视频中查看。
