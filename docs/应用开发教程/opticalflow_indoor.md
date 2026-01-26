# 室内光流定位

注意：本教程适用于XDKJY系列无人机中的光流配置款型。
搭载光流配置的XDKJY无人机，为用户以少量设置实现室内定位提供了方便的选择。该平台配备有 PX4FLOW光流传感器 和 北醒TF-Luna激光测距仪 ，
PX4FLOW是PX4社区中的一款基于摄像头的智能传感器，在水平速度估计方面表现出色；激光测距仪则可在高度测量中提供厘米级精度。
该配置飞机在具备纹理地面的室内环境中，水平、垂直定位精度分别可达到0.1m、0.05m级别。此外，飞机也支持室外环境下的无GPS飞行。

## 如何设置

### 环境要求

使用光流传感器，需要室内地面具备丰富的纹理，因此建议用户使用棋盘纸或其他图案来对地面进行覆盖、装饰，棋盘纸图片可通过此 链接 下载。另外，PX4FLOW相机的光照条件也要足够。

### 固件设置

搭载光流配置的XDKJY无人机在出厂阶段已正确配置。面向室内场景，可在QGroundcontrol界面中配置如下参数：

```
SENS_EN_TF   2
SENS_EN_PX4FLOW: 1
EKF2_HGT_MODE: 2
EKF2_AID_MASK: 2
EKF2_MAG_TYPE: 0
EKF2_OF_POS_Y: -0.07
```

参数说明如下：

- SENS_EN_TF: 启动北醒TF测距传感器驱动程序
- SENS_EN_PX4FLOW: 启动PX4FLOW驱动程序
- EKF2_HGT_MODE: 将激光测距仪设置为主要高度信息源
- EKF2_AID_MASK: 启动EKF估计中的光流设置
- EKF2_MAG_TYPE: 设置磁力计航向测量为自动模式
- EKF2_OF_POS_Y: 设置光流传感器在体坐标系Y轴的相对位置

### 光流对焦设置

PX4FLOW传感器的相机镜头在出厂时已调好，官方调参指引可参见： https://docs.px4.io/master/en/sensor/px4flow.html。
简要陈述：用户可使用提供的micro-usb线，将PX4FLOW传感器接入运行有QGC的电脑，
然后在QGC设置菜单中选择PX4FLOW，很快就能在QGC界面中看到来自PX4FLOW相机的图像。
用户可简单地放置一本书在地面，将相机抬高到想要飞行的高度（一般为1~3米），拧下固定螺丝，然后通过拧松、拧紧镜头寻找焦点位置以实现对焦。
当书本的边缘在图像中清晰可见时，调焦结果可视为能接受。

### 传感器数据查看

设置后可以通过QGC地面站交互界面查看确认光流传感器和激光测距传感器数据。

## 演示视频

该视频演示了按照快速启动指南中步骤实现的半自动室内飞行。

## 参考文献

Dominik Honegger, Lorenz Meier, Petri Tanskanen and Marc Pollefeys. An Open Source and Open Hardware Embedded Metric Optical Flow CMOS Camera for Indoor and Outdoor Applications.
