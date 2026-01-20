# 基于立体视觉和GPU加速的视觉里程系统(VINS)

注意：本文只适用于 Kerloud SLAM Indoor无人机产品
Kerloud SLAM Indoor配备有Nvidia TX2模块和Intel Realsense D435i立体摄像头。凭借更强大的GPU内核，硬件配置能够通过纯视觉实现完全自主的室内定位。VINS系统为视觉辅助自主控制和其他应用（如 SLAM、AR（增强现实））开辟了广泛的机会。该产品适合需要深入研究计算机视觉和机器人自主控制的高端用户。

## 背景

VINS系统是基于香港科技大学空中机器人组 (https://uav.hkust.edu.hk) 的原创作品，相关资源可以在本教程的参考部分中找到。该方法将IMU数据与机载计算机视觉无缝融合，可输出实时姿态估计。它提供了从前端IMU预集成到后端全局优化的完整通路。

## 包含内容

请注意，原始开源代码无法直接部署在无人机上，需要复杂的设置才能让用户实现成功飞行。我们所做的工作包括以下几个方面：

- 具备必要前提（如OpenCV、英特尔实感库等）的完整软件环境配置
- 包含内参、外参的相机校准
- VINS与Realsense立体视觉融合的配置
- 与飞控集成以实现底层状态估计
- 数据集仿真、手持移动和自主飞行测试

## 如何使用

位于路径~/src/catkin_ws_VINS-Fusion-gpu中的VINS系统工作区包含如下软件包：

- VINS-Fusion-gpu：具备GPU加速的VINS算法包，专为Kerloud无人机定制
- vison_opencv：用于连接ROS与OpenCV的软件包
- pose_converter：用于连接里程计系统与飞控系统的软件包

可使用如下指令启动VINS系统：

```bash
# terminal 1
roscore

# terminal 2: launch mavros node
cd ~/src/catkinws_realsense \
&& source devel/setup.bash \
&& roslaunch mavros px4.launch fcu_url:=/dev/ttyPixhawk:921600

# terminal 3: launch realsense driver node
cd ~/src/catkinws_realsense \
&& source devel/setup.bash \
&& roslaunch realsense2_camera rs_d435i_kerloud_stereo_slam.launch

# terminal 4: launch VINS node
cd ~/src/catkin_ws_VINS-Fusion-gpu \
&& source devel/setup.bash \
&& rosrun vins vins_node /home/ubuntu/src/catkin_ws_VINS-Fusion-gpu/src/VINS-Fusion-gpu/config/kerloud_tx2_d435i/realsense_stereo_imu_config.yaml

# (Optional) terminal 5: launch vins-loop-fusion node
cd ~/src/catkin_ws_VINS-Fusion-gpu \
&& source devel/setup.bash \
&& rosrun loop_fusion loop_fusion_node /home/ubuntu/src/catkin_ws_VINS-Fusion-gpu/src/VINS-Fusion-gpu/config/kerloud_tx2_d435i/realsense_stereo_imu_config.yaml

# terminal 6: launch pose_converter node
cd ~/src/catkin_ws_VINS-Fusion-gpu \
&& source devel/setup.bash \
&& roslaunch pose_converter poseconv.launch

# (Optional) terminal 7: launch rviz for visualization
cd ~/src/catkin_ws_VINS-Fusion-gpu \
&& source devel/setup.bash \
&& roslaunch vins vins_rviz.launch
```

或者更简单地：

```bash
cd ~/src/catkin_ws_VINS-Fusion-gpu \
&& bash run.sh
```

## 演示

### 使用数据集仿真

基于Euroc数据集的仿真测试可以使用以下命令进行：

```bash
cd ~/src/catkin_ws_VINS-Fusion-gpu \
&& source devel/setup.bash \
&& roslaunch vins vins_rviz.launch \
&& rosrun vins vins_node ~/src/catkin_ws_VINS-Fusion-gpu/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml

# Optional:
cd ~/src/catkin_ws_VINS-Fusion-gpu \
&& source devel/setup.bash \
&& rosrun loop_fusion loop_fusion_node ~/src/catkin_ws_VINS-Fusion-gpu/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \
&& rosbag play YOUR_DATASET_FOLDER/MH_01_easy.bag
```

### 地面测试

VINS手持测试：

VINS动态地面测试：

### 自主室内悬停

## 参考资料

- VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, IEEE Transactions on Robotics. 2018.
- https://github.com/HKUST-Aerial-Robotics/VINS-Mono
- https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
