# Mavros在线控制 (C++)

注意：本文适用于所有XDKJY无人机产品

mavros ROS程序包可视为运行ROS的计算机、启用MAVLink的无人机和启用MAVLink的地面站三者间的网关，其安装流程可参考：
https://dev.px4.io/v1.9.0/en/ros/mavros_installation.html 或
https://github.com/mavlink/mavros/tree/master/mavros#installation.
我们提供了一个现成的工程环境，用户可以在如下目录中开发其应用程序： ~/src/uav_space/catkinws_offboard，该目录下包含三个文件夹：mavlink、mavros和offboard_demo。由于国内网络限制，如果用户想从头开始操作，可能会在尝试访问github资源或 rosdep时出现中断。

Pixhawk设置
为了使能Pixhawk的offboard控制模式，建议用户参照如下网页中的介绍：
https://dev.px4.io/master/en/ros/offboard_control.html,
https://dev.px4.io/master/en/companion_computer/pixhawk_companion.html。
在我们的环境下，用户可以按如下内容对Pixhawk板载参数进行设置：

针对默认安装的 XDKJY mini 自驾仪:

```
MAV_0_CONFIG = TELEM 1
MAV_0_MODE = Normal
MAV_0_FORWARD = Enable
SER_TEL1_BAUD = 57600

MAV_1_CONFIG = TELEM 2
MAV_1_MODE = Onboard
MAV_1_FORWARD = Enable
SER_TEL2_BAUD = 921600
```

## 如何编译

编译环节我们采用catkin-tools，它比常规的catkin_make快很多。 有关catkin工具的详细信息，请参见：https://catkin-tools.readthedocs.io。
用户可以按以下指令对工程环境进行编译：

```bash
cd ~/src/uav_space/catkinws_offboard
catkin init  # initialize the workspace
catkin build -j2  # build the workspace with two threads only to avoid overloading in Jetson Nano
```

编译过程耗时超过十分钟，首次操作请耐心等待。使用如下指令可清除工程环境：

```bash
cd ~/src/uav_space/catkinws_offboard
catkin clean
```

## 例程代码讲解

offboard_demo文件夹下的例程实现航点飞行任务。航点信息在yaml文件中定义，其路径为src/ offboard_demo/launch/waypoints_xyzy.yaml，文件中还包含ENU坐标和相应的偏航值。该软件包支持SITL（软件在环）仿真，用户可以在实际飞行前对自身软件进行测试。

对于订阅、发布和服务信息的声明：

```
//subscriptions, publications and services
ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 10, local_pose_cb);
ros::Subscriber rc_input_sub = nh.subscribe<mavros_msgs::RCIn>
        ("mavros/rc/in", 5, rc_input_cb);
ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
```

载入航点信息，检查仿真结果：

```
// load yaml files
XmlRpc::XmlRpcValue wp_list;
private_nh.getParam("waypoints",wp_list);

// simulation flag
int simulation_flag = 0;
private_nh.param("simulation_flag", simulation_flag, 0);
```

在SITL仿真中解锁飞机：

```
if (simulation_flag == 1)
{
    //set offboard mode, then arm the vehicle
    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){
        if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard mode enabled");
        }
        last_request = ros::Time::now();
    } else {
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }
}
```

发布当前航点：

```
pose = waypoints.at(current_wpindex);
local_pos_pub.publish(pose);
```

降落后锁定：

```
if( arming_client.call(disarm_cmd) &&
    arm_cmd.response.success){
    ROS_INFO("Vehicle disarmed");
}
```
