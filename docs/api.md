# 应用程序接口 (API)

我们为XDKJY无人机系列的开发提供开源API，可选的API接口将会支持C++、python及其他语言，详情会在本部分持续更新。请注意，我们假设用户熟悉 ROS 的基本概念，不了解的用户建议阅读 http://wiki.ros.org/ROS/Tutorials 中的官方教程。

## 1. ROS API (C++)

ROS（Robot Operating System）中的 API 是基于PX4 社区的官方 MAVROS包 实现的。
我们维护的资源库为：

- mavros（dev_XDKJYuav 分支）: https://github.com/cloudkernel-tech/mavros
- mavlink（dev_XDKJYuav 分支）: https ://github.com/cloudkernel-tech/mavlink-gdp-release

请注意，offboard控制演示代码仅随XDKJY机器提供，因此不是开源的。更多编程细节将在教程部分 Mavros在线控制 (C++) 中阐述。本文介绍我们 XDKJY 无人机案例中常用的主题和服务。

### 1.1 话题

#### (1) ~mavros/extended_state

可以通过订阅该主题来获取当前的无人机状态，消息类型在 这里 定义。我们可以相应地获取着陆状态，例如：如果landed_state等于LANDED_STATE_IN_AIR，则无人机在空中。

```
# Extended autopilot state
#
# https://mavlink.io/en/messages/common.html#EXTENDED_SYS_STATE

uint8 VTOL_STATE_UNDEFINED = 0
uint8 VTOL_STATE_TRANSITION_TO_FW = 1
uint8 VTOL_STATE_TRANSITION_TO_MC = 2
uint8 VTOL_STATE_MC = 3
uint8 VTOL_STATE_FW = 4

uint8 LANDED_STATE_UNDEFINED = 0
uint8 LANDED_STATE_ON_GROUND = 1
uint8 LANDED_STATE_IN_AIR = 2
uint8 LANDED_STATE_TAKEOFF = 3
uint8 LANDED_STATE_LANDING = 4

std_msgs/Header header
uint8 vtol_state
uint8 landed_state
```

#### (2) ~mavros/setpoint_position/local, ~mavros/setpoint_position/global

该主题用于在本地或全局坐标系下发布位置设定值，然后无人机将被引导至所需位置。请注意，ROS 中使用的坐标系为ENU坐标系，而自驾仪中使用NED坐标系。消息类型则分别为 geometry_msgs/PoseStamped 或 mavros_msgs/GlobalPositionTarget 。

#### (3) ~mavros/setpoint_velocity/cmd_vel_unstamped, ~mavros/setpoint_velocity/cmd_vel

这两个主题可用于在本地坐标系中发布速度设定点，车模式和多旋翼模式下通用。它们间的唯一区别是在速度信息中是否带有时间戳。消息类型为 geometry_msgs/Twist 。

### 1.2 服务

#### (1) ~mavros/cmd/命令

该服务用于设置无人机的各种模式，如上锁/解锁、设置offboard模式等。消息类型为 mavros_msgs/CommandLong 。
举例来说，假定我们要对机体进行解锁，则需要三个步骤。

第 1 步，我们必须定义服务客户端：

```
//service for arm/disarm
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
```

第2步，我们定义arm命令如下：

```
mavros_msgs::CommandBool arm_cmd;
arm_cmd.request.value = true;
```

第3步，我们可以调用服务了：

```
arming_client.call(arm_cmd)
```

## 2. ROS API（python）

与 C++ 版的 ROS API 类似，ROS python API 也是基于我们维护的 mavros 和 mavlink 包来。唯一的区别是上面提到的所有主题和服务都应基于python 使用。
我们维护的资源库为：

- mavros（dev_XDKJYuav 分支）: https://github.com/cloudkernel-tech/mavros
- mavlink（dev_XDKJYuav 分支）: https://github.com/cloudkernel-tech/mavlink-gdp-release

offboard控制例程代码也随XDKJY机器一起提供，在教程 Mavros在线控制 (Python) 部分中有详细阐述。为简洁起见，低级别消息通信在 Px4Controller 类中处理，而用于用户编程的友好 API 则包装在 Commander 类中，部分列举如下：

- move(x,y,z, BODY_FLU=False)：请求机体移动到 FLU 坐标系或 ENU坐标系下定义的所需位置
- turn(yaw_degree)：请求飞行器在飞行过程中转向所需的偏航角
- land()：请求机体着陆
- hover()：请求机体悬停在当前位置
- arm()：请求机体解锁
- disarm()：请求机体上锁
- return_home(height)：请求机体返回
