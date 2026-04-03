# Mavros在线控制 (Python)

注意：本文适用于所有XDKJY无人机产品

python 语言使用mavros 进行offboard控制的方式与C++ 案例相同。该方法是利用 mavros 包中的标准主题或服务来检索车辆信息或发送命令。 工作区位于 ~/src/uav_workspace/pythonuav_ros，所有必要的软件依赖在出厂阶段已配置。

## 1. 环境设置

本教程的依赖项为：

- mavros：具有不同ROS版本的标准mavros包，安装指令：sudo apt install ros-<ROS_VERSION>-mavros
- mavlink：具有不同ROS版本的标准mavlink包，安装指令：sudo apt install ros-<ROS_VERSION>-mavlink

用户可以参考工作区中的 README 文件了解详情。

## 2. 代码说明

工作区包含两个程序文件：commander.py 和 px4_mavros_run.py。command.py（指令接口）为应用开发提供了一个友好接口，px4_mavros_run.py（控制接口）封装了与mavros通信的底层消息。

### 2.1 指令接口

指令接口为用户编程提供友好的 API，如 move()、turn()、hover()、land() 等命令。接口示例如下：

```
if __name__ == "__main__":

    # the mission performs a square waypoint flight with position target defined in FLU frame
    con = Commander()
    time.sleep(2)
    con.move(5, 0, 0)
    time.sleep(5)
    con.move(0, -5, 0)
    time.sleep(5)
    con.move(-5, 0, 0)
    time.sleep(5)
    con.move(0, 5, 0)
    time.sleep(5)
    con.land()
    time.sleep(5)
```

commander类首先将被实例化为一个对象，然后休眠 2 秒。默认情况下，程序将要求机体前往 ENU 坐标系下定义的几个航路点。请依据情况调整休眠时间，以确保机体能够到达所需位置。基于例程，用户可以轻松添加更多的无人机动作，如悬停、转弯等。

### 2.2 控制接口

控制接口实现与mavros网关的底层通信，大多数情况下无需更改。入口函数为Px4Controller类中的start()，函数在执行px4_mavros_run.py时会被调用。
start() 函数中，首先会初始化 ros 节点，然后程序将等待 IMU 航向数据就绪，以便为机体控制提供初始目标位姿。该程序将首先命令机体起飞到所需高度（默认为 3.2m）。

```
rospy.init_node("offboard_node")
for i in range(10):
    if self.current_heading is not None:
        break
    else:
        print("Waiting for initialization.")
        time.sleep(0.5)
self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                             self.local_pose.pose.position.y,
                                             self.local_pose.pose.position.z + self.takeoff_height,
                                             self.current_heading)
```

然后程序将解锁机体并自动切换到offboard模式：

```
for i in range(100):
    self.local_target_pub.publish(self.cur_target_pose)
    self.arm_state = self.arm()
    self.offboard_state = self.offboard()
    time.sleep(0.05)
```

此后，程序将进入一个while循环来响应来自指令接口的不同命令，包括位置设定点、转弯、悬停等。

```
while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):

    self.local_target_pub.publish(self.cur_target_pose)

    if (self.state is "LAND") and (self.local_pose.pose.position.z < -1.0):

        if(self.disarm()):

            self.state = "DISARMED"


            time.sleep(0.05)
```

## 3. 如何运行SITL仿真

强烈建议用户在实际飞行前进行 SITL 仿真测试，SITL环境的搭建过程可以参考之前的C++版offboard控制教程。
假定固件库位于 ~/src/Firmware 目录下，基于gazebo的SITL仿真可通过如下方式启动：

```bash
# launch a new terminal
cd ~/src/Firmware
make px4_sitl_default gazebo

# launch a new terminal
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

# launch a new terminal
cd ~/src/uav_workspace/pythonuav_ros
python3 px4_mavros_run.py

# launch a new terminal
cd ~/src/uav_workspace/pythonuav_ros
python3 commander.py
```

## 4. 实际飞行

建议用户严格按照以下流程完成测试：

- 用绑带固定好电池，确保在飞行过程中不会掉落
- 确保所有开关都处于初始位置，且油门拉至最低
- 上电开机
- 等待 GPS 锁星：通常 2-3 分钟后我们会听到提示音，并且 pixhawk 中的主 LED 会变绿
- 确保安全员处于待命状态，且机体周围没有人。等待几分钟让机载计算机启动，通过本地 wifi 网络远程登录，确认 ~/src/uav_workspace/pythonuav_ros/commander.py 中定义的航点任务，然后运行如下命令启动offboard控制模块

```bash
# launch a terminal (users can source customized mavros & mavlink workspace alternatively)
roslaunch mavros px4.launch fcu_url:="/dev/ttyPixhawk:921600"

# launch a new terminal
cd ~/src/uav_workspace/pythonuav_ros
python3 px4_mavros_run.py

# launch a new terminal
cd ~/src/uav_workspace/pythonuav_ros
python3 commander.py
```

机体将起飞并自主执行所定义的任务。如果发生任何意外，请务必切换到 POSITION 模式。
