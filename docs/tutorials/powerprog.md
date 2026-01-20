# 供电和编程界面

注意：本文只适用于装载Jetson Nano的Kerloud无人机产品

英伟达Jetson Nano支持两种供电方式：Micro USB供电和DC Jack供电。Kerloud UAV系列组件默认选用DC Jack，如下图所示。
可通过将Jetson Nano载板上的J48跳选位断开，切换为Micro USB供电方式。
我们建议用户使用DC Jack进行供电，因为Micro USB最大供电电流为2A，在高负载计算场景下会存在供电不足。

使用HDMI线将Jetson Nano连接到显示屏，我们可以对机载计算机进行登录。注意，Jetson Nano仅支持机身支持HDMI接口的显示屏。默认的用户名、密码均为ubuntu，我们已预装QTcreator作为默认的软件开发环境。

用户也可以实现远程登录，前提是Jetson Nano和远程PC必须位于相同本地wifi网络中。假定 Jetson Nano的IP地址是：192.168.0.101，则可以使用如下指令进行登录：

```bash
sudo apt update
sudo apt install openssh-server
ssh ubuntu@192.168.0.101
```

为了避免更改IP地址的繁琐操作，用户可以在wifi连接选项中手动为Jetson Nano设置IP。
