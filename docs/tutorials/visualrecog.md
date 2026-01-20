# 实时视觉识别

注意：本文只适用于装载Jetson Nano的Kerloud无人机产品

开启视觉识别介绍所使用的官方示例项目是jetson-inference资源库，链接为：
https://github.com/dusty-nv/jetson-inference，这是一个适用于英伟达Jetson Nano的hello-world示例。
该库支持图像分类、物体定位及图像分割，用户可以根据自身兴趣进一步探究项目细节。本文中我们仅对视觉识别的基础部分加以阐述，并解决代码调试中的常见问题。

## 代码结构

主要目录列举如下，读者可阅读顶层的CMakeLists.txt进一步理解：

- c/: 用于detectNet、imageNet及segNet的网络类
- data/: 网络数据和图像
- docker/: 用于提取、创建、运行本地镜像的脚本
- doc/: 文件
- examples/: 用于部署各种网络的主执行文件
- plugin/, python/, tools/, utils/: 其他功能、库及脚本

## 如何编译

用户可参考官方关于该项目编译的说明文件，链接为：
https://github.com/dusty-nv/jetson-inference/blob/master/docs/building-repo-2.md。
然而，由于中国境内网络限制，用户通过代码访问github或nvida的被禁网页时，可能会被中断。于是我们修改代码，
并将代码及全部网络数据下载到目录~/jetson-inference下作为库的组成部分。用户可按照如下指令轻松实现编译：

```bash
sudo apt-get update
sudo apt-get install git cmake libpython3-dev python3-numpy
cd ~/jetson-inference
git checkout master_ck
mkdir build
cd build
cmake ../
make -j2
sudo make install
sudo ldconfig
```

## 如何运行

关于如何部署代码的参考链接如下：
https://github.com/dusty-nv/jetson-inference/blob/master/docs/imagenet-console.md,
https://github.com/dusty-nv/jetson-inference/blob/master/docs/imagenet-camera-2.md,
可使用如下指令识别文件夹中图片：

```bash
cd ~/jetson-inference/build/aarch64/bin

./imagenet-console.py --network=googlenet images/orange_0.jpg output_0.jpg  # --network flag is optional (default is googlenet)
```

这些命令将在bin文件夹下生成一张新图片，标有结果和一个置信度。

实时视频识别样例：（请注意，板载摄像头连接到CSI端口0上）

```bash
./imagenet csi://0                    # MIPI CSI camera
```

为了获得更好的识别结果，强烈建议使用干净的背景。此处样例中所部署的googlenet在视频识别方面效果不佳。

## 参考资料

- https://developer.nvidia.com/embedded/twodaystoademo
- https://developer.nvidia.com/embedded/learn/tutorials
- https://developer.nvidia.com/embedded/learn/getting-started-jetson
- https://developer.nvidia.com/embedded/community/support-resources

