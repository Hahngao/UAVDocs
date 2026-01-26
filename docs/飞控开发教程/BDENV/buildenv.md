# APM、PX4环境搭建及固件编译

> *Hahngao 编辑于 2026/1/26

## 一、需求说明


## 二、AMP环境搭建及固件编译步骤
### (一)环境准备

1. **安装WSL2 Ubuntu24.04**

   - 在 Windows 11 终端中输入以下命令安装

     ```cmd
     wsl.exe --install Ubuntu-24.04
     ```

2. **Windows 下安装 [VS Code](https://code.visualstudio.com/) 或推荐使用TRAE**

3. **在Windows开始菜单找到Ubuntu，直接打开**

4. **WSL2 Ubuntu 下安装 git gitk git-gui**

   - 在 Ubuntu 终端中输入以下命令安装

     ```bash
     sudo apt update
     sudo apt upgrade
     sudo apt install git gitk git-gui
     ```

5. **克隆项目源码**

   - 在 Ubuntu 终端中输入以下命令克隆

     ```bash
     git clone https://github.com/ArduPilot/ardupilot.git
     ```

6. **配置工作环境**
   - 在 VS Code 中点击`远程`按键，选择`WSL`，选择安装的 Ubuntu-24.04，等待目录切换完成
   - 在 VS Code 中 `File -> Open Folder` 打开 `ardupilot` 文件夹
   - 切换到 master 分支

7. **安装必要依赖**

   - 打开终端 (`Ctrl + ~`)

   -   输入以下命令安装依赖

       ```bash
       sudo apt-get update
       sudo apt-get install -y binutils-arm-none-eabi gcc-arm-none-eabi
       chmod 777 ./Tools/environment_install/install-prereqs-ubuntu.sh
       ./Tools/environment_install/install-prereqs-ubuntu.sh -y
       . ~/.profile
       ```

   -   等待终端输出

       ```bash
       ---------- ./Tools/environment_install/install-prereqs-ubuntu.sh end ----------
       ```

       依赖安装成功

   -   强烈建议安装 `ARDUPILOT DEVENV` 扩展以检查依赖和工具是否齐全，如有缺失工具和依赖一定要补齐

## 三、PX4环境搭建及固件编译步骤
