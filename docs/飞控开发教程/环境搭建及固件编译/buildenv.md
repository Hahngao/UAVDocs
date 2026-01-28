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
   - 安装之后，还需要首次打开，完成用户名和密码的配置
     
       ```cmd
       wsl.exe -d Ubuntu-24.04
       ```

2. **Windows 下安装 [VS Code](https://code.visualstudio.com/) 或推荐使用TRAE**

3. **注意以上过程，wsl安装出错，可能是网络问题，可能是需要注销重新安装**

4. **WSL2 Ubuntu 下安装 git gitk git-gui**

   - 在 Ubuntu 终端中输入以下命令安装

     ```bash
      # 更新可用软件包列表，从软件源获取最新的版本信息
      sudo apt update
      
      # 升级所有已安装的软件包到最新版本
      sudo apt upgrade
      
      # 安装Git版本控制系统及相关图形化工具：
      # git - 核心版本控制工具
      # gitk - Git仓库可视化浏览器（基于Tk）
      # git-gui - Git图形化界面工具
      sudo apt install git gitk git-gui
     ```

3. **克隆项目源码**

   - 在 Ubuntu 终端中输入以下命令克隆，建议使用SSH，配置方法：[github配置SSH-Key保姆级教程 - 知乎](https://zhuanlan.zhihu.com/p/688103044)

     ```bash
     git clone git@github.com:ArduPilot/ardupilot.git
     ```

4. **配置工作环境**
   
   - 在 VS Code 中点击`远程`按键，选择`WSL`，选择安装的 Ubuntu-24.04，等待目录切换完成
   - 在 VS Code 中 `File -> Open Folder` 打开 `ardupilot` 文件夹
   - 切换到 master 分支
   
5. **安装必要依赖**

   - 打开终端 (`Ctrl + ~`)

   -   输入以下命令安装依赖

       ```bash
       sudo apt-get update
       sudo apt-get install -y binutils-arm-none-eabi gcc-arm-none-eabi
       chmod 777 ./Tools/environment_install/install-prereqs-ubuntu.sh
       ./Tools/environment_install/install-prereqs-ubuntu.sh -y
       . ~/.profile
       ```

   -   耐心等待终端输出，需要挂梯子

       ```bash
       ---------- ./Tools/environment_install/install-prereqs-ubuntu.sh end ----------
       ```

       依赖安装成功

   -   强烈建议安装 `ARDUPILOT DEVENV` 扩展以检查依赖和工具是否齐全，如有缺失工具和依赖一定要补齐

## 三、PX4环境搭建及固件编译步骤
