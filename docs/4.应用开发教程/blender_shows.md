# Blender 表演无人机灯光秀制作（WSL + Skybrush Studio for Blender）

> *Hahngao 首次编辑于 2026/2/3

本文介绍在 Windows 上通过 WSL 运行 Linux 版 Blender，并安装/使用 `studio-blender`（Skybrush Studio for Blender）插件来制作无人机灯光秀（show）。

## 1. 安装与运行 Blender（WSL GUI）

### 1.1 配置 WSL 运行 Linux GUI 应用

按微软官方指南完成 WSLg（Linux GUI）环境配置：

- https://learn.microsoft.com/zh-cn/windows/wsl/tutorials/gui-apps

完成后，在 WSL 里运行 GUI 程序会以“原生窗口”的方式显示在 Windows 桌面上。

### 1.2 在 WSL 中安装 Blender

在 WSL 终端执行：

```bash
# 更新软件源
sudo apt update

# 安装 Blender
sudo apt install blender -y
```

随着Skybrush Studio的更新，需要更高版本的Blender，所以apt安装 Blender 版本不符合，但是通过apt安装可以把需要的依赖性补齐，所以现在只需要下载新版本的Blender即可。下载地址：https://download.blender.org/release/Blender4.4/

```bash
# 下载
wget https://download.blender.org/release/Blender4.4/blender-4.4.3-linux-x64.tar.xz

# 解压
tar -xf blender-4.4.3-linux-x64.tar.xz

# 将文件移动到系统目录（推荐）
sudo mv blender-4.4.3-linux-x64 /opt/blender-4.4.3

# 安装依赖库（可能需要），大部分之前apt旧版本blender已经完成。 
sudo apt update
sudo apt install libxxf86vm1 libxi6 libglx-mesa0 libxrender1 libxkbcommon0 libxfixes3 libxcb-glx0 libx11-xcb1 libxxf86vm1 libxinerama1

#到目录下运行blender 4.4.3，在wsl下开启blender
cd /opt/blender-4.4.3/
./blender
```

## 2. 安装 studio-blender 插件（Skybrush Studio）

`studio-blender` 源码目录位于：

- Windows 路径：`d:\CodeProject\UAVDocs\studio-blender\`
- WSL 路径：`/mnt/d/CodeProject/UAVDocs/studio-blender/`

插件在 Blender 中一般以 “Skybrush Studio” 的名称出现。

### 2.1 方式 A：从本仓库构建 ZIP（推荐）

该仓库提供了构建脚本，会在 `dist/` 目录生成可直接在 Blender 中安装的 `.zip` 插件包。

在 WSL 里执行：

```bash
cd /mnt/d/CodeProject/UAVDocs/studio-blender

# 构建脚本依赖 Python3 / venv / zip，以及 uv（用于导出依赖）
sudo apt update
sudo apt install -y python3 python3-venv python3-pip zip

# 安装 uv（如果 uv 命令不可用，确认 ~/.local/bin 在 PATH 中）
python3 -m pip install --user uv

# 构建插件 ZIP
bash etc/scripts/create_blender_dist.sh

# 查看产物
ls -al dist/*.zip
```

完成后会得到类似 `dist/<name>-<version>.zip` 的文件。

### 2.2 在 Blender 中安装 ZIP 插件

1. 打开 Blender
2. 进入 Edit -> Preferences -> Add-ons
3. 右上角下拉菜单选择 Install from Disk...
4. 选择上一步生成的 `dist/*.zip`（在 WSL 的文件选择器中通常需要用 `/mnt/d/...` 路径访问）
5. 安装完成后，在列表里勾选启用 “Skybrush Studio”
6. 展开该插件条目（左侧 `>`），按需设置 Server URL / API Key

### 2.3 Blender 4.2+：启用联网权限（重要）

从 Blender 4.2 开始，Blender 提供 “Allow Online Access” 选项。Skybrush Studio 需要访问在线资源（社区服务器或自建服务器）来完成部分功能（例如自动过渡计算、部分导出）。

在 Blender 中：

- Edit -> Preferences -> System -> Network
- 勾选 Allow Online Access
- 必要时提高 Network timeout（大编队/长时长导出更需要）

## 3. studio-blender 使用指南（快速上手）

以下流程摘取并按本项目使用场景做了整理，覆盖从“初始化场景”到“导出 show”的常用步骤。

### 3.1 打开 Skybrush 面板

在 Blender 的 3D Viewport 中按 `N` 打开右侧栏，安装插件后会出现与 Skybrush Studio 相关的多个标签页（例如 Skybrush、Formations、LEDs、Pyro、Safety & Export）。

### 3.2 初始化编队与起飞网格

在 Formations 相关面板中：

1. 点击 Create Takeoff Grid
2. 在弹窗中设置起飞网格参数并确认

该步骤通常会创建 Skybrush 需要的基础集合（Drones、Formations、Templates），并把起飞网格加入到 storyboard，且在视图中初始化无人机对象。

### 3.3 自动起飞 / 返航 / 降落

典型 show 的技术段（起飞、过渡、返航、降落）可以自动生成：

- Takeoff：生成起飞流程并加入 storyboard
- RTH：生成 Return-to-home
- Land：生成降落流程并加入 storyboard

### 3.4 创建 Formation（队形）

在 Formations 面板中用 `+` 创建 formation。formation 可以来自：

- 多个独立对象（一个对象对应一架无人机）
- 单个网格的选中顶点（一个顶点对应一架无人机）

注意让 formation 的 marker 数量与机群数量匹配；数量不足时用占位 marker 补齐，数量过多会在自动过渡计算时触发错误。

### 3.5 用 Storyboard 串联队形并计算自动过渡

1. 在 Storyboard 面板中按时间顺序添加 formations
2. 设置每段的开始时间与持续时间，过渡类型优先使用 Auto
3. 点击 Recalculate transitions 自动计算 formations 之间的过渡

### 3.6 安全检查（Safety Check）

在 Safety & Export 中启用 Safety Check，配置速度/高度/距离等阈值，用于实时发现风险（违规项会在视图中高亮）。

### 3.7 灯光（LEDs）与高级效果

在 LEDs 面板中可对选中的无人机在指定帧打灯光关键帧（Apply / Fade to），也可以使用 Light Effects 面板创建参数化的高级灯光效果。

### 3.8 导出 show（.skyc 等）

在 Safety & Export -> Export 面板中导出：

- Export to .skyc：导出 Skybrush 编译格式 show 文件
- Export to .csv：按固定采样导出轨迹与颜色数据
- Export to validation .pdf：导出验证报告

部分格式/能力可能依赖服务器或许可证；若使用社区服务器，能力可能受限。

## 4. 参考资料（本仓库）

- `studio-blender` 源码与构建入口：[README.md](file:///d%3A/CodeProject/UAVDocs/studio-blender/README.md)
- 插件安装说明（上游文档节选来源）：`studio-blender/doc/modules/ROOT/pages/install.adoc`
- 快速入门教程（上游文档节选来源）：`studio-blender/doc/modules/ROOT/pages/tutorials/easy-drone-show-design.adoc`
