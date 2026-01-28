# ArduPilot SITL 仿真教程

本文档基于 [ArduPilot 官方文档](https://ardupilot.org/dev/docs/copter-sitl-mavproxy-tutorial.html) 编写，详细介绍了如何使用 SITL (Software In The Loop) 和 MAVProxy 进行无人机仿真测试。

## 概述

本教程主要面向开发者，介绍如何使用 SITL 和 MAVProxy 测试新的 Copter 固件和修复 bug。内容包括起飞、GUIDED 模式飞行、执行任务、设置地理围栏等基本测试任务。

## 前提条件

在开始仿真前，请确保已经：

1. 在 Windows 或 Linux 系统上设置好 SITL 环境
2. 需要在master分支上
3. 使用以下命令启动 SITL（包含地图和控制台选项）：

```bash
cd ~/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py --map --console
```

**注意**：
- 如果想在不同地图位置启动仿真或使用不同的机架类型（hexa、tri、octa、heli 等），请参考 SITL 高级测试文档
- 可以使用 `sim_vehicle.py --help` 查看完整的启动选项

## 起飞步骤

### 1. 切换到 GUIDED 模式

在 MAVProxy 命令提示符中输入：
```
mode guided
```

### 2. 解锁油门

输入解锁命令：
```
arm throttle
```

### 3. 执行起飞

起飞到指定高度（例如 40 米）：
```
takeoff 40
```

**重要注意事项**：
- 起飞必须在解锁后 15 秒内执行，否则电机会重新上锁
- Copter 默认只在 GUIDED 模式下支持起飞
- 如果想在 AUTO 模式下起飞，需要设置 `AUTO_OPTIONS` 参数为 3

### 4. 监控起飞过程

- 在控制台的 `Alt` 字段观察高度变化
- 可以使用 `gtakeoff` 命令绘制起飞图表（需要先设置 MAVProxy 启动脚本并加载 graph 模块）

## 故障排除

### 常见起飞问题

1. **未在 GUIDED 模式下启动**：确保首先切换到 GUIDED 模式
2. **车辆未解锁**：检查是否成功执行了 `arm throttle`
3. **预解锁检查失败**：使用以下命令查看所有启用的检查：

```
arm list
```

典型的检查项目包括：
- params（参数）
- voltage（电压）
- compass（罗盘）
- battery（电池）
- ins（惯性导航系统）
- rc（遥控器）
- baro（气压计）
- gps（GPS）

可以使用 `arm check n` 和 `arm uncheck n` 分别启用和禁用检查，其中 `n` 是检查名称。使用 `all` 启用/禁用所有检查。

## 飞行模式切换

### 切换到 CIRCLE 模式

让无人机以指定半径绕圈飞行：
```
mode circle
param set circle_radius 2000
```

**注意**：如果将 `CIRCLE_RADIUS` 设置为 0，无人机将在原地旋转。

### 可用飞行模式

使用 `mode` 命令查看所有可用模式：
```
mode
```

输出示例：
```
('Available modes: ', ['RTL', 'POSHOLD', 'LAND', 'OF_LOITER', 'STABILIZE', 'AUTO', 'GUIDED', 'DRIFT', 'FLIP', 'AUTOTUNE', 'ALT_HOLD', 'LOITER', 'POSITION', 'CIRCLE', 'SPORT', 'ACRO'])
```

### 常用模式切换命令

- **立即降落**：`mode land`
- **返回起飞点并降落**：`rtl`
- **自动模式**：`mode auto`
- **稳定模式**：`mode stabilize`

## GUIDED 模式操作

### 1. 地图点击导航

在 GUIDED 模式下，最简单的方法是：
1. 在地图上右键点击目标位置
2. 选择 "Fly to"
3. 输入目标高度

### 2. 命令行导航

手动指定目标位置：
```
guided ALTITUDE              # 使用最后指定的经纬度，只改变高度
guided LAT LON ALTITUDE      # 指定完整的经纬度和高度
```

### 3. 其他控制命令

```
setyaw ANGLE ANGULAR_SPEED MODE   # 设置偏航角（MODE: 0-绝对, 1-相对）
setspeed SPEED_VALUE              # 设置速度
velocity x y z                    # 设置速度向量 (m/s)
```

## 执行任务

### 加载和运行任务

1. **加载任务文件**：
```
wp load ..\Tools\autotest\Generic_Missions\CMAC-circuit.txt
```

2. **切换到 AUTO 模式**：
```
mode auto
```

3. **跳转到指定航点**：
```
wp set 2
```

4. **循环执行任务**：
```
wp loop
```

### 航点管理命令

使用 `wp` 命令可以查看完整的航点操作命令，或使用自动补全功能查看可用选项。

## 高级功能

### 地理围栏设置

地理围栏可以限制无人机的飞行区域，确保安全操作。具体设置方法请参考相关文档。

### 自定义测试场景

可以通过修改任务文件或使用参数设置来创建自定义测试场景，验证特定功能或修复的问题。

## 注意事项

1. **仿真环境**：确保 SITL 环境正确配置，所有依赖项都已安装
2. **命令时机**：注意命令执行的时机，特别是解锁和起飞之间的时间间隔
3. **模式限制**：不同飞行模式有不同的功能限制，请参考官方文档
4. **参数设置**：某些功能需要特定的参数配置才能正常工作

## 扩展阅读

- [SITL 高级测试文档](https://ardupilot.org/dev/docs/sitl-advanced-testing.html)
- [MAVProxy 完整文档](https://ardupilot.org/mavproxy/)
- [ArduPilot 参数参考](https://ardupilot.org/copter/docs/parameters.html)

---

*本文档最后更新：2026年1月28日*
*基于 ArduPilot 官方文档翻译和整理*