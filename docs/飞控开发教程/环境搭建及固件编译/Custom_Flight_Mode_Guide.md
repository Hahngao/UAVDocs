# ArduCopter 自定义飞行模式添加指南

本指南基于 `newcode` 目录下的修改，详细说明如何在 ArduCopter 中添加一个自定义飞行模式（以 `AUTO_DRAW` 星形绘制模式为例）。

## 涉及文件

1.  `ArduCopter/mode.h`: 定义飞行模式枚举和类。
2.  `ArduCopter/Copter.h`: 在 Copter 主类中声明飞行模式对象。
3.  `ArduCopter/mode.cpp`: 注册飞行模式，使其可通过 MAVLink 或其他方式调用。
4.  `ArduCopter/mode_AutoDraw.cpp`: (新建) 实现自定义飞行模式的具体逻辑。

---

## 步骤 1: 定义飞行模式类 (`mode.h`)

在 `ArduCopter/mode.h` 文件中，需要完成两件事：添加模式枚举值和定义模式类。

### 1.1 添加枚举值

在 `Mode::Number` 枚举类中添加新的模式编号：

```cpp
// ArduCopter/mode.h

enum class Number : uint8_t {
    // ... 现有模式 ...
    TURTLE =       28,
    AUTO_DRAW =    29,  // 新增：Auto Draw mode
};
```

### 1.2 定义模式类

在文件末尾（或其他合适位置）定义继承自 `Mode` 的新类 `ModeAutoDraw`：

```cpp
// ArduCopter/mode.h

// 新增代码：自定义飞行模式类 ModeAutoDraw，继承自 Mode
class ModeAutoDraw : public Mode {

public:
    // 继承构造函数
    using Copter::Mode::Mode;

    // 初始化和主运行函数
    bool init(bool ignore_checks) override;
    void run() override;

    // 模式属性设置
    bool requires_GPS() const override { return true; }         // 需要 GPS
    bool has_manual_throttle() const override { return false; } // 自动油门
    bool allows_arming(bool from_gcs) const override { return false; } // 不允许在模式下解锁
    bool is_autopilot() const override { return true; }         // 属于自动模式
    bool in_guided_mode() const { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return false; }

protected:
    const char *name() const override { return "DRAWSTAR"; }
    const char *name4() const override { return "DRAWSTAR"; }

private:
    Vector3f path[10];      // 存储星形路线的航点
    int path_num;           // 当前航点索引

    void generate_path();   // 生成星形路径
    void pos_control_start(); // 启动位置控制
    void pos_control_run();   // 执行位置控制
};
```

---

## 步骤 2: 在 Copter 类中声明实例 (`Copter.h`)

在 `ArduCopter/Copter.h` 的 `Copter` 类定义中，添加新模式的实例对象。建议使用宏控制。

```cpp
// ArduCopter/Copter.h

class Copter {
    // ... 现有代码 ...

#if MODE_ACRO_ENABLED
    ModeAcro mode_acro;
#endif

    // ... 其他模式 ...

#if MODE_AUTO_DRAW_ENABLED
    ModeAutoDraw mode_autodraw;    // 新增：创建自动 draw 飞行模式对象
#endif

    // ... 现有代码 ...
};
```

> **注意**: `MODE_AUTO_DRAW_ENABLED` 宏需要在项目的配置头文件（如 `AP_Config.h`）中定义，或者直接在相关文件中定义。

---

## 步骤 3: 注册飞行模式 (`mode.cpp`)

在 `ArduCopter/mode.cpp` 中，修改 `mode_from_mode_num` 函数，将枚举值映射到对应的模式对象。

```cpp
// ArduCopter/mode.cpp

Mode *Copter::mode_from_mode_num(const Mode::Number mode)
{
    Mode *ret = nullptr;

    switch (mode) {
        // ... 现有 case ...

#if MODE_AUTO_DRAW_ENABLED
        case Mode::Number::AUTO_DRAW:   // 新增：映射 AUTO_DRAW 到 mode_autodraw 对象
            ret = &mode_autodraw;
            break;
#endif

        default:
            break;
    }

    return ret;
}
```

---

## 步骤 4: 实现飞行模式逻辑 (`mode_AutoDraw.cpp`)

创建一个新文件 `ArduCopter/mode_AutoDraw.cpp`，实现模式的初始化、路径生成和控制逻辑。

### 4.1 包含头文件

```cpp
#include "Copter.h"
```

### 4.2 初始化 (`init`)

初始化时检查位置状态，生成路径，并切换到位置控制。

```cpp
bool Copter::ModeAutoDraw::init(bool ignore_checks)
{
    if (copter.position_ok() || ignore_checks) {
        auto_yaw.set_mode_to_default(false);
        path_num = 0;
        generate_path();      // 生成路径
        pos_control_start();  // 启动控制
        return true;
    } else {
        return false;
    }
}
```

### 4.3 路径生成 (`generate_path`)

示例代码生成了一个星形路径。

```cpp
void Copter::ModeAutoDraw::generate_path()
{
    float radius_cm = g2.star_radius_cm; // 假设有一个参数控制半径
    wp_nav->get_wp_stopping_point(path[0]); // 以当前停止点为起点

    // 计算星形顶点 (示例逻辑)
    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    // ... 计算其他点 ...
    path[6] = path[1]; // 闭合路径
}
```

### 4.4 控制循环 (`run` 和 `pos_control_run`)

`run()` 函数由主循环以 100Hz 调用。

```cpp
void Copter::ModeAutoDraw::run()
{
    // 检查航点到达情况并切换下一个航点
    if(path_num < 6){
        if(wp_nav->reached_wp_destination()){
            path_num ++;
            wp_nav->set_wp_destination(path[path_num], false);
        }
    }

    pos_control_run(); // 执行位置控制
}

void Copter::ModeAutoDraw::pos_control_run()    
{
    // 安全检查
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // 允许飞行员控制偏航
    // ... (处理偏航输入逻辑)

    // 设置电机输出
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // 更新导航控制器
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // 更新高度控制器
    pos_control->update_z_controller();

    // 发送姿态控制指令
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
}
```

---

完成以上步骤后，重新编译 ArduCopter 固件即可使用新的 `AUTO_DRAW` 飞行模式。