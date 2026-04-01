# ArduCopter 自定义飞行模式添加指南

本文目标是给出一份**可对照官方源码直接落地**的步骤说明。

## 当前实现涉及文件

1. `ArduCopter/config.h`：开关宏 `MODE_AUTO_DRAW_ENABLED`
2. `ArduCopter/mode.h`：`Mode::Number::AUTO_DRAW` + `ModeAutoDraw` 类声明
3. `ArduCopter/Copter.h`：`Copter` 类成员 `mode_autodraw`
4. `ArduCopter/mode.cpp`：`mode_from_mode_num()` 中注册 `AUTO_DRAW`
5. `ArduCopter/mode_autodraw.cpp`：飞行模式逻辑实现
6. `ArduCopter/Parameters.h`：`ParametersG2::star_radius_cm` 字段声明

---

## 步骤 1：打开模式开关（`config.h`）


```cpp
#ifndef MODE_AUTO_DRAW_ENABLED
# define MODE_AUTO_DRAW_ENABLED 1
#endif
```

如果移植到其他分支，先确认该宏存在并生效。

---

## 步骤 2：在 `mode.h` 增加模式编号与类声明

### 2.1 枚举编号

```cpp
enum class Number : uint8_t {
    // ...
    TURTLE =       28,
    AUTO_DRAW=     29,
};
```

### 2.2 模式类声明

关键点：

- 基类构造继承写法是 `using Mode::Mode;`
- 使用 `requires_position()`（不是 `requires_GPS()`）
- `allows_arming` 签名是 `allows_arming(AP_Arming::Method method)`
- 当前实现里 `requires_position()` 返回 `false`

```cpp
class ModeAutoDraw : public Mode {
public:
    using Mode::Mode;
    Number mode_number() const override { return Number::AUTO_DRAW; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_position() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return false; }

protected:
    const char *name() const override { return "DRAWSTAR"; }
    const char *name4() const override { return "DRAWSTAR"; }

private:
    char _aircraft_name[32] = {0};
    Vector3f path[10];
    int path_num = 0;

    void generate_path();
    void pos_control_start();
    void pos_control_run();
};
```

---

## 步骤 3：在 `Copter.h` 声明模式对象

```cpp
#if MODE_AUTO_DRAW_ENABLED
    ModeAutoDraw mode_autodraw;
#endif
```

放在其他 `Mode*` 成员附近，保持现有文件结构。

---

## 步骤 4：在 `mode.cpp` 注册模式映射

在 `Copter::mode_from_mode_num()` 的 `switch` 中加入：

```cpp
#if MODE_AUTO_DRAW_ENABLED
    case Mode::Number::AUTO_DRAW:
        return &mode_autodraw;
#endif
```

当前实现直接 `return`，不使用中间 `ret` 变量。

---

## 步骤 5：实现模式逻辑（`mode_autodraw.cpp`）

### 5.1 `init()`

```cpp
bool ModeAutoDraw::init(bool ignore_checks)
{
    _aircraft_name[0] = '\0';
    path_num = 0;

    if (!ignore_checks && !copter.position_ok()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "ModeAutoDraw: GPS not ready, ignoring checks");
    }

    auto_yaw.set_mode_to_default(false);
    generate_path();

    const int name_fd = AP::FS().open("autodraw_name.txt", O_RDONLY, true);
    if (name_fd != -1) {
        char line[sizeof(_aircraft_name)];
        if (AP::FS().fgets(line, sizeof(line) - 1, name_fd)) {
            line[sizeof(line) - 1] = '\0';
            const size_t len = strcspn(line, "\r\n");
            line[len] = '\0';
            strncpy(_aircraft_name, line, sizeof(_aircraft_name) - 1);
            _aircraft_name[sizeof(_aircraft_name) - 1] = '\0';
        }
        AP::FS().close(name_fd);
    }

    pos_control_start();
    return true;
}
```

### 5.2 `generate_path()`

```cpp
void ModeAutoDraw::generate_path()
{
    float radius_cm = g2.star_radius_cm;  // 此处star_radius_cm变量需要在Parameters.h中声明
    if (radius_cm <= 0.0f) {
        radius_cm = 1000.0f;
    }

    Vector3f stopping_point_neu_cm;
    if (wp_nav) {
        wp_nav->get_wp_stopping_point_NEU_cm(stopping_point_neu_cm);
    } else {
        stopping_point_neu_cm.zero();
    }
    path[0] = stopping_point_neu_cm;

    path[1] = path[0] + Vector3f(1.0f, 0, 0) * radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)), -sinf(radians(36.0f)), 0) * radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)), cosf(radians(18.0f)), 0) * radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)), -cosf(radians(18.0f)), 0) * radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)), sinf(radians(36.0f)), 0) * radius_cm;
    path[6] = path[1];
}
```

### 5.3 `pos_control_start()`

```cpp
void ModeAutoDraw::pos_control_start()
{
    if (wp_nav) {
        wp_nav->wp_and_spline_init_m();
        wp_nav->set_wp_destination_NEU_cm(path[0], false);
    }
    auto_yaw.set_mode_to_default(false);
}
```

### 5.4 `run()`

```cpp
void ModeAutoDraw::run()
{
    static uint32_t last_gps_time_print_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_gps_time_print_ms >= 1000) {
        last_gps_time_print_ms = now_ms;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "ID: %s, GPS time: week %u ms %lu",
                        _aircraft_name,
                        copter.gps.time_week(),
                        (unsigned long)copter.gps.time_week_ms());
    }

    if (wp_nav && path_num < 6) {
        if (wp_nav->reached_wp_destination()) {
            path_num++;
            if (path_num <= 6) {
                wp_nav->set_wp_destination_NEU_cm(path[path_num], false);
            }
        }
    }

    pos_control_run();
}
```

### 5.5 `pos_control_run()`

```cpp
void ModeAutoDraw::pos_control_run()
{
    if (!motors || !motors->armed() || !copter.ap.auto_armed || !motors->get_interlock() || copter.ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    float target_yaw_rate_rads = 0.0f;
    if (!copter.failsafe.radio) {
        target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();
        if (!is_zero(target_yaw_rate_rads)) {
            auto_yaw.set_mode(AutoYaw::Mode::PILOT_RATE);
        }
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    if (wp_nav) {
        copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    }
    if (pos_control) {
        pos_control->D_update_controller();
    }

    if (attitude_control && wp_nav) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
            wp_nav->get_roll_rad(),
            wp_nav->get_pitch_rad(),
            target_yaw_rate_rads);
    }
}
```


---

## 编译验证


在仓库根目录执行：

```sh
./waf configure --board sitl
./waf copter -j4
```

先编译 SITL 的目的：快速确认飞行模式的代码接线（模式枚举、对象声明、模式注册与实现文件）是否正确，并在不依赖具体硬件外设与板级配置的前提下尽早发现编译错误，缩短调试迭代周期。

如果需要验证 `MicoAir743v2` 硬件目标(示例)，可执行：

```sh
./waf configure --board MicoAir743v2
./waf copter -j4
```

如果编译通过，说明代码层面的集成没有问题。后续可以在 SITL仿真中测试飞行模式逻辑。