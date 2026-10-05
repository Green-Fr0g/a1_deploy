#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <frog_deploy/config/config_types.hpp>
#include <frog_deploy/hardware/gamepad.hpp>
#include <frog_deploy/hardware/keyboard.hpp>
#include <frog_deploy/tui/display.hpp>
#include <frog_deploy/types.hpp>

// 手柄（摇杆）给出的机体速度设定值。
struct UserCommand
{
    float x{0.0F};   // forward
    float y{0.0F};   // lateral
    float yaw{0.0F}; // yaw rate
};

// 仅仿真需要的动作（复位/暂停），由输入设备转发。
class SimulationControl
{
public:
    virtual ~SimulationControl() = default;
    virtual void reset() = 0;
    virtual void toggle_pause() = 0;
};

// 输入硬件抽象。向上只暴露两样东西：
//   1. 速度指令（摇杆或键盘给出）与摇杆数值
//   2. 手柄按键（键盘已在 key_to_button 里映射成手柄，上层看不到键盘）
// 按键“是什么意思”不进这一层：状态机判 current_gamepad，观测构造判命令模式键。
class InputHardware
{
public:
    InputHardware(const RobotConfig &config, SimulationControl *simulation)
        : simulation_(simulation)
    {
        (void)config;
        keyboard_.set_event_callback([this](Input::Keyboard key) { feed_key(key); });
    }

    virtual ~InputHardware()
    {
        stop();
    }

    InputHardware(const InputHardware &) = delete;
    InputHardware &operator=(const InputHardware &) = delete;

    void start()
    {
        if (running_)
            return;
        running_ = true;
        keyboard_.start();
        start_transport();
    }

    void stop()
    {
        if (!running_)
            return;
        stop_transport();
        keyboard_.stop();
        running_ = false;
    }

    // 最新速度指令；可从任意线程调用。
    virtual UserCommand get_user_command() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return command_;
    }

    // 最新摇杆数值；可从任意线程调用。
    Input::Axis get_axis() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return axis_;
    }

    // 取出一条尚未消费的按键事件（没有则 None）。键盘已映射成手柄。
    // 每次只消费一条：同一拍里连按多个键不会互相覆盖，
    // 剩下的事件会在接下来的几拍依次被处理。
    Input::Gamepad poll_button(Input::ButtonCursor &cursor) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &event : events_)
        {
            if (event.sequence < cursor.next_sequence)
                continue;
            cursor.next_sequence = event.sequence + 1;
            return event.button;
        }
        return Input::Gamepad::None;
    }

    // 把游标对齐到队尾，丢弃尚未处理的历史事件。
    // 新状态进入、策略重新配置时调用，避免重放进入之前的按键。
    void skip_pending(Input::ButtonCursor &cursor) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cursor.next_sequence = next_sequence_;
    }

protected:
    // 传输层实现（ROS /joy、Unitree 遥控器）把原始数据喂进 feed_joy。
    virtual void start_transport() {}
    virtual void stop_transport() {}

    // 键盘按键入口（KeyboardHandler 回调，也供派生类/测试注入）。
    // 按键先映射成手柄按键（key_to_button）再走同一条路，
    // 因此手柄/键盘在上去看来是同一种输入；WASD/QE 仍作为速度暂存。
    void feed_key(Input::Keyboard key)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            apply_key(key);
            push_button(Input::key_to_button(key), true);
        }

        // 复位/暂停与是否虚拟成手柄按键无关，按原始按键处理。
        if (simulation_ == nullptr)
            return;
        if (key == Input::Keyboard::R)
        {
            tui::message("INFO", "R: reset simulation");
            simulation_->reset();
        }
        else if (key == Input::Keyboard::Enter)
        {
            tui::message("INFO", "Enter: pause / resume simulation");
            simulation_->toggle_pause();
        }
    }

    // 原生输入布局：
    //   buttons: A, B, X, Y, LB(4), RB(5), LStick(9), RStick(10),
    //            DPad 由 axes 6（左右）与 7（上下）给出
    //   axes:    L_X[0], L_Y[1], R_X[2], R_Y[3]
    void feed_joy(const std::vector<int> &buttons, const std::vector<float> &axes)
    {
        const Input::Gamepad button = decode_button(buttons, axes, dpad_deadzone_);
        const Input::Axis axis = decode_axis(axes);

        bool fresh_press = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            apply_axis(axis);
            fresh_press = push_button(button);
        }

        if (fresh_press && debug_input())
            dump_joy(buttons, axes, button);

        if (!fresh_press || simulation_ == nullptr)
            return;
        // 只做仿真动作，并把动作打印出来：机器人突然不动时能立刻看出是暂停了。
        if (button == Input::Gamepad::Select)
        {
            tui::message("INFO", "Select: reset simulation");
            simulation_->reset();
        }
        else if (button == Input::Gamepad::Start)
        {
            tui::message("INFO", "Start: pause / resume simulation");
            simulation_->toggle_pause();
        }
    }

    // 摇杆静默后清空速度；速度随后回落到键盘暂存值。
    void reset_joystick()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        joystick_active_ = false;
        staging_ = keyboard_staging_;
        publish_velocity();
    }

private:
    static bool button_value(const std::vector<int> &buttons, std::size_t index)
    {
        return index < buttons.size() && buttons[index] != 0;
    }

    // 排查手柄映射时设 FROG_DEBUG_INPUT=1：打印按下的原始按键索引与方向键轴值。
    // 对照 ros2 topic echo /joy 就能看出设备实际用的是哪几个索引。
    static bool debug_input()
    {
        static const bool enabled = std::getenv("FROG_DEBUG_INPUT") != nullptr;
        return enabled;
    }

    static void dump_joy(const std::vector<int> &buttons,
                         const std::vector<float> &axes,
                         Input::Gamepad decoded)
    {
        std::cerr << "[input] pressed=";
        for (std::size_t i = 0; i < buttons.size(); ++i)
            if (buttons[i] != 0)
                std::cerr << i << ' ';
        std::cerr << "| hat=(" << axis_value(axes, 6) << "," << axis_value(axes, 7)
                  << ") | decoded=" << static_cast<int>(decoded) << '\n';
    }

    static float axis_value(const std::vector<float> &axes, std::size_t index)
    {
        return index < axes.size() ? axes[index] : 0.0F;
    }

    // 肩键 + 方向键组合。
    static Input::Gamepad bumper_dpad(Input::Gamepad dpad, bool left)
    {
        switch (dpad)
        {
        case Input::Gamepad::DPadUp:
            return left ? Input::Gamepad::LB_DPadUp : Input::Gamepad::RB_DPadUp;
        case Input::Gamepad::DPadDown:
            return left ? Input::Gamepad::LB_DPadDown : Input::Gamepad::RB_DPadDown;
        case Input::Gamepad::DPadLeft:
            return left ? Input::Gamepad::LB_DPadLeft : Input::Gamepad::RB_DPadLeft;
        case Input::Gamepad::DPadRight:
            return left ? Input::Gamepad::LB_DPadRight : Input::Gamepad::RB_DPadRight;
        default:
            return Input::Gamepad::None;
        }
    }

    // 手柄原始状态 → 单个按键枚举。优先级（先命中的先返回）：
    //   双肩键 > 单肩键+其它键 > 面键/摇杆按下 > 方向键
    // 方向键来自 axis 6/7（有些手柄报成 button 11..14），且**必须带死区**：
    // 不然轴上的小偏置会被当成“一直按住”，把所有其它按键全遮掉，
    // 表现为状态机怎么按都不切。
    static Input::Gamepad decode_button(const std::vector<int> &buttons,
                                        const std::vector<float> &axes,
                                        float dpad_deadzone)
    {
        const bool a = button_value(buttons, 0);
        const bool b = button_value(buttons, 1);
        const bool x = button_value(buttons, 2);
        const bool y = button_value(buttons, 3);
        const bool lb = button_value(buttons, 4);
        const bool rb = button_value(buttons, 5);
        const bool lstick = button_value(buttons, 9);
        const bool rstick = button_value(buttons, 10);

        Input::Gamepad dpad = Input::Gamepad::None;
        if (button_value(buttons, 11))
            dpad = Input::Gamepad::DPadUp;
        else if (button_value(buttons, 12))
            dpad = Input::Gamepad::DPadDown;
        else if (button_value(buttons, 13))
            dpad = Input::Gamepad::DPadLeft;
        else if (button_value(buttons, 14))
            dpad = Input::Gamepad::DPadRight;

        if (axis_value(axes, 7) > dpad_deadzone)
            dpad = Input::Gamepad::DPadUp;
        else if (axis_value(axes, 7) < -dpad_deadzone)
            dpad = Input::Gamepad::DPadDown;
        else if (axis_value(axes, 6) < -dpad_deadzone)
            dpad = Input::Gamepad::DPadLeft;
        else if (axis_value(axes, 6) > dpad_deadzone)
            dpad = Input::Gamepad::DPadRight;

        if (lb && rb)
            return Input::Gamepad::LB_RB;
        if (lb)
        {
            if (a)
                return Input::Gamepad::LB_A;
            if (b)
                return Input::Gamepad::LB_B;
            if (x)
                return Input::Gamepad::LB_X;
            if (y)
                return Input::Gamepad::LB_Y;
            if (lstick)
                return Input::Gamepad::LB_LStick;
            if (rstick)
                return Input::Gamepad::LB_RStick;
            const auto combined = bumper_dpad(dpad, true);
            return combined != Input::Gamepad::None ? combined : Input::Gamepad::LB;
        }
        if (rb)
        {
            if (a)
                return Input::Gamepad::RB_A;
            if (b)
                return Input::Gamepad::RB_B;
            if (x)
                return Input::Gamepad::RB_X;
            if (y)
                return Input::Gamepad::RB_Y;
            if (lstick)
                return Input::Gamepad::RB_LStick;
            if (rstick)
                return Input::Gamepad::RB_RStick;
            const auto combined = bumper_dpad(dpad, false);
            return combined != Input::Gamepad::None ? combined : Input::Gamepad::RB;
        }
        if (a)
            return Input::Gamepad::A;
        if (b)
            return Input::Gamepad::B;
        if (x)
            return Input::Gamepad::X;
        if (y)
            return Input::Gamepad::Y;
        if (lstick)
            return Input::Gamepad::LStick;
        if (rstick)
            return Input::Gamepad::RStick;
        if (button_value(buttons, 6))
            return Input::Gamepad::Select;
        if (button_value(buttons, 7))
            return Input::Gamepad::Start;
        return dpad;
    }

    static Input::Axis decode_axis(const std::vector<float> &axes)
    {
        Input::Axis axis{};
        axis.L_X = axis_value(axes, 0);
        axis.L_Y = axis_value(axes, 1);
        axis.R_X = axis_value(axes, 2);
        axis.R_Y = axis_value(axes, 3);
        return axis;
    }

    // 记录一次手柄按键事件并返回是否是新的按下。手柄持续按住只触发一次，
    // 松开后（None 到达）允许再次触发同一个键；键盘没有松开事件，
    // 所以每次按键都算一次上升沿（always_edge）。
    // 调用者需持有 mutex_。
    bool push_button(Input::Gamepad button, bool always_edge = false)
    {
        if (button == Input::Gamepad::None)
        {
            last_gamepad_ = Input::Gamepad::None;
            return false;
        }
        if (button == last_gamepad_ && !always_edge)
            return false;

        last_gamepad_ = button;
        events_.push_back({next_sequence_++, button});
        while (events_.size() > kMaxEvents)
            events_.pop_front();
        return true;
    }

    float axis_to_cmd(float value, float max_abs) const
    {
        if (std::fabs(value) < deadzone_)
            return 0.0F;
        const float sign = value >= 0.0F ? 1.0F : -1.0F;
        const float mag = (std::fabs(value) - deadzone_) / (1.0F - deadzone_);
        return sign * std::clamp(mag, 0.0F, 1.0F) * max_abs;
    }

    // 摇杆映射：-L_Y -> x, -L_X -> y, -R_X -> yaw（符号由传输层负责翻转）。
    // 调用者需持有 mutex_。
    void apply_axis(const Input::Axis &axis)
    {
        axis_ = axis;
        const float jx = axis_to_cmd(-axis.L_Y, max_x_);
        const float jy = axis_to_cmd(-axis.L_X, max_y_);
        const float jyaw = axis_to_cmd(-axis.R_X, max_yaw_);

        joystick_active_ = std::fabs(jx) > 0.0F || std::fabs(jy) > 0.0F || std::fabs(jyaw) > 0.0F;
        if (joystick_active_)
        {
            staging_.x = jx;
            staging_.y = jy;
            staging_.yaw = jyaw;
        }
        else
        {
            staging_ = keyboard_staging_;
        }
        publish_velocity();
    }

    // 键盘：W/S -> x, A/D -> y, Q/E -> yaw, Space -> 清零。
    // 调用者需持有 mutex_。
    void apply_key(Input::Keyboard key)
    {
        switch (key)
        {
        case Input::Keyboard::W:
            keyboard_staging_.x = std::clamp(keyboard_staging_.x + keyboard_step_x_, -max_x_, max_x_);
            break;
        case Input::Keyboard::S:
            keyboard_staging_.x = std::clamp(keyboard_staging_.x - keyboard_step_x_, -max_x_, max_x_);
            break;
        case Input::Keyboard::A:
            keyboard_staging_.y = std::clamp(keyboard_staging_.y + keyboard_step_y_, -max_y_, max_y_);
            break;
        case Input::Keyboard::D:
            keyboard_staging_.y = std::clamp(keyboard_staging_.y - keyboard_step_y_, -max_y_, max_y_);
            break;
        case Input::Keyboard::Q:
            keyboard_staging_.yaw =
                std::clamp(keyboard_staging_.yaw + keyboard_step_yaw_, -max_yaw_, max_yaw_);
            break;
        case Input::Keyboard::E:
            keyboard_staging_.yaw =
                std::clamp(keyboard_staging_.yaw - keyboard_step_yaw_, -max_yaw_, max_yaw_);
            break;
        case Input::Keyboard::Space:
            keyboard_staging_ = {};
            staging_ = {};
            break;
        default:
            break;
        }
        if (!joystick_active_ && key != Input::Keyboard::Space)
            staging_ = keyboard_staging_;
        publish_velocity();
    }

    // 调用者需持有 mutex_。
    void publish_velocity()
    {
        command_.x = staging_.x;
        command_.y = staging_.y;
        command_.yaw = staging_.yaw;
    }

    struct ButtonEvent
    {
        std::uint64_t sequence;
        Input::Gamepad button;
    };

    static constexpr std::size_t kMaxEvents = 64;

    SimulationControl *simulation_;
    KeyboardHandler keyboard_;
    mutable std::mutex mutex_;
    UserCommand command_;
    Input::Axis axis_{};
    std::deque<ButtonEvent> events_;
    std::uint64_t next_sequence_{1};
    Input::Gamepad last_gamepad_{Input::Gamepad::None};
    VelocityCmd staging_{};
    VelocityCmd keyboard_staging_{};
    bool joystick_active_{false};
    bool running_{false};
    float max_x_{3.0F};
    float max_y_{2.0F};
    float max_yaw_{1.2F};
    float deadzone_{0.05F};
    // 方向键轴（hat）的死区：必须明显大于摇杆死区，避免轴偏置遮掉其它按键。
    float dpad_deadzone_{0.5F};
    float keyboard_step_x_{0.1F};
    float keyboard_step_y_{0.1F};
    float keyboard_step_yaw_{0.1F};
};

// 输入后端的按名工厂；工厂拿到仿真控制钩子（真机为 nullptr），
// 这样仿真复位/暂停仍由输入设备驱动。
class InputRegister
{
public:
    using Factory = std::function<std::unique_ptr<InputHardware>(const RobotConfig &,
                                                                 SimulationControl *)>;

    static InputRegister &instance()
    {
        static InputRegister registry;
        return registry;
    }

    void add(const std::string &backend, Factory factory)
    {
        factories_[backend] = std::move(factory);
    }

    std::unique_ptr<InputHardware> create(const std::string &backend,
                                          const RobotConfig &config,
                                          SimulationControl *simulation) const
    {
        const auto it = factories_.find(backend);
        return it == factories_.end() ? nullptr : it->second(config, simulation);
    }

private:
    InputRegister() = default;

    std::unordered_map<std::string, Factory> factories_;
};

// 输入后端自注册。
#define FROG_REGISTER_INPUT(BACKEND, TYPE)                                                         \
    namespace                                                                                      \
    {                                                                                              \
    const bool FROG_CONCAT(frog_input_reg_, __COUNTER__) = []() {                                  \
        InputRegister::instance().add(BACKEND, [](const RobotConfig &config,                       \
                                                  SimulationControl *simulation) {                 \
            return std::unique_ptr<InputHardware>(new TYPE(config, simulation));                   \
        });                                                                                        \
        return true;                                                                               \
    }();                                                                                           \
    }
