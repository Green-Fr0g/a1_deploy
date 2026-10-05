#pragma once

#include <cstdint>

// 手柄抽象（命名与 rl_sar 对齐）。上层（状态机、观测构造）只认这里的手柄按键：
// 真机遥控器、仿真 /joy、终端键盘都由 InputHardware 先变成手柄按键，
// 键盘的映射见 keyboard.hpp 的 key_to_button()。
namespace Input
{

enum class Gamepad
{
    None = 0,
    A, B, X, Y,
    LB, RB, LStick, RStick,
    // Back/Select 与 Start/Menu：仿真复位 / 暂停（不与肩键模式键冲突）
    Select, Start,
    DPadUp, DPadDown, DPadLeft, DPadRight,
    LB_A, LB_B, LB_X, LB_Y,
    LB_LStick, LB_RStick,
    LB_DPadUp, LB_DPadDown, LB_DPadLeft, LB_DPadRight,
    RB_A, RB_B, RB_X, RB_Y,
    RB_LStick, RB_RStick,
    RB_DPadUp, RB_DPadDown, RB_DPadLeft, RB_DPadRight, LB_RB
};

// 摇杆数值（原始归一化值，-1..1）。
struct Axis
{
    float L_X{0.0F};
    float L_Y{0.0F};
    float R_X{0.0F};
    float R_Y{0.0F};
};

// 组合键族判断：按住 LB/RB 再按其他键会解成 LB_*/RB_* 枚举，
// “哪一侧肩键被按下”这类判断需要把它们归为一族。
inline bool is_left_bumper(Gamepad button)
{
    switch (button)
    {
    case Gamepad::LB:
    case Gamepad::LB_A:
    case Gamepad::LB_B:
    case Gamepad::LB_X:
    case Gamepad::LB_Y:
    case Gamepad::LB_LStick:
    case Gamepad::LB_RStick:
    case Gamepad::LB_DPadUp:
    case Gamepad::LB_DPadDown:
    case Gamepad::LB_DPadLeft:
    case Gamepad::LB_DPadRight:
    case Gamepad::LB_RB:
        return true;
    default:
        return false;
    }
}

inline bool is_right_bumper(Gamepad button)
{
    switch (button)
    {
    case Gamepad::RB:
    case Gamepad::RB_A:
    case Gamepad::RB_B:
    case Gamepad::RB_X:
    case Gamepad::RB_Y:
    case Gamepad::RB_LStick:
    case Gamepad::RB_RStick:
    case Gamepad::RB_DPadUp:
    case Gamepad::RB_DPadDown:
    case Gamepad::RB_DPadLeft:
    case Gamepad::RB_DPadRight:
    case Gamepad::LB_RB:
        return true;
    default:
        return false;
    }
}

// 输入事件的消费游标。状态机每个状态一个游标，命令构造再持一个，
// 互不影响地读取同一串事件。
struct ButtonCursor
{
    std::uint64_t next_sequence{1};
};

} // namespace Input
