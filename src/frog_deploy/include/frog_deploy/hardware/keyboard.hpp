#pragma once

#include <functional>
#include <mutex>
#include <termios.h>
#include <unistd.h>
#include <utility>

#include <frog_deploy/hardware/gamepad.hpp>
#include <frog_deploy/utils/thread_timer.hpp>

namespace Input
{

enum class Keyboard
{
    None = 0,
    A, B, C, D, E, F, G, H, I, J, K, L, M,
    N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Space, Enter, Escape,
    Up, Down, Left, Right
};

// 键盘 → 手柄按键。这是“键盘虚拟成手柄”的唯一入口，映射必须覆盖
// 状态机用到的每个手柄按键（见 fsm/robot/a1/a1_fsm.hpp），否则该键不会切状态。
// 注意：字母 W/S/A/D/Q/E 必须返回 None——它们只负责速度暂存。
inline Gamepad key_to_button(Keyboard key)
{
    switch (key)
    {
    case Keyboard::Num0:
    case Keyboard::P:
        return Gamepad::X; // Passive
    case Keyboard::Num1:
        return Gamepad::A; // GetUp
    case Keyboard::Num2:
    case Keyboard::Up:
        return Gamepad::DPadUp; // Locomotion
    case Keyboard::Num3:
    case Keyboard::Right:
        return Gamepad::DPadRight; // DreamWaQ
    case Keyboard::Num4:
    case Keyboard::Down:
        return Gamepad::DPadDown; // WalkTheseWay
    case Keyboard::Num5:
    case Keyboard::Left:
        return Gamepad::DPadLeft; // ExtremeParkour
    case Keyboard::Num6:
        return Gamepad::LStick; // AmpForHardware
    case Keyboard::B:
        return Gamepad::B; // Disabled
    case Keyboard::Num7:
        return Gamepad::LB; // 左摇杆命令模式
    case Keyboard::Num8:
        return Gamepad::RB; // 右摇杆命令模式
    case Keyboard::Num9:
        return Gamepad::Y; // 步态模式
    default:
        return Gamepad::None;
    }
}

} // namespace Input

// 终端键盘：把 stdin 切到 raw 模式并上报按键事件。
// 控制循环可用 get_keyboard() 轮询，也可注册回调处理后端相关动作（如仿真复位/暂停）。
class KeyboardHandler
{
public:
    using EventCallback = std::function<void(Input::Keyboard)>;

    ~KeyboardHandler()
    {
        stop();
    }

    void start()
    {
        if (raw_mode_)
            return;
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &original_terminal_) == -1)
            return;

        termios raw = original_terminal_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
            return;

        raw_mode_ = true;
        keyboard_timer_.start(0.01, [this]() { handle(); });
    }

    void stop()
    {
        keyboard_timer_.stop();
        if (!raw_mode_)
            return;

        tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_);
        raw_mode_ = false;
    }

    void set_event_callback(EventCallback callback)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        event_callback_ = std::move(callback);
    }

    // Returns the last key once, then None until a new key arrives.
    Input::Keyboard get_keyboard()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const Input::Keyboard key = current_;
        current_ = Input::Keyboard::None;
        return key;
    }

    // Polled by keyboard_timer_.
    void handle()
    {
        char character = '\0';
        if (read(STDIN_FILENO, &character, 1) != 1)
            return;

        if (character != 27)
        {
            dispatch(character);
            return;
        }

        // ESC sequence: arrows arrive as ESC [ A/B/C/D.
        char sequence[2]{};
        if (read(STDIN_FILENO, &sequence[0], 1) == 1 && sequence[0] == '[' &&
            read(STDIN_FILENO, &sequence[1], 1) == 1)
        {
            switch (sequence[1])
            {
            case 'A':
                dispatch(Input::Keyboard::Up);
                break;
            case 'B':
                dispatch(Input::Keyboard::Down);
                break;
            case 'C':
                dispatch(Input::Keyboard::Right);
                break;
            case 'D':
                dispatch(Input::Keyboard::Left);
                break;
            default:
                break;
            }
        }
        else
        {
            dispatch(Input::Keyboard::Escape);
        }
    }

private:
    void dispatch(char character)
    {
        switch (character)
        {
        case 'w':
            dispatch(Input::Keyboard::W);
            break;
        case 'a':
            dispatch(Input::Keyboard::A);
            break;
        case 's':
            dispatch(Input::Keyboard::S);
            break;
        case 'd':
            dispatch(Input::Keyboard::D);
            break;
        case 'q':
            dispatch(Input::Keyboard::Q);
            break;
        case 'e':
            dispatch(Input::Keyboard::E);
            break;
        case 'p':
            dispatch(Input::Keyboard::P);
            break;
        case 'b':
            dispatch(Input::Keyboard::B); // → 手柄 B（Disabled）
            break;
        case 'r':
            dispatch(Input::Keyboard::R);
            break;
        case ' ':
            dispatch(Input::Keyboard::Space);
            break;
        case '\n':
        case '\r':
            dispatch(Input::Keyboard::Enter);
            break;
        case '0':
            dispatch(Input::Keyboard::Num0);
            break;
        case '1':
            dispatch(Input::Keyboard::Num1);
            break;
        case '2':
            dispatch(Input::Keyboard::Num2);
            break;
        case '3':
            dispatch(Input::Keyboard::Num3);
            break;
        case '4':
            dispatch(Input::Keyboard::Num4);
            break;
        case '5':
            dispatch(Input::Keyboard::Num5);
            break;
        case '6':
            dispatch(Input::Keyboard::Num6);
            break;
        case '7':
            dispatch(Input::Keyboard::Num7);
            break;
        case '8':
            dispatch(Input::Keyboard::Num8);
            break;
        case '9':
            dispatch(Input::Keyboard::Num9);
            break;
        default:
            break;
        }
    }

    void dispatch(Input::Keyboard key)
    {
        EventCallback event_callback;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_ = key;
            event_callback = event_callback_;
        }

        if (event_callback != nullptr)
            event_callback(key);
    }

    termios original_terminal_{};
    bool raw_mode_{false};
    Input::Keyboard current_{Input::Keyboard::None};
    EventCallback event_callback_;
    std::mutex mutex_;
    ThreadTimer keyboard_timer_;
};
