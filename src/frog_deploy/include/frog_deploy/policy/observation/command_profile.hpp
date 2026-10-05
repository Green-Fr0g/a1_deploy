#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include <frog_deploy/hardware/gamepad.hpp>

// 手柄 → 策略命令。按键约定与 walk-these-ways 的遥控器一致：
//   LB（键盘 7）   循环左摇杆模式：机身高度 → 站立宽度 → 侧向速度
//   RB（键盘 8）   循环右摇杆模式：步频 → 抬腿高度 → 机身俯仰
//   Y （键盘 9）   循环步态：trot → walk → pace → bound（决定 phase/offset/bound）
//   L_Y  前进速度、R_X  偏航速度（速度本身由输入层给出，这里只做取舍）
// 键盘没有 L_X/R_Y 轴，所以模式对应的连续量在纯键盘下保持标称值。
class CommandProfile
{
public:
    // walk-these-ways 的 15 维命令：与训练侧
    // [vx, vy, yaw, height, freq, phase, offset, bound, duration,
    //  footswing, pitch, roll, stance_width, stance_length, aux] 一一对应。
    static constexpr std::size_t kCommandSize = 15;

    CommandProfile()
    {
        reset();
    }

    void reset()
    {
        left_mode_ = 0;
        right_mode_ = 0;
        gait_mode_ = 0;
        gait_index_ = 0.0;
        wtw_.assign(kCommandSize, 0.0F);
        clock_ = {0.0F, 0.0F, 0.0F, 0.0F};
        refresh({}, 0.0F, 0.0F, 0.0F);
    }

    // 每个策略节拍调用一次：处理按键边沿、刷新命令、推进步态相位。
    // button 是本拍最近一次按下的手柄按键（None 表示没有新按键）。
    void update(Input::Gamepad button,
                const Input::Axis &axis,
                float velocity_x,
                float velocity_y,
                float velocity_yaw,
                double dt)
    {
        if (button != Input::Gamepad::None)
        {
            if (Input::is_left_bumper(button))
                left_mode_ = (left_mode_ + 1) % kLeftModeCount;
            if (Input::is_right_bumper(button))
                right_mode_ = (right_mode_ + 1) % kRightModeCount;
            if (button == Input::Gamepad::Y)
                gait_mode_ = (gait_mode_ + 1) % kGaitModeCount;
        }

        refresh(axis, velocity_x, velocity_y, velocity_yaw);
        advance_gait(dt);
    }

    // 未乘 commands_scale 的 walk-these-ways 命令。
    const std::vector<float> &walk_these_ways() const
    {
        return wtw_;
    }

    // 4 维步态时钟 sin(2π·foot_index)。
    const std::array<float, 4> &clock_inputs() const
    {
        return clock_;
    }

    int left_mode() const { return left_mode_; }
    int right_mode() const { return right_mode_; }
    int gait_mode() const { return gait_mode_; }

private:
    enum LeftMode
    {
        kBodyHeight = 0,
        kStanceWidth,
        kLateralVelocity,
        kLeftModeCount
    };

    enum RightMode
    {
        kStepFrequency = 0,
        kFootswingHeight,
        kBodyPitch,
        kRightModeCount
    };

    enum GaitMode
    {
        kTrot = 0,
        kWalk,
        kPace,
        kBound,
        kGaitModeCount
    };

    struct GaitParams
    {
        float phase;
        float offset;
        float bound;
        float duration;
    };

    static constexpr std::array<GaitParams, kGaitModeCount> kGaitParams{{
        {0.5F, 0.0F, 0.0F, 0.5F}, // trot
        {0.0F, 0.0F, 0.0F, 0.5F}, // walk
        {0.0F, 0.5F, 0.0F, 0.5F}, // pace
        {0.0F, 0.0F, 0.5F, 0.5F}, // bound
    }};

    void refresh(const Input::Axis &axis, float velocity_x, float velocity_y, float velocity_yaw)
    {
        const GaitParams &gait = kGaitParams[static_cast<std::size_t>(gait_mode_)];

        bool lateral_mode_driven = false;

        // 标称值（与训练侧部署脚本一致）。
        float height = 0.0F;
        float lateral_velocity = 0.0F;
        float stance_width = 0.33F;
        const float stance_length = 0.40F;
        float footswing = 0.08F;
        float pitch = 0.0F;
        const float roll = 0.0F;
        float frequency = 3.0F;

        switch (static_cast<LeftMode>(left_mode_))
        {
        case kBodyHeight:
            height = 0.3F * axis.L_X;
            break;
        case kStanceWidth:
            stance_width = 0.275F + 0.175F * axis.L_X;
            break;
        case kLateralVelocity:
            // 该模式下左摇杆 X 专用于侧向速度（与训练侧部署一致）。
            lateral_velocity = 0.6F * axis.L_X;
            lateral_mode_driven = true;
            break;
        default:
            break;
        }

        switch (static_cast<RightMode>(right_mode_))
        {
        case kStepFrequency:
            frequency = (1.0F + axis.R_Y) * 0.5F * (kMaxFrequency - kMinFrequency) + kMinFrequency;
            break;
        case kFootswingHeight:
            footswing = std::max(0.0F, axis.R_Y) * 0.32F + 0.03F;
            break;
        case kBodyPitch:
            pitch = -0.4F * axis.R_Y;
            break;
        default:
            break;
        }

        wtw_[0] = velocity_x;
        // 侧向速度：默认直接采用手柄/键盘给出的侧向指令；
        // 左摇杆切到 lat_vel 模式时改用左摇杆 X。
        wtw_[1] = lateral_mode_driven ? lateral_velocity : velocity_y;
        wtw_[2] = velocity_yaw;
        wtw_[3] = height;
        wtw_[4] = frequency;
        wtw_[5] = gait.phase;
        wtw_[6] = gait.offset;
        wtw_[7] = gait.bound;
        wtw_[8] = gait.duration;
        wtw_[9] = footswing;
        wtw_[10] = pitch;
        wtw_[11] = roll;
        wtw_[12] = stance_width;
        wtw_[13] = stance_length;
        wtw_[14] = 0.0F; // aux_reward
    }

    // 步态相位积分与四足时钟。foot_index 与训练侧一致：
    //   [g + phase + offset + bound, g + offset, g + bound, g + phase]
    void advance_gait(double dt)
    {
        constexpr double kPi = 3.14159265358979323846;
        const double frequency = static_cast<double>(wtw_[4]);
        gait_index_ = std::fmod(gait_index_ + dt * frequency, 1.0);
        if (gait_index_ < 0.0)
            gait_index_ += 1.0;

        const double foot_indices[4] = {
            gait_index_ + static_cast<double>(wtw_[5]) + static_cast<double>(wtw_[6]) +
                static_cast<double>(wtw_[7]),
            gait_index_ + static_cast<double>(wtw_[6]),
            gait_index_ + static_cast<double>(wtw_[7]),
            gait_index_ + static_cast<double>(wtw_[5]),
        };
        for (std::size_t i = 0; i < clock_.size(); ++i)
            clock_[i] = static_cast<float>(std::sin(2.0 * kPi * foot_indices[i]));
    }

    static constexpr float kMinFrequency = 2.0F;
    static constexpr float kMaxFrequency = 4.0F;

    std::vector<float> wtw_;
    std::array<float, 4> clock_{0.0F, 0.0F, 0.0F, 0.0F};
    double gait_index_{0.0};
    int left_mode_{kBodyHeight};
    int right_mode_{kStepFrequency};
    int gait_mode_{kTrot};
};
