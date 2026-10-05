#pragma once

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <frog_deploy/config/config_loader.hpp>
#include <frog_deploy/hardware/input_hardware.hpp>
#include <frog_deploy/hardware/robot_hardware.hpp>
#include <frog_deploy/policy/policy_manager.hpp>
#include <frog_deploy/tui/display.hpp>
#include <frog_deploy/types.hpp>

// 状态基类。命名与 rl_sar 对齐：Enter() / Run() / Exit() / CheckChange()，
// CheckChange() 返回下一个状态名（返回自身表示保持不变）。
class FsmBase
{
public:
    explicit FsmBase(std::string name)
        : state_name_(std::move(name))
    {
    }

    virtual ~FsmBase() = default;

    virtual void Enter() {}
    virtual void Run() = 0;
    virtual void Exit() {}
    virtual std::string CheckChange() { return state_name_; }

    const std::string &GetStateName() const
    {
        return state_name_;
    }

protected:
    std::string state_name_;
};

// 技能参数挂在 `config` 上并按状态重新加载，因此这里不缓存 RobotConfig。
struct FsmContext
{
    RobotInterface &robot;
    InputHardware &input;
    SkillConfigLoader &config;
    PolicyManager &policy;
};

// 状态基类（带输入）：每个状态持有自己的按键游标，
// 于是 CheckChange() 里可以直接写
//   if (current_gamepad == Input::Gamepad::X) ...
// 键盘已由硬件层映射成手柄按键，这里看不到键盘。
class FsmState : public FsmBase
{
public:
    FsmState(std::string name, FsmContext &ctx)
        : FsmBase(std::move(name))
        , ctx_(ctx)
    {
        const auto count = static_cast<std::size_t>(std::max(ctx_.config.robot().dof, 0));
        cmd_.initialize(count);
    }

protected:
    // 每次 CheckChange() 调用一次，刷新最近一次按下的手柄按键
    // （键盘已在硬件层映射成手柄，见 keyboard.hpp 的 key_to_button()）。
    void poll_input()
    {
        current_gamepad = ctx_.input.poll_button(cursor_);
    }

    // 进入状态时丢弃进入前的历史按键，只响应进入之后的按下。
    void skip_pending_buttons()
    {
        ctx_.input.skip_pending(cursor_);
    }

    FsmContext &ctx_;
    RobotCommand cmd_;
    Input::Gamepad current_gamepad{Input::Gamepad::None};

private:
    Input::ButtonCursor cursor_;
};
