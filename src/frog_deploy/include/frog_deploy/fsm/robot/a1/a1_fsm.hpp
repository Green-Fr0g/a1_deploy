#pragma once

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <frog_deploy/fsm/fsm_register.hpp>

// A1 状态机。只看手柄按键（键盘已在 keyboard.hpp 的 key_to_button() 里映射成手柄）：
//   X         → RLFSMStatePassive        （阻尼安全态；键盘 0 / P）
//   A         → RLFSMStateGetUp          （插值到默认站姿；键盘 1）
//   DPadUp    → RLFSMStateLocomotion     （policy/a1/locomotion；键盘 2 / ↑）
//   DPadRight → RLFSMStateDreamWaQ       （policy/a1/dreamwaq；键盘 3 / →）
//   DPadDown  → RLFSMStateWalkTheseWay   （policy/a1/walk_these_way；键盘 4 / ↓）
//   DPadLeft  → RLFSMStateExtremeParkour （policy/a1/extreme_parkour；键盘 5 / ←）
//   LStick    → RLFSMStateAmpForHardware （policy/a1/amp_for_hardware；键盘 6）
//   B         → RLFSMStateDisabled       （增益与力矩全零；键盘 B）
// 每个技能一个独立状态类，切换条件写在各自的 CheckChange() 里。
namespace a1
{

// 全部状态名（顺序即注册顺序，首项由 Factory::initial_state() 指定）。
inline const std::vector<std::string> &state_names()
{
    static const std::vector<std::string> names{
        "RLFSMStatePassive",
        "RLFSMStateGetUp",
        "RLFSMStateDisabled",
        "RLFSMStateLocomotion",
        "RLFSMStateDreamWaQ",
        "RLFSMStateWalkTheseWay",
        "RLFSMStateExtremeParkour",
        "RLFSMStateAmpForHardware",
    };
    return names;
}

// ---------------------------------------------------------------- Passive

// 阻尼模式：kp=0、kd=固定阻尼，关节可被外力自由拖动。
class RLFSMStatePassive : public FsmState
{
public:
    explicit RLFSMStatePassive(FsmContext &ctx)
        : FsmState("RLFSMStatePassive", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStatePassive");
        skip_pending_buttons();
    }

    void Run() override
    {
        const auto &robot = ctx_.config.robot();
        for (std::size_t i = 0; i < cmd_.position.size(); ++i)
        {
            cmd_.position[i] = 0.0F;
            cmd_.velocity[i] = 0.0F;
            cmd_.torque[i] = 0.0F;
            cmd_.kp[i] = 0.0F;
            cmd_.kd[i] = i < robot.fixed_kd.size() ? robot.fixed_kd[i] : 3.0F;
        }
        ctx_.robot.set_joint_command(cmd_);
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStatePassive");
    }

    std::string CheckChange() override
    {
        poll_input();
        if (current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateDisabled";
        }
        return state_name_;
    }
};

// ---------------------------------------------------------------- Disabled

// 卸载模式：所有增益与力矩为零，关节无任何控制力。
class RLFSMStateDisabled : public FsmState
{
public:
    explicit RLFSMStateDisabled(FsmContext &ctx)
        : FsmState("RLFSMStateDisabled", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStateDisabled");
        skip_pending_buttons();
    }

    void Run() override
    {
        const auto position = ctx_.robot.get_joint_position();
        for (std::size_t i = 0; i < cmd_.position.size(); ++i)
        {
            cmd_.position[i] = i < position.size() ? position[i] : 0.0F;
            cmd_.velocity[i] = 0.0F;
            cmd_.torque[i] = 0.0F;
            cmd_.kp[i] = 0.0F;
            cmd_.kd[i] = 0.0F;
        }
        ctx_.robot.set_joint_command(cmd_);
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStateDisabled");
    }

    std::string CheckChange() override
    {
        poll_input();
        if (current_gamepad == Input::Gamepad::X || current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStatePassive";
        }
        return state_name_;
    }
};

// ---------------------------------------------------------------- GetUp

// 站起：把关节从当前位置插值到 base.yaml 的 default_dof_pos。
// percent_getdown 到 1 之前不接受技能请求。
class RLFSMStateGetUp : public FsmState
{
public:
    explicit RLFSMStateGetUp(FsmContext &ctx)
        : FsmState("RLFSMStateGetUp", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStateGetUp");
        skip_pending_buttons();
        percent_getdown_ = 0.0F;
        start_ = ctx_.robot.get_joint_position();
        target_ = ctx_.config.robot().default_dof_pos;
    }

    void Run() override
    {
        const auto &robot = ctx_.config.robot();
        const double dt = robot.control_dt > 0.0 ? robot.control_dt : 0.005;
        percent_getdown_ = std::min(1.0F, percent_getdown_ + static_cast<float>(dt / kDuration));

        for (std::size_t i = 0; i < cmd_.position.size(); ++i)
        {
            const float from = i < start_.size() ? start_[i] : 0.0F;
            const float to = i < target_.size() ? target_[i] : 0.0F;
            cmd_.position[i] = from + (to - from) * percent_getdown_;
            cmd_.velocity[i] = 0.0F;
            cmd_.torque[i] = 0.0F;
            cmd_.kp[i] = i < robot.fixed_kp.size() ? robot.fixed_kp[i] : 0.0F;
            cmd_.kd[i] = i < robot.fixed_kd.size() ? robot.fixed_kd[i] : 0.0F;
        }
        ctx_.robot.set_joint_command(cmd_);
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStateGetUp");
    }

    std::string CheckChange() override
    {
        poll_input();
        if (current_gamepad == Input::Gamepad::X)
        {
            return "RLFSMStatePassive";
        }
        else if (percent_getdown_ < 1.0F)
        {
            return state_name_; // 站起未完成，忽略技能请求
        }
        else if (current_gamepad == Input::Gamepad::DPadUp)
        {
            return "RLFSMStateLocomotion";
        }
        else if (current_gamepad == Input::Gamepad::DPadRight)
        {
            return "RLFSMStateDreamWaQ";
        }
        else if (current_gamepad == Input::Gamepad::DPadDown)
        {
            return "RLFSMStateWalkTheseWay";
        }
        else if (current_gamepad == Input::Gamepad::DPadLeft)
        {
            return "RLFSMStateExtremeParkour";
        }
        else if (current_gamepad == Input::Gamepad::LStick)
        {
            return "RLFSMStateAmpForHardware";
        }
        return state_name_;
    }

private:
    static constexpr float kDuration = 1.5F;
    float percent_getdown_{0.0F};
    std::vector<float> start_;
    std::vector<float> target_;
};

// ---------------------------------------------------------------- Locomotion

class RLFSMStateLocomotion : public FsmState
{
public:
    explicit RLFSMStateLocomotion(FsmContext &ctx)
        : FsmState("RLFSMStateLocomotion", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStateLocomotion");
        skip_pending_buttons();
        started_ = ctx_.policy.start_skill("locomotion");
        if (!started_)
            tui::message("ERROR", "Failed to start skill locomotion");
    }

    void Run() override
    {
        ctx_.policy.run_skill();
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStateLocomotion");
        ctx_.policy.stop_skill();
    }

    std::string CheckChange() override
    {
        poll_input();
        if (!started_ || !ctx_.policy.skill_active())
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::X)
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (current_gamepad == Input::Gamepad::DPadRight)
        {
            return "RLFSMStateDreamWaQ";
        }
        else if (current_gamepad == Input::Gamepad::DPadDown)
        {
            return "RLFSMStateWalkTheseWay";
        }
        else if (current_gamepad == Input::Gamepad::DPadLeft)
        {
            return "RLFSMStateExtremeParkour";
        }
        else if (current_gamepad == Input::Gamepad::LStick)
        {
            return "RLFSMStateAmpForHardware";
        }
        return state_name_;
    }

private:
    bool started_{false};
};

// ---------------------------------------------------------------- DreamWaQ

class RLFSMStateDreamWaQ : public FsmState
{
public:
    explicit RLFSMStateDreamWaQ(FsmContext &ctx)
        : FsmState("RLFSMStateDreamWaQ", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStateDreamWaQ");
        skip_pending_buttons();
        started_ = ctx_.policy.start_skill("dreamwaq");
        if (!started_)
            tui::message("ERROR", "Failed to start skill dreamwaq");
    }

    void Run() override
    {
        ctx_.policy.run_skill();
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStateDreamWaQ");
        ctx_.policy.stop_skill();
    }

    std::string CheckChange() override
    {
        poll_input();
        if (!started_ || !ctx_.policy.skill_active())
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::X)
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (current_gamepad == Input::Gamepad::DPadUp)
        {
            return "RLFSMStateLocomotion";
        }
        else if (current_gamepad == Input::Gamepad::DPadDown)
        {
            return "RLFSMStateWalkTheseWay";
        }
        else if (current_gamepad == Input::Gamepad::DPadLeft)
        {
            return "RLFSMStateExtremeParkour";
        }
        else if (current_gamepad == Input::Gamepad::LStick)
        {
            return "RLFSMStateAmpForHardware";
        }
        return state_name_;
    }

private:
    bool started_{false};
};

// ---------------------------------------------------------------- WalkTheseWay

class RLFSMStateWalkTheseWay : public FsmState
{
public:
    explicit RLFSMStateWalkTheseWay(FsmContext &ctx)
        : FsmState("RLFSMStateWalkTheseWay", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStateWalkTheseWay");
        skip_pending_buttons();
        started_ = ctx_.policy.start_skill("walk_these_way");
        if (!started_)
            tui::message("ERROR", "Failed to start skill walk_these_way");
    }

    void Run() override
    {
        ctx_.policy.run_skill();
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStateWalkTheseWay");
        ctx_.policy.stop_skill();
    }

    std::string CheckChange() override
    {
        poll_input();
        if (!started_ || !ctx_.policy.skill_active())
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::X)
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (current_gamepad == Input::Gamepad::DPadUp)
        {
            return "RLFSMStateLocomotion";
        }
        else if (current_gamepad == Input::Gamepad::DPadRight)
        {
            return "RLFSMStateDreamWaQ";
        }
        else if (current_gamepad == Input::Gamepad::DPadLeft)
        {
            return "RLFSMStateExtremeParkour";
        }
        else if (current_gamepad == Input::Gamepad::LStick)
        {
            return "RLFSMStateAmpForHardware";
        }
        return state_name_;
    }

private:
    bool started_{false};
};

// ---------------------------------------------------------------- ExtremeParkour

class RLFSMStateExtremeParkour : public FsmState
{
public:
    explicit RLFSMStateExtremeParkour(FsmContext &ctx)
        : FsmState("RLFSMStateExtremeParkour", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStateExtremeParkour");
        skip_pending_buttons();
        started_ = ctx_.policy.start_skill("extreme_parkour");
        if (!started_)
            tui::message("ERROR", "Failed to start skill extreme_parkour");
    }

    void Run() override
    {
        ctx_.policy.run_skill();
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStateExtremeParkour");
        ctx_.policy.stop_skill();
    }

    std::string CheckChange() override
    {
        poll_input();
        if (!started_ || !ctx_.policy.skill_active())
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::X)
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (current_gamepad == Input::Gamepad::DPadUp)
        {
            return "RLFSMStateLocomotion";
        }
        else if (current_gamepad == Input::Gamepad::DPadRight)
        {
            return "RLFSMStateDreamWaQ";
        }
        else if (current_gamepad == Input::Gamepad::DPadDown)
        {
            return "RLFSMStateWalkTheseWay";
        }
        else if (current_gamepad == Input::Gamepad::LStick)
        {
            return "RLFSMStateAmpForHardware";
        }
        return state_name_;
    }

private:
    bool started_{false};
};

// ---------------------------------------------------------------- AmpForHardware

class RLFSMStateAmpForHardware : public FsmState
{
public:
    explicit RLFSMStateAmpForHardware(FsmContext &ctx)
        : FsmState("RLFSMStateAmpForHardware", ctx)
    {
    }

    void Enter() override
    {
        tui::message("INFO", "Enter RLFSMStateAmpForHardware");
        skip_pending_buttons();
        started_ = ctx_.policy.start_skill("amp_for_hardware");
        if (!started_)
            tui::message("ERROR", "Failed to start skill amp_for_hardware");
    }

    void Run() override
    {
        ctx_.policy.run_skill();
    }

    void Exit() override
    {
        tui::message("INFO", "Exit RLFSMStateAmpForHardware");
        ctx_.policy.stop_skill();
    }

    std::string CheckChange() override
    {
        poll_input();
        if (!started_ || !ctx_.policy.skill_active())
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::X)
        {
            return "RLFSMStatePassive";
        }
        else if (current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (current_gamepad == Input::Gamepad::DPadUp)
        {
            return "RLFSMStateLocomotion";
        }
        else if (current_gamepad == Input::Gamepad::DPadRight)
        {
            return "RLFSMStateDreamWaQ";
        }
        else if (current_gamepad == Input::Gamepad::DPadDown)
        {
            return "RLFSMStateWalkTheseWay";
        }
        else if (current_gamepad == Input::Gamepad::DPadLeft)
        {
            return "RLFSMStateExtremeParkour";
        }
        return state_name_;
    }

private:
    bool started_{false};
};

// ---------------------------------------------------------------- Factory

class Factory : public FsmFactory
{
public:
    std::shared_ptr<FsmBase> create_state(FsmContext &ctx, const std::string &name) override
    {
        if (name == "RLFSMStatePassive")
            return std::make_shared<RLFSMStatePassive>(ctx);
        if (name == "RLFSMStateGetUp")
            return std::make_shared<RLFSMStateGetUp>(ctx);
        if (name == "RLFSMStateDisabled")
            return std::make_shared<RLFSMStateDisabled>(ctx);
        if (name == "RLFSMStateLocomotion")
            return std::make_shared<RLFSMStateLocomotion>(ctx);
        if (name == "RLFSMStateDreamWaQ")
            return std::make_shared<RLFSMStateDreamWaQ>(ctx);
        if (name == "RLFSMStateWalkTheseWay")
            return std::make_shared<RLFSMStateWalkTheseWay>(ctx);
        if (name == "RLFSMStateExtremeParkour")
            return std::make_shared<RLFSMStateExtremeParkour>(ctx);
        if (name == "RLFSMStateAmpForHardware")
            return std::make_shared<RLFSMStateAmpForHardware>(ctx);
        return nullptr;
    }

    std::string type() const override
    {
        return "a1";
    }

    std::vector<std::string> supported_states() const override
    {
        return state_names();
    }

    std::string initial_state() const override
    {
        return "RLFSMStatePassive";
    }
};

} // namespace a1

FROG_REGISTER_FSM(a1::Factory)
