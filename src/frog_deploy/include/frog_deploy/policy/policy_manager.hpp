#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <frog_deploy/config/config_loader.hpp>
#include <frog_deploy/config/config_types.hpp>
#include <frog_deploy/hardware/input_hardware.hpp>
#include <frog_deploy/hardware/robot_hardware.hpp>
#include <frog_deploy/policy/observation/command_profile.hpp>
#include <frog_deploy/policy/observation/obs_collection.hpp>
#include <frog_deploy/policy/runtime/policy_register.hpp>
#include <frog_deploy/tui/display.hpp>
#include <frog_deploy/utils/utils.hpp>
#include <frog_deploy/utils/math.hpp>
#include <frog_deploy/utils/thread_timer.hpp>
#include <frog_deploy/utils/triple_buffer.hpp>

class PolicyManager
{
public:
    using ObsFn = std::function<std::vector<float>()>;

    PolicyManager(RobotInterface &robot,
                  InputHardware &input,
                  SkillConfigLoader &config,
                  std::filesystem::path policy_root)
        : robot_(robot)
        , input_(input)
        , config_(config)
        , policy_root_(std::move(policy_root))
    {
    }

    // Start a skill: load its config, configure the policy and start inference.
    // Until the first inference output arrives the robot holds its current
    // joint position.
    bool start_skill(const std::string &skill)
    {
        stop_skill();

        const auto robot_name = config_.robot().robot_name;
        if (!config_.load(policy_root_, robot_name, skill))
        {
            tui::message("ERROR", "Failed to load skill " + skill);
            return false;
        }

        const auto &policy = config_.policy();
        const auto &robot = config_.robot();

        if (!configure(robot, policy) || !start())
        {
            tui::message("ERROR", "Failed to start skill " + skill);
            return false;
        }

        hold_cmd_ = RobotCommand{};
        hold_cmd_.initialize(static_cast<std::size_t>(robot.dof));
        const auto joints = robot_.get_joint_position();
        for (std::size_t i = 0; i < hold_cmd_.position.size(); ++i)
        {
            hold_cmd_.position[i] = joints[i];
            hold_cmd_.kp[i] = robot.fixed_kp[i];
            hold_cmd_.kd[i] = robot.fixed_kd[i];
        }

        skill_active_ = true;
        return true;
    }

    // Called every control tick while a skill state is active.
    void run_skill()
    {
        if (!skill_active_)
            return;

        if (!has_output())
        {
            robot_.set_joint_command(hold_cmd_);
            return;
        }

        robot_.set_joint_command(target_command());
    }

    void stop_skill()
    {
        skill_active_ = false;
        reset();
    }

    bool skill_active() const
    {
        return skill_active_;
    }

    // 注入运行时输入张量（如深度图），按名字被 models 的 inputs 引用。
    void set_sensor(const std::string &name, std::vector<float> values)
    {
        sensor_values_[name] = std::move(values);
    }

private:
    bool configure(const RobotConfig &robot,
                   const PolicyConfig &policy)
    {
        stop();
        ready_ = false;
        robot_cfg_ = robot;
        policy_ = policy;
        command_profile_.reset();
        // 只响应本次配置之后的按键，不重放启动以来的历史事件。
        input_.skip_pending(command_cursor_);

        obs_.configure(policy_);
        proprio_frame_.clear();
        proprio_history_.clear();
        register_builtin_obs();
        if (obs_.value().size() != static_cast<std::size_t>(std::max(policy_.input_dim, 0)))
        {
            std::cerr << "[PolicyManager] Configured observation size " << obs_.value().size()
                      << " != policy input_dim " << policy_.input_dim << '\n';
            return false;
        }

        models_.clear();
        action_output_.clear();
        std::vector<ModelSpec> specs = policy_.models;
        if (specs.empty() && !policy_.model_name.empty())
        {
            ModelSpec single;
            single.name = "policy";
            single.file = policy_.model_name;
            single.inputs = {"obs"};
            single.output = "action";
            single.is_action = true;
            specs.push_back(std::move(single));
        }
        for (const auto &spec : specs)
        {
            auto backend = PolicyRegister::instance().create(spec.file);
            const auto path = config_.skill_dir() / spec.file;
            if (!backend || !backend->load(spec, path))
            {
                std::cerr << "[PolicyManager] Failed to load model '" << spec.file << "'\n";
                models_.clear();
                return false;
            }
            if (spec.is_action || spec.output == "action")
                action_output_ = spec.output;
            models_.push_back({spec, std::move(backend)});
        }
        if (models_.empty() || action_output_.empty())
        {
            std::cerr << "[PolicyManager] No action model configured\n";
            models_.clear();
            return false;
        }

        for (auto &model : models_)
            model.backend->reset();

        current_action_.assign(static_cast<std::size_t>(std::max(policy_.dof, 0)), 0.0F);
        previous_action_.assign(current_action_.size(), 0.0F);
        action_buffer_.publish(current_action_);
        has_output_.store(false);
        observation_failed_.store(false);
        infer_failed_.store(false);

        ready_ = true;
        return true;
    }

    // Sampling, history advancement and inference share one policy timer.
    bool start()
    {
        if (!ready_)
            return false;
        const auto decimation = std::max(policy_.decimation, 1);
        return infer_timer_.start(robot_cfg_.control_dt * static_cast<double>(decimation),
                                  [this]() { on_infer_tick(); });
    }

    void stop()
    {
        infer_timer_.stop();
    }

    void reset()
    {
        stop();
        if (!ready_)
            return;
        std::fill(current_action_.begin(), current_action_.end(), 0.0F);
        action_buffer_.publish(current_action_);
        has_output_.store(false);
        command_profile_.reset();
        proprio_frame_.clear();
        proprio_history_.clear();
        for (auto &model : models_)
            model.backend->reset();
        obs_.reset();
    }

    bool has_output() const
    {
        return has_output_.load();
    }

    RobotCommand target_command() const
    {
        const auto action = action_buffer_.read();
        const auto count = static_cast<std::size_t>(std::max(policy_.dof, 0));
        RobotCommand command;
        command.initialize(count);
        for (std::size_t policy_index = 0; policy_index < count; ++policy_index)
        {
            const int mapped = policy_index < policy_.joint_mapping.size()
                                   ? policy_.joint_mapping[policy_index]
                                   : static_cast<int>(policy_index);
            if (mapped < 0 || static_cast<std::size_t>(mapped) >= count)
                continue;

            const auto hardware_index = static_cast<std::size_t>(mapped);
            const float action_value = policy_index < action.size() ? action[policy_index] : 0.0F;
            const float scale = value_at(policy_.action_scale, policy_index, 1.0F);
            command.position[hardware_index] =
                value_at(policy_.default_dof_pos, policy_index, 0.0F) + action_value * scale;
            command.kp[hardware_index] = value_at(policy_.kp, policy_index, 0.0F);
            command.kd[hardware_index] = value_at(policy_.kd, policy_index, 0.0F);
        }
        return command;
    }

    static float value_at(const std::vector<float> &values, std::size_t index, float fallback)
    {
        return index < values.size() ? values[index] : fallback;
    }

    // 去掉首尾空白。
    static std::string trim(const std::string &text)
    {
        const auto begin = text.find_first_not_of(" \t");
        if (begin == std::string::npos)
            return {};
        const auto end = text.find_last_not_of(" \t");
        return text.substr(begin, end - begin + 1);
    }

    // 按顶层逗号拆分 concat(...) 里的引用列表。
    static std::vector<std::string> split_refs(const std::string &refs)
    {
        std::vector<std::string> parts;
        std::string current;
        for (const char character : refs)
        {
            if (character == ',')
            {
                parts.push_back(trim(current));
                current.clear();
            }
            else
            {
                current.push_back(character);
            }
        }
        parts.push_back(trim(current));
        return parts;
    }

    // 解析模型输入引用："obs" / "obs[a:b]" / "concat(a, b, ...)" / 上游输出名 / 传感器名。
    // 切片为半开区间 [a, b)，支持负索引（从尾部计数）。
    // concat 把多路张量按顺序拼成一个：多数 TorchScript 的 forward 只接受一个 tensor，
    // 例如 dreamwaq 的 actor 要吃 [code(19), obs(45)] 拼接后的 64 维。
    bool resolve_tensor(const std::string &ref, std::vector<float> &out) const
    {
        constexpr const char *kConcat = "concat(";
        if (ref.rfind(kConcat, 0) == 0 && ref.back() == ')')
        {
            out.clear();
            for (const auto &part : split_refs(ref.substr(std::string(kConcat).size(),
                                                          ref.size() - std::string(kConcat).size() - 1)))
            {
                std::vector<float> piece;
                if (!resolve_tensor(part, piece))
                    return false;
                out.insert(out.end(), piece.begin(), piece.end());
            }
            return !out.empty();
        }

        const auto open = ref.find('[');
        if (open != std::string::npos && ref.back() == ']')
        {
            const auto name = ref.substr(0, open);
            const auto range = ref.substr(open + 1, ref.size() - open - 2);
            const auto colon = range.find(':');
            if (colon == std::string::npos)
                return false;
            const auto it = tensors_.find(name);
            if (it == tensors_.end())
                return false;
            try
            {
                const auto size = static_cast<long>(it->second.size());
                const auto begin = std::stol(range.substr(0, colon));
                const auto end = std::stol(range.substr(colon + 1));
                const auto low = begin < 0 ? size + begin : begin;
                const auto high = end < 0 ? size + end : end;
                if (low < 0 || high > size || low > high)
                    return false;
                out.assign(it->second.begin() + low, it->second.begin() + high);
                return true;
            }
            catch (const std::exception &)
            {
                return false;
            }
        }
        const auto it = tensors_.find(ref);
        if (it != tensors_.end())
        {
            out = it->second;
            return true;
        }
        // 回退到硬件后端提供的运行时输入（如 depth_image）。
        return robot_.read_external_input(ref, out);
    }

    // Refresh observation terms once per policy step, before advancing history.
    void sample_observation()
    {
        sample_.joint_pos = robot_.get_joint_position();
        sample_.joint_vel = robot_.get_joint_velocity();
        sample_.joint_torque = robot_.get_joint_torque();
        sample_.base_quat = robot_.get_base_quaternion();
        sample_.base_ang_vel = robot_.get_base_angular_velocity();
        sample_.base_lin_acc = robot_.get_base_linear_acceleration();
        sample_cmd_ = input_.get_user_command();
        // 手柄按键与摇杆 → 命令（含步态参数与时钟），每拍推进一次。
        // 命令构造持有自己的按键游标，与状态机的判定互不影响。
        command_profile_.update(input_.poll_button(command_cursor_),
                               input_.get_axis(),
                               sample_cmd_.x,
                               sample_cmd_.y,
                               sample_cmd_.yaw,
                               robot_cfg_.control_dt * std::max(policy_.decimation, 1));
        collect_obs();
    }

    // One history frame per control_dt * decimation, then publish the action
    // for the control thread, which continues to run every control_dt.
    void on_infer_tick()
    {
        sample_observation();
        obs_.push();
        const auto &observation = obs_.value();
        if (observation.size() != static_cast<std::size_t>(policy_.input_dim))
        {
            if (!observation_failed_.exchange(true))
                std::cerr << "[PolicyManager] Observation size " << observation.size()
                          << " != expected " << policy_.input_dim << '\n';
            return;
        }

        tensors_.clear();
        tensors_.emplace("obs", observation);
        // 注入的运行时输入（如深度图），按名字引用。
        for (const auto &entry : sensor_values_)
            tensors_[entry.first] = entry.second;

        std::vector<float> action;
        bool has_action = false;
        for (const auto &model : models_)
        {
            std::vector<std::vector<float>> inputs;
            inputs.reserve(model.spec.inputs.size());
            bool resolved = true;
            for (const auto &ref : model.spec.inputs)
            {
                std::vector<float> tensor;
                if (!resolve_tensor(ref, tensor))
                {
                    std::cerr << "[PolicyManager] Cannot resolve input '" << ref
                              << "' for model '" << model.spec.name << "'\n";
                    resolved = false;
                    break;
                }
                inputs.push_back(std::move(tensor));
            }

            std::vector<float> output;
            if (!resolved || !model.backend->infer(inputs, output))
            {
                if (!infer_failed_.exchange(true))
                    std::cerr << "[PolicyManager] Inference failed for model '"
                              << model.spec.name << "'; retaining previous action\n";
                return;
            }

            tensors_[model.spec.output] = std::move(output);
            if (model.spec.output == action_output_)
            {
                action = tensors_[model.spec.output];
                has_action = true;
            }
        }

        if (!has_action || action.size() != current_action_.size())
        {
            if (!infer_failed_.exchange(true))
                std::cerr << "[PolicyManager] Action output missing or wrong size\n";
            return;
        }

        for (std::size_t i = 0; i < action.size(); ++i)
        {
            if (!std::isfinite(action[i]))
                action[i] = 0.0F;
            action[i] = std::clamp(action[i],
                                   value_at(policy_.action_lower, i, -100.0F),
                                   value_at(policy_.action_upper, i, 100.0F));
        }

        infer_failed_.store(false);
        previous_action_ = current_action_;
        current_action_ = std::move(action);
        action_buffer_.publish(current_action_);
        has_output_.store(true);
    }

    void collect_obs()
    {
        proprio_frame_.clear();
        for (const auto &name : policy_.observations)
        {
            const auto it = obs_fns_.find(name);
            if (it == obs_fns_.end())
                continue;
            auto values = it->second();
            utils::clip(values, policy_.clip_obs);
            obs_.set(name, values);
            if (proprio_frame_.size() < kProprioDim)
            {
                const auto take = std::min(kProprioDim - proprio_frame_.size(), values.size());
                proprio_frame_.insert(proprio_frame_.end(),
                                      values.begin(),
                                      values.begin() + static_cast<std::ptrdiff_t>(take));
            }
        }
        push_proprio_history();
    }

    // 记录本拍的 proprio(53) 供后续 parkour history 使用。与训练一致：写入前屏蔽
    // [6:8]（yaw 项），首帧用当前值填满整个历史窗口。
    void push_proprio_history()
    {
        if (proprio_frame_.size() != kProprioDim)
            return;
        auto frame = proprio_frame_;
        frame[6] = 0.0F;
        frame[7] = 0.0F;
        if (proprio_history_.empty())
            proprio_history_.assign(kHistoryLen, frame);
        else
        {
            proprio_history_.erase(proprio_history_.begin());
            proprio_history_.push_back(std::move(frame));
        }
    }

    void register_builtin_obs()
    {
        auto add = [this](const char *name, ObsFn fn) {
            if (obs_fns_.count(name) == 0)
                obs_fns_[name] = std::move(fn);
        };

        add("ang_vel", [this]() {
            auto out = sample_.base_ang_vel;
            utils::apply_scale(out, policy_.ang_vel_scale);
            return out;
        });

        add("gravity_vec", [this]() {
            return math::gravity_from_quat(sample_.base_quat);
        });

        add("commands", [this]() {
            std::vector<float> out{sample_cmd_.x, sample_cmd_.y, sample_cmd_.yaw};
            utils::apply_scale(out, policy_.command_scale);
            return out;
        });

        add("dof_pos", [this]() {
            auto out = utils::gather(sample_.joint_pos, policy_.joint_mapping);
            auto defaults = policy_.default_dof_pos;
            utils::expand_to_num(defaults, out.size(), 0.0F);
            for (std::size_t i = 0; i < out.size(); ++i)
                out[i] -= defaults[i];
            for (const int index : policy_.wheel_joint_index)
            {
                if (index >= 0 && static_cast<std::size_t>(index) < out.size())
                    out[static_cast<std::size_t>(index)] = 0.0F;
            }
            utils::apply_scale(out, policy_.dof_pos_scale);
            return out;
        });

        add("dof_vel", [this]() {
            auto out = utils::gather(sample_.joint_vel, policy_.joint_mapping);
            utils::apply_scale(out, policy_.dof_vel_scale);
            return out;
        });

        add("actions", [this]() {
            return current_action_;
        });

        add("last_actions", [this]() {
            return previous_action_;
        });

        // 步态时钟：由 command_profile 按当前步频与步态参数生成。
        add("clock_inputs", [this]() {
            const auto &clock = command_profile_.clock_inputs();
            return std::vector<float>(clock.begin(), clock.end());
        });

        // Walk These Ways 命令向量（15 维）：速度来自手柄/键盘，
        // 步态与机身参数由手柄按键切模式后由摇杆给出。
        add("walk_these_ways/commands", [this]() {
            auto out = command_profile_.walk_these_ways();
            utils::apply_scale(out, policy_.command_scale);
            return out;
        });

        // ---- extreme_parkour 专有观测 ----
        add("extreme_parkour/roll_pitch", [this]() {
            const auto &q = sample_.base_quat;
            if (q.size() != 4)
                return std::vector<float>(2, 0.0F);
            const float w = q[0], x = q[1], y = q[2], z = q[3];
            const float roll = std::atan2(2.0F * (w * x + y * z), 1.0F - 2.0F * (x * x + y * y));
            const float pitch = std::asin(std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F));
            return std::vector<float>{roll, pitch};
        });

        // 航向误差项：部署无航向目标，暂置 0（接入航向目标后填充）。
        add("extreme_parkour/yaw_delta", []() { return std::vector<float>(3, 0.0F); });

        // gait_command = [0*cmd_x, 0*cmd_y, cmd_x]
        add("extreme_parkour/gait_command", [this]() {
            return std::vector<float>{0.0F, 0.0F, sample_cmd_.x};
        });

        // env_class = [is_class17, is_not_class17] -> 部署恒为 [0, 1]
        add("extreme_parkour/env_class", []() { return std::vector<float>{0.0F, 1.0F}; });

        // 足端接触(4)：运行时无接触力，暂置 0（需足端力传感器/接触估计）。
        add("extreme_parkour/feet_contact", []() { return std::vector<float>(4, 0.0F); });
        // 高度扫描(132)：视觉部署下由 depth_latent 替代，置 0 占位。
        add("extreme_parkour/scan", []() { return std::vector<float>(132, 0.0F); });
        // priv_explicit(9)：模型内部 estimator 会覆盖该区域，置 0。
        add("extreme_parkour/priv_explicit", []() { return std::vector<float>(9, 0.0F); });
        // priv_latent(29)：仅特权训练用，部署置 0。
        add("extreme_parkour/priv_latent", []() { return std::vector<float>(29, 0.0F); });
        // proprio 历史(10x53)：最近 kHistoryLen 帧 proprio(kProprioDim)，旧->新，
        // 供 actor 的 StateHistoryEncoder 计算 hist_latent（训练 hist_encoding=True）。
        add("extreme_parkour/history", [this]() {
            std::vector<float> out;
            out.reserve(kHistoryLen * kProprioDim);
            if (proprio_history_.empty())
            {
                auto frame = proprio_frame_;
                frame.resize(kProprioDim, 0.0F);
                frame[6] = 0.0F;
                frame[7] = 0.0F;
                for (std::size_t i = 0; i < kHistoryLen; ++i)
                    out.insert(out.end(), frame.begin(), frame.end());
                return out;
            }
            for (const auto &frame : proprio_history_)
                out.insert(out.end(), frame.begin(), frame.end());
            return out;
        });
    }

    RobotInterface &robot_;
    InputHardware &input_;
    SkillConfigLoader &config_;
    std::filesystem::path policy_root_;
    bool skill_active_{false};
    RobotCommand hold_cmd_;
    RobotConfig robot_cfg_;
    struct RunningModel
    {
        ModelSpec spec;
        std::unique_ptr<PolicyBackend> backend;
    };

    PolicyConfig policy_;
    ObsCollection obs_;
    std::vector<RunningModel> models_;
    std::string action_output_;
    std::unordered_map<std::string, std::vector<float>> tensors_;
    std::unordered_map<std::string, std::vector<float>> sensor_values_;
    std::unordered_map<std::string, ObsFn> obs_fns_;
    std::vector<float> current_action_;
    std::vector<float> previous_action_;
    CommandProfile command_profile_;
    Input::ButtonCursor command_cursor_;
    // extreme_parkour proprio 历史（旧->新）及其本拍帧缓存。
    static constexpr std::size_t kProprioDim = 53;
    static constexpr std::size_t kHistoryLen = 10;
    std::vector<float> proprio_frame_;
    std::vector<std::vector<float>> proprio_history_;
    TripleBuffer<std::vector<float>> action_buffer_;
    bool ready_{false};
    std::atomic<bool> infer_failed_{false};
    std::atomic<bool> observation_failed_{false};
    std::atomic<bool> has_output_{false};
    RobotState sample_{};
    UserCommand sample_cmd_{};
    ThreadTimer infer_timer_;
};
