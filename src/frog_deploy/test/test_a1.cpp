#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <frog_deploy/fsm/all.hpp>
#include <frog_deploy/hardware/input_hardware.hpp>
#include <frog_deploy/hardware/robot_hardware.hpp>

namespace
{
void require(bool ok, const char *message)
{
    if (!ok)
        throw std::runtime_error(message);
}

bool nearly(float a, float b, float tolerance = 1e-4F)
{
    return std::fabs(a - b) <= tolerance;
}

// 状态机每拍只消费一条按键事件，测试里循环几拍把已注入的按键消化完，
// 返回第一个与当前状态不同的目标（没变化则返回当前状态名）。
std::string drive_fsm(FsmBase &state, int ticks = 8)
{
    for (int i = 0; i < ticks; ++i)
    {
        const auto next = state.CheckChange();
        if (next != state.GetStateName())
            return next;
    }
    return state.GetStateName();
}

void check_history_collection()
{
    PolicyConfig cfg;
    cfg.observations = {"first", "second"};
    cfg.observation_dims = {1, 1};
    cfg.observation_history = {3, 1, 0};
    for (const auto *order : {"time", "term"})
    {
        cfg.observation_history_priority = order;
        ObsCollection history;
        history.configure(cfg);
        require(history.value() == std::vector<float>(6, 0.0F), "Wrong initial history size");
        for (int step = 1; step <= 4; ++step)
        {
            history.set("first", {static_cast<float>(step)});
            history.set("second", {static_cast<float>(step + 10)});
            history.push();
        }
        const auto expected = cfg.observation_history_priority == "time"
                                  ? std::vector<float>{1, 11, 3, 13, 4, 14}
                                  : std::vector<float>{1, 3, 4, 11, 13, 14};
        require(history.value() == expected, "History indices or flatten order changed");
        history.reset();
        require(history.value() == std::vector<float>(6, 0.0F), "Reset retained old history");
        history.set("first", {7.0F});
        history.push();
        const auto seeded = cfg.observation_history_priority == "time"
                                ? std::vector<float>{7, 0, 7, 0, 7, 0}
                                : std::vector<float>{7, 7, 7, 0, 0, 0};
        require(history.value() == seeded, "First frame after reset was not seeded correctly");
    }
    cfg.observation_history = {0};
    ObsCollection single;
    single.configure(cfg);
    single.set("first", {1.0F});
    single.push();
    single.set("first", {2.0F});
    single.push();
    require(single.value() == std::vector<float>{2, 0}, "Single-frame observation is not current");
}

class TestRobot : public RobotInterface
{
public:
    explicit TestRobot(const RobotConfig &config) : RobotInterface(config)
    {
        state_staging_.joint_pos = config.default_dof_pos;
        state_staging_.joint_vel.assign(config.default_dof_pos.size(), 0.0F);
        publish_state(state_staging_);
    }
    RobotCommand last;

protected:
    void start_backend() override {}
    void stop_backend() override {}
    void send_command_impl(const RobotCommand &cmd) override { last = cmd; }
};

// 键盘即可驱动速度；同时用于确认速度指令进入了观测。
class TestInput : public InputHardware
{
public:
    using InputHardware::InputHardware;
    using InputHardware::feed_joy;
    using InputHardware::feed_key;
    UserCommand get_user_command() const override
    {
        auto command = InputHardware::get_user_command();
        command.x = static_cast<float>(++samples);
        return command;
    }
    mutable std::size_t samples{0}; // read by the test only after inference stops
};

struct ModelCall
{
    std::string name;
    std::vector<std::vector<float>> inputs;
};

struct Capture
{
    mutable std::mutex mutex;
    std::condition_variable cv;
    std::vector<ModelCall> calls;

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return calls.size();
    }

    // 最后一次指定模型的调用（拷贝一份，避免与推理线程竞争）。
    ModelCall last_call(const std::string &name) const
    {
        std::lock_guard<std::mutex> lock(mutex);
        for (auto it = calls.rbegin(); it != calls.rend(); ++it)
            if (it->name == name)
                return *it;
        return {};
    }

    void wait_for(std::size_t count, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait_for(lock, timeout, [&]() { return calls.size() >= count; });
    }
};

// 用桩后端替换真实推理后端，用于验证各技能的输入解析与执行顺序。
class CaptureBackend : public PolicyBackend
{
public:
    CaptureBackend(Capture &capture, std::map<std::string, std::size_t> dims)
        : capture_(capture)
        , dims_(std::move(dims))
    {
    }

    bool load(const ModelSpec &spec, const std::filesystem::path &) override
    {
        spec_ = spec;
        return dims_.count(spec.name) != 0;
    }

    bool infer(const std::vector<std::vector<float>> &inputs, std::vector<float> &output) override
    {
        {
            std::lock_guard<std::mutex> lock(capture_.mutex);
            capture_.calls.push_back({spec_.name, inputs});
            output.assign(dims_.at(spec_.name), 0.1F);
        }
        capture_.cv.notify_one();
        return true;
    }

    std::size_t output_dim() const override
    {
        const auto it = dims_.find(spec_.name);
        return it == dims_.end() ? 0U : it->second;
    }

private:
    Capture &capture_;
    std::map<std::string, std::size_t> dims_;
    ModelSpec spec_;
};
} // namespace

int main()
{
    try
    {
        check_history_collection();
        const std::filesystem::path root(FROG_POLICY_ROOT);
        SkillConfigLoader config;
        require(config.load(root, "a1"), "A1 base config failed to load");
        require(config.robot().dof == 12, "Wrong A1 dof");
        require(config.robot().joint_names.size() == 12, "Wrong A1 joint names");

        require(config.load(root, "a1", "dreamwaq"), "dreamwaq config failed to load");
        const auto cfg = config.policy();
        require(cfg.input_dim == 225, "Wrong dreamwaq observation size");
        require(cfg.observations.size() == 6 &&
                    cfg.observation_dims == std::vector<int>({3, 3, 3, 12, 12, 12}),
                "Wrong dreamwaq observation layout");
        require(cfg.models.size() == 4, "dreamwaq must declare a 4-model pipeline");
        require(cfg.models.back().is_action, "The last dreamwaq model must produce the action");

        Capture capture;
        const std::map<std::string, std::size_t> dims{
            {"encoder", 64}, {"vel", 3}, {"latent", 16}, {"actor", 12}};
        require(PolicyRegister::instance().register_backend(".pt", [&capture, dims]() {
                    return std::make_unique<CaptureBackend>(capture, dims);
                }),
                "Failed to register the capture backend");

        Capture wtw_capture;
        const std::map<std::string, std::size_t> wtw_dims{{"adaptation", 2}, {"body", 12}};
        require(PolicyRegister::instance().register_backend(".jit", [&wtw_capture, wtw_dims]() {
                    return std::make_unique<CaptureBackend>(wtw_capture, wtw_dims);
                }),
                "Failed to register the WTW capture backend");

        TestRobot robot(config.robot());
        TestInput input(config.robot(), nullptr);
        PolicyManager policy(robot, input, config, root);
        FsmContext ctx{robot, input, config, policy};

        // --- A1 FSM：每个技能一个状态，各自写 CheckChange() ---
        a1::Factory factory;
        require(factory.type() == "a1", "Wrong FSM type");
        require(factory.initial_state() == "RLFSMStatePassive", "Wrong initial state");
        require(factory.create_state(ctx, "Passive") == nullptr, "Old state names must be gone");
        auto passive = factory.create_state(ctx, "RLFSMStatePassive");
        auto getup = factory.create_state(ctx, "RLFSMStateGetUp");
        auto dreamwaq = factory.create_state(ctx, "RLFSMStateDreamWaQ");
        auto wtw = factory.create_state(ctx, "RLFSMStateWalkTheseWay");
        require(passive != nullptr && getup != nullptr && dreamwaq != nullptr && wtw != nullptr,
                "A1 states are not registered");
        for (const auto &name : a1::state_names())
            require(factory.create_state(ctx, name) != nullptr, "State is not registered");

        // Passive 只施加阻尼，不施加位置增益
        passive->Enter();
        passive->Run();
        require(robot.last.kp == std::vector<float>(12, 0.0F), "Passive must not apply position gains");
        require(robot.last.kd == std::vector<float>(12, 3.0F), "Passive must apply joint damping");

        // 键盘映射：Num1 → 手柄 A → GetUp，P / Num0 → 手柄 X → Passive
        input.feed_key(Input::Keyboard::Num1);
        require(drive_fsm(*passive) == "RLFSMStateGetUp", "Num1 (gamepad A) must request GetUp");
        input.feed_key(Input::Keyboard::P);
        require(drive_fsm(*passive) == "RLFSMStatePassive", "P (gamepad X) must stay in Passive");

        // GetUp：站起未完成前忽略技能请求
        getup->Enter();
        input.feed_key(Input::Keyboard::Num3);
        require(drive_fsm(*getup) == "RLFSMStateGetUp", "Skills must be ignored while getting up");
        for (int i = 0; i < 400; ++i)
            getup->Run();
        input.feed_key(Input::Keyboard::Num3);
        require(drive_fsm(*getup) == "RLFSMStateDreamWaQ",
                "Num3 (gamepad DPadRight) must request DreamWaQ");
        getup->Exit();

        // --- DreamWaQ：4 模型管线的输入解析与执行顺序 ---
        dreamwaq->Enter();
        require(policy.skill_active(), "dreamwaq skill did not start");
        capture.wait_for(4, std::chrono::seconds(2));
        require(capture.size() >= 4, "Policy-rate inference did not run");
        {
            std::lock_guard<std::mutex> lock(capture.mutex);
            const std::vector<std::string> expected{"encoder", "vel", "latent", "actor"};
            for (std::size_t i = 0; i < expected.size(); ++i)
                require(capture.calls[i].name == expected[i], "Models ran out of order");
        }
        const auto encoder = capture.last_call("encoder");
        const auto actor = capture.last_call("actor");
        require(encoder.inputs.size() == 1 && encoder.inputs[0].size() == 225,
                "Encoder input must be the 5-frame 225-dim history");
        require(actor.inputs.size() == 1 && actor.inputs[0].size() == 64,
                "Actor input must be the concatenated [vel(3), latent(16), obs(180:225)]");

        policy.run_skill();
        require(robot.last.kp == cfg.kp && robot.last.kd == cfg.kd, "RL PD gains were not applied");

        input.feed_key(Input::Keyboard::Down);
        require(drive_fsm(*dreamwaq) == "RLFSMStateWalkTheseWay",
                "DPadDown must request WalkTheseWay");
        dreamwaq->Exit();
        require(!policy.skill_active(), "Skill exit did not stop inference");

        // --- WalkTheseWay：15 维命令与步态时钟 ---
        require(config.load(root, "a1", "walk_these_way"), "walk_these_way config failed to load");
        const auto wtw_cfg = config.policy();
        require(wtw_cfg.input_dim == 2100, "Wrong WTW observation size");
        require(wtw_cfg.command_scale.size() == 15, "WTW needs a 15-dim command scale");
        require(wtw_cfg.observations.size() == 7 && wtw_cfg.observation_dims[1] == 15,
                "WTW observations must include the 15-dim command");

        wtw->Enter();
        require(policy.skill_active(), "walk_these_way skill did not start");
        wtw_capture.wait_for(4, std::chrono::seconds(2));
        require(wtw_capture.size() >= 4, "WTW inference did not run");
        {
            std::lock_guard<std::mutex> lock(wtw_capture.mutex);
            require(wtw_capture.calls[0].name == "adaptation" &&
                        wtw_capture.calls[1].name == "body",
                    "WTW models ran out of order");
        }

        // 最新一帧（history 索引 0）在帧优先展平里位于最后：29*70。
        constexpr std::size_t kFrame = 70;
        constexpr std::size_t kFrames = 29;
        constexpr std::size_t kCommandOffset = kFrames * kFrame + 3;
        constexpr std::size_t kClockOffset = kFrames * kFrame + 66;

        const auto check_command = [&](const std::vector<float> &obs,
                                       float stance_width,
                                       float footswing,
                                       bool expect_trot) {
            require(obs.size() == 2100, "Wrong WTW flattened observation size");
            const auto command = [&](std::size_t index) { return obs[kCommandOffset + index]; };
            require(nearly(command(1), 0.0F), "vy must be 0 without a joystick");
            require(nearly(command(2), 0.0F), "yaw must be 0 without a joystick");
            require(nearly(command(3), 0.0F), "body_height must default to 0");
            require(nearly(command(4), 3.0F), "gait frequency must default to 3 Hz");
            require(nearly(command(5), expect_trot ? 0.5F : 0.0F), "wrong gait phase");
            require(nearly(command(6), 0.0F), "gait offset must default to 0");
            require(nearly(command(7), 0.0F), "gait bound must default to 0");
            require(nearly(command(8), 0.5F), "gait duration must default to 0.5");
            require(nearly(command(9), footswing * 0.15F), "wrong footswing height");
            require(nearly(command(10), 0.0F), "body pitch must default to 0");
            require(nearly(command(11), 0.0F), "body roll must default to 0");
            require(nearly(command(12), stance_width), "wrong stance width");
            require(nearly(command(13), 0.40F), "stance length must default to 0.40");
            require(nearly(command(14), 0.0F), "aux reward must be 0");
            require(command(0) > 0.0F, "forward velocity command is missing");

            // 步态时钟：trot(phase 0.5) 时 [−s, s, s, −s]；walk(phase 0) 时四足同相。
            const auto clock = [&](std::size_t index) { return obs[kClockOffset + index]; };
            for (std::size_t i = 0; i < 4; ++i)
                require(std::fabs(clock(i)) <= 1.0F, "clock input out of range");
            if (expect_trot)
            {
                require(nearly(clock(0), -clock(1), 1e-3F),
                        "trot must keep the diagonal pair in antiphase");
                require(nearly(clock(2), clock(1), 1e-3F), "trot must keep the diagonal pair in phase");
                require(nearly(clock(3), clock(0), 1e-3F), "trot must keep the diagonal pair in phase");
            }
            else
            {
                for (std::size_t i = 1; i < 4; ++i)
                    require(nearly(clock(i), clock(0), 1e-3F), "walk must drive all four feet in phase");
            }
        };

        // body 的输入是 concat(obs(2100), latent(2)) = 2102，观测取前 2100 维。
        const auto wtw_observation = [&]() {
            const auto call = wtw_capture.last_call("body");
            if (call.inputs.empty() || call.inputs.front().size() < 2100)
                return std::vector<float>{};
            return std::vector<float>(call.inputs.front().begin(),
                                     call.inputs.front().begin() + 2100);
        };
        require(wtw_capture.last_call("body").inputs.front().size() == 2102,
                "body input must be the concatenated [obs(2100), latent(2)]");

        check_command(wtw_observation(), 0.33F, 0.08F, true);

        // 命令模式键（每次按下一个，等一个策略拍让它生效）：
        // Num7(LB) 左摇杆模式 身高->站立宽度、Num8(RB) 右摇杆模式 步频->抬腿高度、
        // Num9(Y) 步态 trot->walk。
        const auto before = wtw_capture.size();
        input.feed_key(Input::Keyboard::Num7);
        wtw_capture.wait_for(before + 2, std::chrono::seconds(2));
        input.feed_key(Input::Keyboard::Num8);
        wtw_capture.wait_for(before + 4, std::chrono::seconds(2));
        input.feed_key(Input::Keyboard::Num9);
        wtw_capture.wait_for(before + 6, std::chrono::seconds(2));
        require(wtw_capture.size() > before, "Mode keys did not trigger new inference");
        check_command(wtw_observation(), 0.275F, 0.03F, false);

        input.feed_key(Input::Keyboard::P);
        require(drive_fsm(*wtw) == "RLFSMStatePassive", "P must request Passive");
        wtw->Exit();
        require(!policy.skill_active(), "Skill exit did not stop inference");

        std::cout << "A1 hardware, per-skill FSM transitions and command observations passed\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
