#pragma once

// 真机后端依赖 thirdparty/unitree_legged_sdk（其预编译库内部使用 lcm）。
// CMake 未找到 SDK 或 lcm 时以 FROG_DEPLOY_HAS_UNITREE=0 编译，
// 本文件退化为空，仿真后端不受影响。
#ifndef FROG_DEPLOY_HAS_UNITREE
#define FROG_DEPLOY_HAS_UNITREE 0
#endif

#if FROG_DEPLOY_HAS_UNITREE

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <unitree_legged_sdk/comm.h>
#include <unitree_legged_sdk/quadruped.h>
#include <unitree_legged_sdk/safety.h>
#include <unitree_legged_sdk/udp.h>

#include <frog_deploy/hardware/input_hardware.hpp>
#include <frog_deploy/hardware/robot_hardware.hpp>

namespace unitree_hw
{

// G1/A1 wireless remote layout (40 bytes, same as the SDK example gamepad).
#pragma pack(push, 1)
struct WirelessRemote
{
    uint8_t head[2]{};
    union
    {
        uint16_t value;
        struct
        {
            uint8_t R1 : 1;
            uint8_t L1 : 1;
            uint8_t start : 1;
            uint8_t select : 1;
            uint8_t R2 : 1;
            uint8_t L2 : 1;
            uint8_t F1 : 1;
            uint8_t F2 : 1;
            uint8_t A : 1;
            uint8_t B : 1;
            uint8_t X : 1;
            uint8_t Y : 1;
            uint8_t up : 1;
            uint8_t right : 1;
            uint8_t down : 1;
            uint8_t left : 1;
        } components;
    } btn{};
    float lx{0.0F};
    float rx{0.0F};
    float ry{0.0F};
    float L2{0.0F};
    float ly{0.0F};
    uint8_t idle[16]{};
};
#pragma pack(pop)

static_assert(sizeof(WirelessRemote) == 40, "WirelessRemote must be 40 bytes");

// Translate the remote into the InputHardware button/axis layout.
inline void parse_wireless_remote(const uint8_t *raw,
                                  std::vector<int> &buttons,
                                  std::vector<float> &axes)
{
    WirelessRemote remote{};
    std::memcpy(&remote, raw, sizeof(remote));

    buttons.assign(11, 0);
    buttons[0] = remote.btn.components.A ? 1 : 0;
    buttons[1] = remote.btn.components.B ? 1 : 0;
    buttons[2] = remote.btn.components.X ? 1 : 0;
    buttons[3] = remote.btn.components.Y ? 1 : 0;
    buttons[4] = remote.btn.components.L1 ? 1 : 0; // LB
    buttons[5] = remote.btn.components.R1 ? 1 : 0; // RB
    buttons[6] = remote.btn.components.select ? 1 : 0; // Select：复位
    buttons[7] = remote.btn.components.start ? 1 : 0;  // Start：暂停
    buttons[9] = remote.btn.components.F1 ? 1 : 0; // LStick proxy
    buttons[10] = remote.btn.components.F2 ? 1 : 0; // RStick proxy

    axes.assign(8, 0.0F);
    axes[0] = remote.lx;
    // The input pipeline maps x = -L_Y; Unitree ly is already forward-positive.
    axes[1] = -remote.ly;
    axes[2] = remote.rx;
    axes[3] = remote.ry;
    axes[4] = remote.L2;
    if (remote.btn.components.left)
        axes[6] = -1.0F;
    else if (remote.btn.components.right)
        axes[6] = 1.0F;
    if (remote.btn.components.up)
        axes[7] = 1.0F;
    else if (remote.btn.components.down)
        axes[7] = -1.0F;
}

using LowCmd = UNITREE_LEGGED_SDK::LowCmd;
using LowState = UNITREE_LEGGED_SDK::LowState;

// Ref-counted shared UDP low-level link. One socket serves both the robot
// state/command and the wireless remote, driven by a fixed-rate control loop.
class UdpLink
{
public:
    using StateListener = std::function<void(const LowState &)>;

    static UdpLink &instance()
    {
        static UdpLink link;
        return link;
    }

    void add_listener(StateListener listener)
    {
        std::lock_guard<std::mutex> lock(listeners_mutex_);
        listeners_.push_back(std::move(listener));
    }

    void start()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (refs_++ > 0)
            return;
        udp_ = std::make_unique<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::LOWLEVEL);
        udp_->InitCmdData(cmd_);
        state_ = LowState{};
        running_ = true;
        thread_ = std::thread([this]() { run(); });
    }

    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (refs_ == 0 || --refs_ > 0)
                return;
            running_ = false;
        }
        if (thread_.joinable())
            thread_.join();
        udp_.reset();
    }

    // Latest commanded LowCmd; the control loop owns the actual Send().
    void set_command(const LowCmd &cmd)
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        cmd_ = cmd;
    }

private:
    UdpLink() = default;
    ~UdpLink() = default;
    UdpLink(const UdpLink &) = delete;
    UdpLink &operator=(const UdpLink &) = delete;

    void run()
    {
        constexpr auto period = std::chrono::microseconds(2000); // 500 Hz low level
        auto next = std::chrono::steady_clock::now();
        while (true)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!running_)
                    break;
            }

            udp_->Recv();
            udp_->GetRecv(state_);

            std::vector<StateListener> listeners;
            {
                std::lock_guard<std::mutex> lock(listeners_mutex_);
                listeners = listeners_;
            }
            for (auto &listener : listeners)
                listener(state_);

            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                safe_.PositionLimit(cmd_);
                safe_.PowerProtect(cmd_, state_, 1);
                udp_->SetSend(cmd_);
            }
            udp_->Send();

            next += period;
            std::this_thread::sleep_until(next);
        }
    }

    UNITREE_LEGGED_SDK::Safety safe_{UNITREE_LEGGED_SDK::LeggedType::A1};
    std::unique_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
    LowCmd cmd_{};
    LowState state_{};
    std::vector<StateListener> listeners_;
    std::mutex mutex_;
    std::mutex listeners_mutex_;
    std::mutex cmd_mutex_;
    int refs_{0};
    bool running_{false};
    std::thread thread_;
};

} // namespace unitree_hw

// A1 real robot backend over the classic Unitree Legged SDK (UDP low level).
// State readback and command output only; the wireless remote is read by UnitreeInput.
class UnitreeRobot : public RobotInterface
{
public:
    explicit UnitreeRobot(const RobotConfig &config)
        : RobotInterface(config)
    {
    }

    ~UnitreeRobot() override
    {
        stop();
    }

protected:
    void start_backend() override
    {
        unitree_hw::UdpLink::instance().add_listener(
            [this](const unitree_hw::LowState &state) { on_low_state(state); });
        unitree_hw::UdpLink::instance().start();
    }

    void stop_backend() override
    {
        unitree_hw::UdpLink::instance().stop();
    }

    void send_command_impl(const RobotCommand &command) override
    {
        unitree_hw::LowCmd low_cmd{};
        low_cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
        for (std::size_t i = 0; i < dof_; ++i)
        {
            auto &motor = low_cmd.motorCmd[i];
            motor.mode = 0;
            motor.q = command.position[i];
            motor.dq = command.velocity[i];
            motor.tau = command.torque[i];
            motor.Kp = command.kp[i];
            motor.Kd = command.kd[i];
        }
        unitree_hw::UdpLink::instance().set_command(low_cmd);
    }

private:
    void on_low_state(const unitree_hw::LowState &state)
    {
        for (std::size_t i = 0; i < dof_; ++i)
        {
            const auto &motor = state.motorState[i];
            state_staging_.joint_pos[i] = motor.q;
            state_staging_.joint_vel[i] = motor.dq;
            state_staging_.joint_torque[i] = motor.tauEst;
        }

        const auto &imu = state.imu;
        state_staging_.base_quat = {imu.quaternion[0],
                                    imu.quaternion[1],
                                    imu.quaternion[2],
                                    imu.quaternion[3]};
        state_staging_.base_ang_vel[0] = imu.gyroscope[0];
        state_staging_.base_ang_vel[1] = imu.gyroscope[1];
        state_staging_.base_ang_vel[2] = imu.gyroscope[2];
        state_staging_.base_lin_acc[0] = imu.accelerometer[0];
        state_staging_.base_lin_acc[1] = imu.accelerometer[1];
        state_staging_.base_lin_acc[2] = imu.accelerometer[2];

        publish_state(state_staging_);
    }
};

// A1 input: terminal keyboard + wireless remote read from the shared low-level link.
class UnitreeInput : public InputHardware
{
public:
    UnitreeInput(const RobotConfig &config, SimulationControl *simulation)
        : InputHardware(config, simulation)
    {
        (void)config;
    }

    ~UnitreeInput() override
    {
        stop();
    }

protected:
    void start_transport() override
    {
        unitree_hw::UdpLink::instance().add_listener(
            [this](const unitree_hw::LowState &state) { on_low_state(state); });
        unitree_hw::UdpLink::instance().start();
    }

    void stop_transport() override
    {
        unitree_hw::UdpLink::instance().stop();
    }

private:
    void on_low_state(const unitree_hw::LowState &state)
    {
        std::vector<int> buttons;
        std::vector<float> axes;
        unitree_hw::parse_wireless_remote(state.wirelessRemote, buttons, axes);
        feed_joy(buttons, axes);
    }
};

FROG_REGISTER_ROBOT("a1", UnitreeRobot)
FROG_REGISTER_INPUT("a1", UnitreeInput)

#endif // FROG_DEPLOY_HAS_UNITREE
