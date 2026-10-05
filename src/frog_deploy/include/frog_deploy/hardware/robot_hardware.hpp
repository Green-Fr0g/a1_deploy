#pragma once

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <frog_deploy/config/config_types.hpp>
#include <frog_deploy/types.hpp>
#include <frog_deploy/utils/triple_buffer.hpp>

// 机器人 I/O 抽象（沿用 M20 部署的分层方式）：只读的扁平状态 + 唯一的下发命令入口。
// 实现按硬件后端分开（sim / Unitree UDP）；状态机与策略永远只看到这个接口。
class RobotInterface
{
public:
    explicit RobotInterface(const RobotConfig &config)
        : dof_(static_cast<std::size_t>(config.dof))
    {
        state_staging_.initialize(dof_);
        state_buffer_.publish(state_staging_);
    }

    virtual ~RobotInterface() = default;

    RobotInterface(const RobotInterface &) = delete;
    RobotInterface &operator=(const RobotInterface &) = delete;

    void start()
    {
        if (running_)
            return;
        start_backend();
        running_ = true;
    }

    void stop()
    {
        if (!running_)
            return;
        stop_backend();
        running_ = false;
    }

    std::vector<float> get_joint_position() const
    {
        return state_buffer_.read().joint_pos;
    }

    std::vector<float> get_joint_velocity() const
    {
        return state_buffer_.read().joint_vel;
    }

    std::vector<float> get_joint_torque() const
    {
        return state_buffer_.read().joint_torque;
    }

    std::vector<float> get_base_quaternion() const
    {
        return state_buffer_.read().base_quat;
    }

    std::vector<float> get_base_angular_velocity() const
    {
        return state_buffer_.read().base_ang_vel;
    }

    std::vector<float> get_base_linear_acceleration() const
    {
        return state_buffer_.read().base_lin_acc;
    }

    // 运行时额外输入（如视觉深度图 "depth_image"），由具体后端提供；默认无。
    virtual bool read_external_input(const std::string &name, std::vector<float> &out) const
    {
        (void)name;
        (void)out;
        return false;
    }

    // 第一帧状态到达前丢弃命令，避免用零状态驱动真机。
    void set_joint_command(const RobotCommand &command)
    {
        if (!state_received_.load())
            return;
        send_command_impl(command);
    }

protected:
    virtual void start_backend() = 0;
    virtual void stop_backend() = 0;
    virtual void send_command_impl(const RobotCommand &command) = 0;

    // 后端回调通过它发布状态。
    void publish_state(const RobotState &state)
    {
        state_buffer_.publish(state);
        state_received_.store(true);
    }

    std::size_t dof_{0};
    RobotState state_staging_;
    TripleBuffer<RobotState> state_buffer_;
    std::atomic<bool> state_received_{false};

private:
    bool running_{false};
};

// 机器人后端的按名工厂。内置后端从各自头文件自注册（sim / unitree），
// 新增机器人只需增加文件 + 汇总头一行。
class RobotRegister
{
public:
    using Factory = std::function<std::unique_ptr<RobotInterface>(const RobotConfig &)>;

    static RobotRegister &instance()
    {
        static RobotRegister registry;
        return registry;
    }

    void add(const std::string &backend, Factory factory)
    {
        factories_[backend] = std::move(factory);
    }

    std::unique_ptr<RobotInterface> create(const std::string &backend,
                                           const RobotConfig &config) const
    {
        const auto it = factories_.find(backend);
        return it == factories_.end() ? nullptr : it->second(config);
    }

private:
    RobotRegister() = default;

    std::unordered_map<std::string, Factory> factories_;
};

#ifndef FROG_CONCAT_DEFINED
#define FROG_CONCAT_DEFINED
#define FROG_CONCAT2(x, y) x##y
#define FROG_CONCAT(x, y) FROG_CONCAT2(x, y)
#endif

// 机器人后端自注册。
#define FROG_REGISTER_ROBOT(BACKEND, TYPE)                                                         \
    namespace                                                                                      \
    {                                                                                              \
    const bool FROG_CONCAT(frog_robot_reg_, __COUNTER__) = []() {                                   \
        RobotRegister::instance().add(BACKEND, [](const RobotConfig &config) {                      \
            return std::unique_ptr<RobotInterface>(new TYPE(config));                              \
        });                                                                                        \
        return true;                                                                               \
    }();                                                                                           \
    }
