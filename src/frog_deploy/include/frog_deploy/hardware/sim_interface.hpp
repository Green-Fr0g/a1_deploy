#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <a1_sim_msgs/msg/depth_image.hpp>
#include <a1_sim_msgs/msg/imu_data.hpp>
#include <a1_sim_msgs/msg/robot_cmd.hpp>
#include <a1_sim_msgs/msg/robot_data.hpp>
#include <a1_sim_msgs/srv/reset.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/empty.hpp>

#include <frog_deploy/hardware/input_hardware.hpp>
#include <frog_deploy/hardware/robot_hardware.hpp>
#include <frog_deploy/utils/thread_timer.hpp>

// MuJoCo 仿真机器人：/{robot}/state、/{robot}/imu、/{robot}/depth 订阅，
// /{robot}/command 发布，以及 /{robot}/reset、/{robot}/pause、/{robot}/resume 服务
// （服务名/类型与 a1_mujoco_sim 节点一致）。
class SimRobot : public RobotInterface, public SimulationControl
{
public:
    explicit SimRobot(const RobotConfig &config)
        : RobotInterface(config)
    {
        node_ = std::make_shared<rclcpp::Node>("sim_robot_" + config.robot_name);
        robot_name_ = config.robot_name;
        reset_service_ = "/" + robot_name_ + "/reset";
        pause_service_ = "/" + robot_name_ + "/pause";
        resume_service_ = "/" + robot_name_ + "/resume";

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        command_pub_ = node_->create_publisher<a1_sim_msgs::msg::RobotCmd>(
            "/" + robot_name_ + "/command", qos);
        state_sub_ = node_->create_subscription<a1_sim_msgs::msg::RobotData>(
            "/" + robot_name_ + "/state", qos,
            [this](const a1_sim_msgs::msg::RobotData::SharedPtr msg) { on_state(msg); });
        imu_sub_ = node_->create_subscription<a1_sim_msgs::msg::ImuData>(
            "/" + robot_name_ + "/imu", qos,
            [this](const a1_sim_msgs::msg::ImuData::SharedPtr msg) { on_imu(msg); });
        depth_sub_ = node_->create_subscription<a1_sim_msgs::msg::DepthImage>(
            "/" + robot_name_ + "/depth", qos,
            [this](const a1_sim_msgs::msg::DepthImage::SharedPtr msg) { on_depth(msg); });

        reset_client_ = node_->create_client<a1_sim_msgs::srv::Reset>(reset_service_);
        pause_client_ = node_->create_client<std_srvs::srv::Empty>(pause_service_);
        resume_client_ = node_->create_client<std_srvs::srv::Empty>(resume_service_);
    }

    ~SimRobot() override
    {
        stop();
    }

    void reset() override
    {
        auto request = std::make_shared<a1_sim_msgs::srv::Reset::Request>();
        request->command = 0;
        call(reset_client_, reset_service_, request);
    }

    void toggle_pause() override
    {
        const bool resume = paused_.load();
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        if (call(resume ? resume_client_ : pause_client_,
                 resume ? resume_service_ : pause_service_, request))
            paused_.store(!resume);
    }

protected:
    void start_backend() override
    {
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() { executor_->spin(); });
    }

    void stop_backend() override
    {
        if (!executor_)
            return;

        executor_->cancel();
        if (executor_thread_.joinable())
            executor_thread_.join();
        executor_.reset();
    }

    void send_command_impl(const RobotCommand &command) override
    {
        a1_sim_msgs::msg::RobotCmd msg;
        msg.joints_cmd.resize(dof_);
        for (std::size_t i = 0; i < dof_; ++i)
        {
            msg.joints_cmd[i].position = command.position[i];
            msg.joints_cmd[i].velocity = command.velocity[i];
            msg.joints_cmd[i].torque = command.torque[i];
            msg.joints_cmd[i].kp = command.kp[i];
            msg.joints_cmd[i].kd = command.kd[i];
        }
        command_pub_->publish(msg);
    }

private:
    void on_state(const a1_sim_msgs::msg::RobotData::SharedPtr msg)
    {
        for (std::size_t i = 0; i < dof_; ++i)
        {
            state_staging_.joint_pos[i] = msg->joints_data[i].position;
            state_staging_.joint_vel[i] = msg->joints_data[i].velocity;
            state_staging_.joint_torque[i] = msg->joints_data[i].torque;
        }
        publish_state(state_staging_);
    }

    void on_imu(const a1_sim_msgs::msg::ImuData::SharedPtr msg)
    {
        state_staging_.base_quat = {static_cast<float>(msg->orientation_w),
                                    static_cast<float>(msg->orientation_x),
                                    static_cast<float>(msg->orientation_y),
                                    static_cast<float>(msg->orientation_z)};

        state_staging_.base_ang_vel[0] = static_cast<float>(msg->angular_velocity_x);
        state_staging_.base_ang_vel[1] = static_cast<float>(msg->angular_velocity_y);
        state_staging_.base_ang_vel[2] = static_cast<float>(msg->angular_velocity_z);

        state_staging_.base_lin_acc[0] = static_cast<float>(msg->linear_acceleration_x);
        state_staging_.base_lin_acc[1] = static_cast<float>(msg->linear_acceleration_y);
        state_staging_.base_lin_acc[2] = static_cast<float>(msg->linear_acceleration_z);

        publish_state(state_staging_);
    }

    // 深度图（a1_sim_msgs/DepthImage, 32FC1 米）：按 extreme-parkour 训练预处理
    //   clip(0, far_clip=2) -> crop(60x106 -> 58x98) -> resize(->58x87) -> /2 - 0.5
    // 结果作为运行时输入 "depth_image"（58*87=5046）供视觉模型使用。
    void on_depth(const a1_sim_msgs::msg::DepthImage::SharedPtr msg)
    {
        std::vector<float> image;
        if (!preprocess_depth(*msg, image))
            return;
        std::lock_guard<std::mutex> lock(depth_mutex_);
        depth_image_ = std::move(image);
        depth_ready_ = true;
    }

    bool read_external_input(const std::string &name, std::vector<float> &out) const override
    {
        if (name != "depth_image")
            return false;
        std::lock_guard<std::mutex> lock(depth_mutex_);
        if (!depth_ready_)
            return false;
        out = depth_image_;
        return true;
    }

    static constexpr int kDepthOutH = 58;
    static constexpr int kDepthOutW = 87;
    static constexpr float kDepthFarClip = 2.0F;

    bool preprocess_depth(const a1_sim_msgs::msg::DepthImage &msg, std::vector<float> &out)
    {
        if (msg.encoding != "32FC1" || msg.is_bigendian != 0)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "Unsupported depth encoding '%s'", msg.encoding.c_str());
            return false;
        }

        const int height = static_cast<int>(msg.height);
        const int width = static_cast<int>(msg.width);
        const int stride = static_cast<int>(msg.step);
        if (height <= 0 || width <= 0 || stride < width * 4 ||
            msg.data.size() < static_cast<std::size_t>(stride) * static_cast<std::size_t>(height))
            return false;

        // 训练 original=(106,60)，先裁掉 [:-2, 4:-4] 得到 (58,98)；其它尺寸直接缩放到目标。
        const bool training_size = (height == 60 && width == 106);
        const int crop_w = training_size ? 98 : width;
        const int crop_h = training_size ? 58 : height;
        const int offset_x = training_size ? 4 : 0;

        const auto sample = [&](int y, int x) -> float {
            float value = 0.0F;
            std::memcpy(&value,
                        msg.data.data() + static_cast<std::size_t>(y) * stride + static_cast<std::size_t>(x) * 4,
                        sizeof(float));
            return std::clamp(value, 0.0F, kDepthFarClip);
        };

        out.assign(static_cast<std::size_t>(kDepthOutH) * kDepthOutW, 0.0F);
        const float scale_y = static_cast<float>(crop_h) / kDepthOutH;
        const float scale_x = static_cast<float>(crop_w) / kDepthOutW;
        for (int oy = 0; oy < kDepthOutH; ++oy)
        {
            const float fy = (static_cast<float>(oy) + 0.5F) * scale_y - 0.5F;
            const int y0 = std::clamp(static_cast<int>(std::floor(fy)), 0, crop_h - 1);
            const int y1 = std::min(y0 + 1, crop_h - 1);
            const float wy = std::clamp(fy - std::floor(fy), 0.0F, 1.0F);
            for (int ox = 0; ox < kDepthOutW; ++ox)
            {
                const float fx = (static_cast<float>(ox) + 0.5F) * scale_x - 0.5F;
                const int x0 = std::clamp(static_cast<int>(std::floor(fx)), 0, crop_w - 1);
                const int x1 = std::min(x0 + 1, crop_w - 1);
                const float wx = std::clamp(fx - std::floor(fx), 0.0F, 1.0F);

                const float v00 = sample(y0, offset_x + x0);
                const float v01 = sample(y0, offset_x + x1);
                const float v10 = sample(y1, offset_x + x0);
                const float v11 = sample(y1, offset_x + x1);
                const float top = v00 + (v01 - v00) * wx;
                const float bottom = v10 + (v11 - v10) * wx;
                const float value = top + (bottom - top) * wy;
                out[static_cast<std::size_t>(oy) * kDepthOutW + ox] = value / kDepthFarClip - 0.5F;
            }
        }
        return true;
    }

    // 发送请求；服务未就绪时只告警一次，避免刷屏。
    template <typename ClientT, typename RequestT>
    bool call(const ClientT &client,
              const std::string &service,
              const std::shared_ptr<RequestT> &request)
    {
        if (!client->service_is_ready())
        {
            if (warned_services_.insert(service).second)
                RCLCPP_WARN(node_->get_logger(),
                            "Simulation service %s is not available "
                            "(is `ros2 run a1_mujoco_sim a1_mujoco_sim` running?)",
                            service.c_str());
            return false;
        }

        warned_services_.erase(service);
        client->async_send_request(request);
        return true;
    }

    std::string robot_name_;
    std::string reset_service_;
    std::string pause_service_;
    std::string resume_service_;
    std::unordered_set<std::string> warned_services_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<a1_sim_msgs::msg::RobotCmd>::SharedPtr command_pub_;
    rclcpp::Subscription<a1_sim_msgs::msg::RobotData>::SharedPtr state_sub_;
    rclcpp::Subscription<a1_sim_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<a1_sim_msgs::msg::DepthImage>::SharedPtr depth_sub_;
    mutable std::mutex depth_mutex_;
    std::vector<float> depth_image_;
    bool depth_ready_{false};
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr resume_client_;
    rclcpp::Client<a1_sim_msgs::srv::Reset>::SharedPtr reset_client_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    std::atomic<bool> paused_{false};
};

// Simulation input: terminal keyboard + ROS /joy. A 250 ms watchdog clears a
// silent joystick so a paused joy node cannot latch velocity.
class SimInput : public InputHardware
{
public:
    SimInput(const RobotConfig &config, SimulationControl *simulation)
        : InputHardware(config, simulation)
    {
        node_ = std::make_shared<rclcpp::Node>("sim_input");
        joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) { on_joy(msg); });
    }

    ~SimInput() override
    {
        stop();
    }

protected:
    void start_transport() override
    {
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() { executor_->spin(); });
        watchdog_.start(0.05, [this]() { check_watchdog(); });
    }

    void stop_transport() override
    {
        watchdog_.stop();

        if (!executor_)
            return;
        executor_->cancel();
        if (executor_thread_.joinable())
            executor_thread_.join();
        executor_.reset();
    }

private:
    void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        std::vector<int> buttons(msg->buttons.begin(), msg->buttons.end());
        std::vector<float> axes(msg->axes.begin(), msg->axes.end());

        // Linux joy: axis 2 is usually a trigger and rests at -1; the yaw axis
        // is axis 3. Internal layout stores yaw on axis 2, R_Y on axis 3.
        if (axes.size() > 3)
            axes[2] = axes[3];
        if (axes.size() > 4)
            axes[3] = axes[4];
        // The input pipeline maps x = -L_Y, y = -L_X, yaw = -R_X; ROS joy is
        // already positive forward/right, so invert these three.
        if (!axes.empty())
            axes[0] = -axes[0];
        if (axes.size() > 1)
            axes[1] = -axes[1];
        if (axes.size() > 2)
            axes[2] = -axes[2];

        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            joy_received_ = true;
            last_joy_ = std::chrono::steady_clock::now();
        }

        feed_joy(buttons, axes);
    }

    void check_watchdog()
    {
        std::lock_guard<std::mutex> lock(joy_mutex_);
        if (joy_received_ &&
            std::chrono::steady_clock::now() - last_joy_ > std::chrono::milliseconds(250))
            reset_joystick();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;
    ThreadTimer watchdog_;
    std::mutex joy_mutex_;
    bool joy_received_{false};
    std::chrono::steady_clock::time_point last_joy_{};
};

FROG_REGISTER_ROBOT("sim", SimRobot)
FROG_REGISTER_INPUT("sim", SimInput)
