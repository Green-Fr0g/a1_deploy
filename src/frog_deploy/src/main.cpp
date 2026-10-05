#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <frog_deploy/config/config_loader.hpp>
#include <frog_deploy/fsm/all.hpp>
#include <frog_deploy/fsm/fsm_register.hpp>
#include <frog_deploy/hardware/all.hpp>
#include <frog_deploy/policy/policy_manager.hpp>

namespace
{
    FsmManager *active_fsm = nullptr;

    void signal_handler(int)
    {
        if (active_fsm != nullptr)
            active_fsm->stop();
    }

    void usage(const char *program)
    {
        std::cout << "Usage: " << program
                  << " --robot <a1> --mode <sim|real>\n";
    }

    std::filesystem::path policy_root()
    {
        return std::filesystem::path(FROG_POLICY_ROOT);
    }
} // namespace

int main(int argc, char **argv)
{
    std::string robot = "a1"; // 目前只有 A1 一个机器人，可用 --robot 覆盖
    std::string mode;
    const auto policies = policy_root();

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            usage(argv[0]);
            return 0;
        }
        if (i + 1 >= argc)
        {
            std::cerr << "Missing value for " << arg << '\n';
            usage(argv[0]);
            return 2;
        }
        if (arg == "--robot" || arg == "-r")
            robot = argv[++i];
        else if (arg == "--mode" || arg == "-m")
            mode = argv[++i];
        else
        {
            std::cerr << "Unknown argument: " << arg << '\n';
            usage(argv[0]);
            return 2;
        }
    }

    if (robot.empty() || mode.empty() || (mode != "sim" && mode != "real"))
    {
        usage(argv[0]);
        return 2;
    }

    // Simulation backends create rclcpp nodes in their constructors. ROS2 must
    // be initialized before those objects are created.
    rclcpp::init(argc, argv);

    const std::string backend = mode == "sim" ? "sim" : robot;

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    SkillConfigLoader config;
    // Only load the robot base configuration here. Skills are selected by FSM states.
    if (!config.load(policies, robot))
    {
        std::cerr << "Failed to load configuration for robot: " << robot << '\n';
        rclcpp::shutdown();
        return 1;
    }

    // Hardware and FSM implementations self-register from their own headers
    // (hardware/all.hpp / fsm/all.hpp); selection is purely by name.
    auto robot_io = RobotRegister::instance().create(backend, config.robot());
    if (!robot_io)
    {
        std::cerr << "Unsupported hardware backend: " << backend << '\n';
        rclcpp::shutdown();
        return 1;
    }

    auto *simulation = dynamic_cast<SimulationControl *>(robot_io.get());
    auto input = InputRegister::instance().create(backend, config.robot(), simulation);
    if (!input)
    {
        std::cerr << "Unsupported input backend: " << backend << '\n';
        rclcpp::shutdown();
        return 1;
    }

    PolicyManager policy(*robot_io, *input, config, policies);
    FsmContext context{*robot_io, *input, config, policy};
    auto fsm = FsmRegister::instance().create(robot, context);
    if (!fsm)
    {
        std::cerr << "No FSM registered for robot: " << robot << '\n';
        rclcpp::shutdown();
        return 1;
    }

    robot_io->start();
    input->start();
    active_fsm = fsm.get();
    fsm->run();
    active_fsm = nullptr;
    input->stop();
    robot_io->stop();
    rclcpp::shutdown();
    return 0;
}
