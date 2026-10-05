#pragma once

#include <string>
#include <vector>

struct RobotConfig
{
    std::string robot_name;
    //control_dt
    double control_dt{0.0};
    //dofs
    std::vector<std::string> joint_names;
    //default_dof_pos
    int dof{0};
    std::vector<float> default_dof_pos;
    std::vector<float> fixed_kp;
    std::vector<float> fixed_kd;
    std::vector<int> joint_mapping;
    std::vector<int> waist_joint_index;
    std::vector<int> wheel_joint_index;
    //limit_joint_position_lower
    std::vector<float> torque_limits;
    std::vector<float> joint_position_lower;
    std::vector<float> joint_position_upper;
    std::vector<float> joint_velocity_limits;
};

// 单个模型在一个技能中的角色。inputs 引用支持：
//   "obs"          运行时按 observations/observations_history 展平的观测张量
//   "obs[a:b]"     上述观测的切片（半开区间，支持负索引从尾部计数）
//   "<name>"       上游模型的 output 名，或 sensors 中声明的传感器名
struct ModelSpec
{
    std::string name;                 // 模型标识（用于日志/引用）
    std::string file;                 // 模型文件，扩展名决定后端（.pt/.jit/.onnx）
    std::vector<std::string> inputs;  // 输入引用，顺序即喂给 forward 的实参顺序
    std::string output;               // 输出张量名
    bool is_action{false};            // 该模型的输出即动作（与 output=="action" 等价）
};

struct PolicyConfig
{
    std::string policy_name;
    std::string model_name;
    //models：多模型推理管线；为空时回退到单模型 model_name
    std::vector<ModelSpec> models;
    //policy_dt
    int decimation{1};
    //observations
    std::vector<std::string> observations;
    std::vector<int> observation_dims;
    std::vector<int> observation_history;
    std::string observation_history_priority{"time"};
    int input_dim{0};
    //default_dof_pos
    int dof{0};
    std::vector<float> default_dof_pos;
    std::vector<float> kp;
    std::vector<float> kd;
    std::vector<int> joint_mapping;
    std::vector<int> waist_joint_index;
    std::vector<int> wheel_joint_index;
    //scales
    std::vector<float> ang_vel_scale{1.0F};
    std::vector<float> lin_vel_scale{1.0F};
    std::vector<float> dof_pos_scale{1.0F};
    std::vector<float> dof_vel_scale{1.0F};
    std::vector<float> command_scale{1.0F};
    std::vector<float> action_scale;
    //limit
    std::vector<float> action_lower;
    std::vector<float> action_upper;
    float clip_obs{0.0F};
};
