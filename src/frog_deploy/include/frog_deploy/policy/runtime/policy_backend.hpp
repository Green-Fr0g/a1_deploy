#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <frog_deploy/config/config_types.hpp>

#ifndef FROG_DEPLOY_HAS_TORCH
#if __has_include(<torch/script.h>)
#define FROG_DEPLOY_HAS_TORCH 1
#include <ATen/Parallel.h>
#include <torch/script.h>
#else
#define FROG_DEPLOY_HAS_TORCH 0
#endif
#elif FROG_DEPLOY_HAS_TORCH
#include <ATen/Parallel.h>
#include <torch/script.h>
#endif

#if __has_include(<onnxruntime_cxx_api.h>)
#define FROG_DEPLOY_HAS_ONNX 1
#include <onnxruntime_cxx_api.h>
#else
#define FROG_DEPLOY_HAS_ONNX 0
#endif

class PolicyBackend
{
public:
    virtual ~PolicyBackend() = default;

    // 多输入：inputs 按顺序对应模型 forward 的位置参数（每个输入为扁平的 1xN 张量）。
    virtual bool load(const ModelSpec &spec,
                      const std::filesystem::path &model_path) = 0;
    virtual bool infer(const std::vector<std::vector<float>> &inputs,
                       std::vector<float> &output) = 0;
    virtual std::size_t output_dim() const = 0;

    // 有状态模型（如 RNN/GRU）可在技能开始时复位；默认无操作。
    virtual void reset() {}

    // 单输入便捷入口（内部包装为单元素输入列表）。
    bool infer(const std::vector<float> &observation, std::vector<float> &output)
    {
        return infer(std::vector<std::vector<float>>{observation}, output);
    }

    // 单模型便捷入口（由 PolicyConfig 合成 inputs=["obs"], output="action"）。
    bool load(const PolicyConfig &config, const std::filesystem::path &model_path)
    {
        ModelSpec spec;
        spec.name = config.policy_name.empty() ? "policy" : config.policy_name;
        spec.file = config.model_name.empty() ? model_path.filename().string() : config.model_name;
        spec.inputs = {"obs"};
        spec.output = "action";
        spec.is_action = true;
        return load(spec, model_path);
    }
};


class TorchPolicyBackend final : public PolicyBackend
{
public:
    using PolicyBackend::infer;
    using PolicyBackend::load;

    bool load(const ModelSpec &spec,
              const std::filesystem::path &model_path) override
    {
        spec_ = spec;
#if FROG_DEPLOY_HAS_TORCH
        try
        {
            at::set_num_threads(1);
            module_ = torch::jit::load(model_path.string());
            module_.eval();
            loaded_ = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[TorchPolicy] Failed to load " << model_path << ": " << e.what() << '\n';
            loaded_ = false;
            return false;
        }
#else
        (void)model_path;
        loaded_ = false;
        return false;
#endif
    }

    bool infer(const std::vector<std::vector<float>> &inputs,
               std::vector<float> &output) override
    {
        if (!loaded_ || inputs.empty())
            return false;
#if FROG_DEPLOY_HAS_TORCH
        try
        {
            torch::NoGradGuard guard;
            at::set_num_threads(1);
            std::vector<torch::Tensor> tensors;
            tensors.reserve(inputs.size());
            for (const auto &input : inputs)
            {
                tensors.push_back(torch::tensor(input, torch::TensorOptions().dtype(torch::kFloat32))
                                      .reshape({1, static_cast<int64_t>(input.size())}));
            }
            std::vector<torch::jit::IValue> arguments;
            arguments.reserve(tensors.size());
            for (auto &tensor : tensors)
                arguments.emplace_back(tensor);

            auto result = module_.forward(arguments).toTensor().to(torch::kCPU).contiguous();
            const auto *data = result.data_ptr<float>();
            const auto numel = static_cast<std::size_t>(result.numel());
            output.assign(data, data + numel);
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[TorchPolicy] Inference error: " << e.what() << '\n';
            return false;
        }
#else
        (void)output;
        return false;
#endif
    }

    std::size_t output_dim() const override
    {
        return output_dim_;
    }

    void reset() override
    {
#if FROG_DEPLOY_HAS_TORCH
        if (!loaded_)
            return;
        try
        {
            module_.run_method("reset");
        }
        catch (const std::exception &)
        {
            // 模型没有 reset 方法时忽略。
        }
#endif
    }

private:
    ModelSpec spec_;
    std::size_t output_dim_{0};
    bool loaded_{false};
#if FROG_DEPLOY_HAS_TORCH
    torch::jit::script::Module module_;
#endif
};

class OnnxPolicyBackend final : public PolicyBackend
{
public:
    using PolicyBackend::infer;
    using PolicyBackend::load;

    bool load(const ModelSpec &spec,
              const std::filesystem::path &model_path) override
    {
        spec_ = spec;
#if FROG_DEPLOY_HAS_ONNX
        try
        {
            env_.emplace(ORT_LOGGING_LEVEL_WARNING, "frog_deploy");
            session_options_.SetIntraOpNumThreads(1);
            session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            session_.emplace(*env_, model_path.c_str(), session_options_);

            const auto input_count = session_->GetInputCount();
            if (session_->GetOutputCount() != 1)
                throw std::runtime_error("Expected exactly one model output");
            if (!spec.inputs.empty() && spec.inputs.size() != input_count)
                throw std::runtime_error("Model input count does not match the config");

            input_names_.clear();
            for (std::size_t i = 0; i < input_count; ++i)
            {
                const auto input_tensor = session_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo();
                const auto input_shape = input_tensor.GetShape();
                if (input_tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                    input_shape.size() != 2 || (input_shape[0] > 0 && input_shape[0] != 1))
                    throw std::runtime_error("Model inputs must be float 1xN tensors");
                auto input_name = session_->GetInputNameAllocated(i, allocator_);
                input_names_.emplace_back(input_name.get());
            }

            const auto output_tensor = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
            const auto output_shape = output_tensor.GetShape();
            if (output_tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
                output_shape.size() != 2 || (output_shape[0] > 0 && output_shape[0] != 1))
                throw std::runtime_error("Model output must be a float 1xN tensor");
            {
                auto output_name = session_->GetOutputNameAllocated(0, allocator_);
                output_name_ = output_name.get();
            }
            output_dim_ = output_shape[1] > 0 ? static_cast<std::size_t>(output_shape[1]) : 0;

            loaded_ = true;
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[OnnxPolicy] Failed to load " << model_path << ": " << e.what() << '\n';
            loaded_ = false;
            return false;
        }
#else
        (void)model_path;
        std::cerr << "[OnnxPolicy] ONNX Runtime is unavailable\n";
        loaded_ = false;
        return false;
#endif
    }

    bool infer(const std::vector<std::vector<float>> &inputs,
               std::vector<float> &output) override
    {
        if (!loaded_)
            return false;
#if FROG_DEPLOY_HAS_ONNX
        if (inputs.size() != input_names_.size())
            return false;
        try
        {
            std::vector<std::array<int64_t, 2>> shapes;
            shapes.reserve(inputs.size());
            for (const auto &input : inputs)
                shapes.push_back({1, static_cast<int64_t>(input.size())});

            std::vector<Ort::Value> tensors;
            tensors.reserve(inputs.size());
            for (std::size_t i = 0; i < inputs.size(); ++i)
            {
                tensors.push_back(Ort::Value::CreateTensor<float>(memory_info_,
                                                                  const_cast<float *>(inputs[i].data()),
                                                                  inputs[i].size(),
                                                                  shapes[i].data(),
                                                                  shapes[i].size()));
            }

            std::vector<const char *> input_names;
            input_names.reserve(input_names_.size());
            for (const auto &name : input_names_)
                input_names.push_back(name.c_str());
            const char *output_names[] = {output_name_.c_str()};

            auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                                input_names.data(),
                                                tensors.data(),
                                                tensors.size(),
                                                output_names,
                                                1);
            const auto *data = output_tensors.front().GetTensorMutableData<float>();
            const auto count = output_tensors.front().GetTensorTypeAndShapeInfo().GetElementCount();
            output.assign(data, data + count);
            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "[OnnxPolicy] Inference error: " << e.what() << '\n';
            return false;
        }
#else
        (void)output;
        return false;
#endif
    }

    std::size_t output_dim() const override
    {
        return output_dim_;
    }

private:
    ModelSpec spec_;
    std::size_t output_dim_{0};
    bool loaded_{false};
#if FROG_DEPLOY_HAS_ONNX
    std::optional<Ort::Env> env_;
    Ort::SessionOptions session_options_;
    Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::optional<Ort::Session> session_;
    Ort::AllocatorWithDefaultOptions allocator_;
    std::vector<std::string> input_names_;
    std::string output_name_;
#endif
};
