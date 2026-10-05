#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <frog_deploy/fsm/fsm_base.hpp>
#include <frog_deploy/fsm/fsm_manager.hpp>

class FsmFactory
{
public:
    virtual ~FsmFactory() = default;
    virtual std::shared_ptr<FsmBase> create_state(FsmContext &ctx, const std::string &name) = 0;
    virtual std::string type() const = 0;
    virtual std::vector<std::string> supported_states() const = 0;
    virtual std::string initial_state() const = 0;
};

class FsmRegister
{
public:
    static FsmRegister &instance()
    {
        static FsmRegister registry;
        return registry;
    }

    void add(std::shared_ptr<FsmFactory> factory)
    {
        if (!factory)
            return;
        factories_[factory->type()] = std::move(factory);
    }

    std::unique_ptr<FsmManager> create(const std::string &type, FsmContext &ctx)
    {
        auto it = factories_.find(type);
        if (it == factories_.end())
            return nullptr;

        auto manager = std::make_unique<FsmManager>(ctx.config.robot().control_dt);
        for (const auto &name : it->second->supported_states())
        {
            auto state = it->second->create_state(ctx, name);
            if (state)
                manager->add(std::move(state));
        }
        manager->start(it->second->initial_state());
        return manager;
    }

    bool supported(const std::string &type) const
    {
        return factories_.count(type) != 0;
    }

private:
    FsmRegister() = default;
    std::unordered_map<std::string, std::shared_ptr<FsmFactory>> factories_;
};

// Self-registration for per-robot FSM factories (rl_sar style): a robot only
// adds its header to fsm/all.hpp and its factory registers itself.
#define FROG_REGISTER_FSM(FACTORY)                                                                 \
    namespace                                                                                      \
    {                                                                                              \
    const bool FROG_CONCAT(frog_fsm_reg_, __COUNTER__) = []() {                                    \
        FsmRegister::instance().add(std::make_shared<FACTORY>());                                  \
        return true;                                                                               \
    }();                                                                                           \
    }
