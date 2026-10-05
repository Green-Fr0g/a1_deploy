#pragma once

// FSM 聚合头。每个机器人 FSM 从自身头文件自注册（FROG_REGISTER_FSM），
// 新增机器人只需在此追加一行 include，无需改动 main.cpp 或工厂注册表。

#include <frog_deploy/fsm/robot/a1/a1_fsm.hpp>
