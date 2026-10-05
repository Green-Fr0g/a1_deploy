#pragma once

// Hardware backend aggregation. Each backend registers itself from its own
// header (FROG_REGISTER_ROBOT / FROG_REGISTER_INPUT); adding a backend means
// adding one include line here and nothing else.

#include <frog_deploy/hardware/sim_interface.hpp>
#include <frog_deploy/hardware/unitree_interface.hpp>
