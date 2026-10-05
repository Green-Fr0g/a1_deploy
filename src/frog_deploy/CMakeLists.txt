cmake_minimum_required(VERSION 3.16)
project(frog_deploy)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(a1_sim_msgs REQUIRED)

set(FROG_POLICY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../policy")
set(INFERENCE_RUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/interface_runtime")
set(LIBTORCH_ROOT "${INFERENCE_RUNTIME_ROOT}/libtorch")
set(ONNXRUNTIME_ROOT "${INFERENCE_RUNTIME_ROOT}/onnxruntime")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS "${ONNXRUNTIME_ROOT}/include" NO_DEFAULT_PATH)
find_library(ONNXRUNTIME_LIBRARY onnxruntime
  PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR "ONNX Runtime is required for AMP; run scripts/download_inference_runtime.sh onnx")
endif()
if(EXISTS "${LIBTORCH_ROOT}/share/cmake/Torch/TorchConfig.cmake")
  find_package(Torch REQUIRED PATHS "${LIBTORCH_ROOT}/share/cmake/Torch" NO_DEFAULT_PATH)
endif()

set(UNITREE_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../thirdparty/unitree_legged_sdk")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(UNITREE_ARCH arm64)
else()
  set(UNITREE_ARCH amd64)
endif()
set(UNITREE_LIB_DIR "${UNITREE_SDK_ROOT}/lib")
set(UNITREE_LIBRARY "${UNITREE_LIB_DIR}/libunitree_legged_sdk_${UNITREE_ARCH}.so")
# unitree_legged_sdk 的预编译库内部依赖 lcm（即使是 UDP 接口，链接时也需要）。
# 缺少任一依赖时只关闭真机后端（unitree_interface.hpp 被编译为空），
# 仿真构建照常进行，不会因为没装 lcm 而整个包构建失败。
find_library(LCM_LIBRARY lcm)
if(LCM_LIBRARY AND EXISTS "${UNITREE_LIBRARY}")
  set(FROG_DEPLOY_HAS_UNITREE 1)
else()
  set(FROG_DEPLOY_HAS_UNITREE 0)
  message(WARNING
    "Unitree real-robot backend disabled: "
    "${UNITREE_LIBRARY} or lcm not found (run scripts/download_inference_runtime.sh, "
    "apt install liblcm-dev). Simulation mode is unaffected.")
endif()

add_executable(frog_deploy_main src/main.cpp)
target_compile_definitions(frog_deploy_main PRIVATE
  FROG_POLICY_ROOT="${FROG_POLICY_ROOT}"
  FROG_DEPLOY_HAS_UNITREE=${FROG_DEPLOY_HAS_UNITREE}
)
target_include_directories(frog_deploy_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${UNITREE_SDK_ROOT}/include
)
ament_target_dependencies(frog_deploy_main rclcpp sensor_msgs std_srvs a1_sim_msgs)
target_link_libraries(frog_deploy_main yaml-cpp)
target_include_directories(frog_deploy_main PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(frog_deploy_main ${ONNXRUNTIME_LIBRARY})
set_property(TARGET frog_deploy_main APPEND PROPERTY BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib")
set_property(TARGET frog_deploy_main APPEND PROPERTY INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib")
target_link_directories(frog_deploy_main PRIVATE
  ${UNITREE_LIB_DIR}
)
if(FROG_DEPLOY_HAS_UNITREE)
  target_link_libraries(frog_deploy_main ${UNITREE_LIBRARY} ${LCM_LIBRARY})
  set_property(TARGET frog_deploy_main APPEND PROPERTY BUILD_RPATH "${UNITREE_LIB_DIR}")
  set_property(TARGET frog_deploy_main APPEND PROPERTY INSTALL_RPATH "${UNITREE_LIB_DIR}")
endif()

if(Torch_FOUND)
  target_link_libraries(frog_deploy_main ${TORCH_LIBRARIES})
  target_compile_options(frog_deploy_main PRIVATE ${TORCH_CXX_FLAGS})
  target_compile_definitions(frog_deploy_main PRIVATE FROG_DEPLOY_HAS_TORCH=1)
  set_property(TARGET frog_deploy_main APPEND PROPERTY BUILD_RPATH "${LIBTORCH_ROOT}/lib")
  set_property(TARGET frog_deploy_main APPEND PROPERTY INSTALL_RPATH "${LIBTORCH_ROOT}/lib")
else()
  message(FATAL_ERROR "LibTorch is required for the configured TorchScript locomotion policy")
endif()

if(BUILD_TESTING)
  enable_testing()
  find_package(Threads REQUIRED)
  add_executable(test_a1 test/test_a1.cpp)
  target_include_directories(test_a1 PRIVATE include)
  target_compile_definitions(test_a1 PRIVATE FROG_DEPLOY_HAS_TORCH=0
    FROG_POLICY_ROOT="${FROG_POLICY_ROOT}")
  target_link_libraries(test_a1 yaml-cpp Threads::Threads)
  add_test(NAME a1_pipeline COMMAND test_a1)
endif()

install(TARGETS frog_deploy_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
