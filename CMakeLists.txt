cmake_minimum_required(VERSION 3.10)
project(go2)

set(CMAKE_CXX_STANDARD 17)

find_package(glfw3 REQUIRED)

set(MUJOCO_ROOT "$ENV{HOME}/.mujoco/mujoco-3.1.6")

add_executable(test_mujoco test/test_mujoco.cpp)
add_executable(interactive_mujoco src/interactive_mujoco.cpp)

target_include_directories(test_mujoco PRIVATE ${MUJOCO_ROOT}/include)
target_include_directories(interactive_mujoco PRIVATE ${MUJOCO_ROOT}/include)

target_link_libraries(test_mujoco ${MUJOCO_ROOT}/lib/libmujoco.so)
target_link_libraries(test_mujoco glfw)
target_link_libraries(interactive_mujoco ${MUJOCO_ROOT}/lib/libmujoco.so)
target_link_libraries(interactive_mujoco glfw)
