cmake_minimum_required(VERSION 3.18)
project(VirtualSurgerySimulation VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W4 /WX-)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/include/core
    ${CMAKE_SOURCE_DIR}/include/physics
    ${CMAKE_SOURCE_DIR}/include/collision
    ${CMAKE_SOURCE_DIR}/include/haptic
    ${CMAKE_SOURCE_DIR}/include/rendering
    ${CMAKE_SOURCE_DIR}/include/io
    ${CMAKE_SOURCE_DIR}/third_party/eigen
)

set(SOURCES
    src/core/math_utils.cpp
    src/physics/tissue.cpp
    src/physics/suture.cpp
    src/physics/instrument.cpp
    src/physics/fem_solver.cpp
    src/physics/xpbd_solver.cpp
    src/physics/rigid_body_solver.cpp
    src/physics/constraint_projector.cpp
    src/collision/aabb_tree.cpp
    src/main.cpp
)

add_executable(surgery_sim ${SOURCES})
