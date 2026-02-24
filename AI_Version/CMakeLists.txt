cmake_minimum_required(VERSION 3.14)
project(RocketSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Optimization flags ────────────────────────────────────────────────────────
add_compile_options(-O2 -Wall -Wextra)

# ── Eigen is header-only; tell the compiler where to find it ─────────────────
# Adjust this path if your Eigen folder is somewhere else.
include_directories(${CMAKE_SOURCE_DIR})

# ── Source files ──────────────────────────────────────────────────────────────
add_executable(rocket_sim
    main.cpp
    Atmosphere.cpp
    Motor.cpp
    RocketConfig.cpp
    RocketState.cpp
    PIDController.cpp
    FlightLogger.cpp
    RocketSimulation.cpp
)

# ── Link threading library (used by chrono) ───────────────────────────────────
find_package(Threads REQUIRED)
target_link_libraries(rocket_sim Threads::Threads)
