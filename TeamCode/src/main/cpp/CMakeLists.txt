# Minimum version for Android Studio NDK integration
cmake_minimum_required(VERSION 3.18.1)

# Name of the project
project(ShooterMath)

# Create shared library (.so)
add_library(
        shootermath
        SHARED
        ShooterMath.cpp
)

# Link to Android log library (standard)
find_library(log-lib log)
target_link_libraries(shootermath ${log-lib})

# --- Optimization flags ---
# Enable high performance math and vectorization for ARM CPU
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math -funroll-loops -march=armv8-a -mfpu=neon")

# Ensure same flags for debug (optional)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O3 -ffast-math")

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
