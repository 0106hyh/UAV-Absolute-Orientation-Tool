cmake_minimum_required(VERSION 3.15)
project(UAV_AbsoluteOrientation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Force MSVC to use UTF-8 encoding for source and output
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# ============================================================
# 通过 vcpkg 自动查找已安装的第三方库
# 运行 cmake 时需要指定:
#   -DCMAKE_TOOLCHAIN_FILE=C:/Users/HYH123/vcpkg/scripts/buildsystems/vcpkg.cmake
# ============================================================

# Eigen3 —— 线性代数核心库（矩阵运算、SVD、Umeyama算法）
find_package(Eigen3 CONFIG REQUIRED)

# GeographicLib —— 大地测量库（LLA 到 ENU 坐标转换）
find_package(GeographicLib CONFIG REQUIRED)

# yaml-cpp —— 配置文件解析库
find_package(yaml-cpp CONFIG REQUIRED)

# nlohmann_json —— JSON 解析库
find_package(nlohmann_json CONFIG REQUIRED)

# 定义可执行文件
add_executable(uav_orientation
    main.cpp
    coordinate_transform.cpp
    alignment_solver.cpp
    config_reader.cpp
    simulation_engine.cpp
)

# 链接依赖库
target_link_libraries(uav_orientation PRIVATE
    Eigen3::Eigen
    GeographicLib::GeographicLib
    yaml-cpp::yaml-cpp
    nlohmann_json::nlohmann_json
)
