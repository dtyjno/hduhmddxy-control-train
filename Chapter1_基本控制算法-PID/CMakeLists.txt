cmake_minimum_required(VERSION 3.10)
# 设置C和C++编译器路径
# set(CMAKE_C_COMPILER "C:/msys64/ucrt64/bin/gcc.exe")
# set(CMAKE_CXX_COMPILER "C:/msys64/ucrt64/bin/g++.exe")

project(pid_pso VERSION 1.0 LANGUAGES C CXX)

# 设置C标准和C++标准
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# 添加编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置包含目录
include_directories(${PROJECT_SOURCE_DIR}/src)
# include_directories(${PROJECT_SOURCE_DIR}/external/some_external_library/include)

# 设置源文件
set(SOURCES
    src/PID.c
    src/FuzzyPID.cpp
    src/pso.cpp
)

# 添加可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 链接外部库
# target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/external/some_external_library/lib/libexternal.a)