cmake_minimum_required(VERSION 3.20)
project(Embedded_DSP_Sensor_Fusion C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

include_directories(include)

file(GLOB_RECURSE SOURCES "src/*.c")

add_executable(dsp_ekf_fusion ${SOURCES})
target_compile_options(dsp_ekf_fusion PRIVATE -Wall -Wextra -O2)
target_link_libraries(dsp_ekf_fusion PRIVATE m)
