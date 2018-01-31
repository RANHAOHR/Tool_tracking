cmake_minimum_required(VERSION 3.5)
project(Tool_tracking)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        tool_model/include/tool_model_lib/tool_model.h
        tool_model/src/showing_image.cpp
        tool_model/src/test_seg.cpp
        tool_model/src/tool_model.cpp
        tool_model/src/tool_model_main.cpp
        tool_tracking/include/tool_tracking/kalman_filter.h
        tool_tracking/include/tool_tracking/particle_filter.h
        tool_tracking/src/kalman_filter.cpp
        tool_tracking/src/particle_filter.cpp
        tool_tracking/src/tracking_kalman.cpp
        tool_tracking/src/tracking_particle.cpp)

add_executable(Tool_tracking ${SOURCE_FILES})