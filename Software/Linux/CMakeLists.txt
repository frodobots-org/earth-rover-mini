cmake_minimum_required(VERSION 3.8)
project(example)

include_directories(
    toolchain/media/include
    toolchain/media/include/sample_comm
    toolchain/media/include/rkaiq
    toolchain/media/include/rkaiq/uAPI2
    toolchain/media/include/rkaiq/algos
    toolchain/media/include/rkaiq/common
    toolchain/media/include/rkaiq/xcore
    toolchain/media/include/rkaiq/iq_parser_v2
    toolchain/media/include/rkaiq/iq_parser
)

link_directories(
    toolchain/media/lib
)

add_executable(move src/Examples/move.cpp)
add_executable(tcp_bridge src/Examples/bridge.c)
add_executable(sample_demo_dual_camera src/Examples/sample_demo_dual_camera.c)

target_link_libraries(sample_demo_dual_camera
    rtsp
    sample_comm
    rkaiq
    rockiva
    rknnmrt
    rockit_full
    rockchip_mpp
    drm
    rga
)
