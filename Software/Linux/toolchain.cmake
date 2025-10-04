set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   /opt/toolchain/toolchain/bin/arm-rockchip830-linux-uclibcgnueabihf-gcc)
set(CMAKE_CXX_COMPILER /opt/toolchain/toolchain/bin/arm-rockchip830-linux-uclibcgnueabihf-g++)

set(CMAKE_SYSROOT  /opt/toolchain/toolchain/arm-rockchip830-linux-uclibcgnueabihf/sysroot)
set(CMAKE_FIND_ROOT_PATH  /opt/toolchain//toolchain/arm-rockchip830-linux-uclibcgnueabihf/sysroot)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
