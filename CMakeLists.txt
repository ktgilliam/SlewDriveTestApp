# define the project name
project(kinco_iface C CXX)
cmake_minimum_required(VERSION 3.10)
# set(CMAKE_BUILD_TYPE "")
include(GNUInstallDirs)
find_package(PkgConfig)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --libs libmodbus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/02_CMake_Modules/")
find_package(Modbus REQUIRED)
include_directories(${MODBUS_INCLUDE_DIRS} )

add_library(terminal_iface TerminalInterface.cc)
add_library(kinco_iface_lib KincoDriver.cc)
target_link_libraries(kinco_iface_lib terminal_iface ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})


add_executable(
    main
    main.cc
)
target_link_libraries(main terminal_iface kinco_iface_lib)

# add_executable(
#     main
#     main.cc
#     KincoDriver.cc
#     TerminalInterface.cc
# )
# target_link_libraries(main ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})