# define the project name
project(kinko_iface C CXX)
cmake_minimum_required(VERSION 3.10)
# set(CMAKE_BUILD_TYPE "")
include(GNUInstallDirs)
find_package(PkgConfig)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --libs libmodbus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )

add_compile_definitions(LFAST_TERMINAL)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/02_CMake_Modules/")
find_package(Threads REQUIRED)
find_package(Modbus REQUIRED)
include_directories(${MODBUS_INCLUDE_DIRS} )

# add_library(bash_wrapper SHARED bash_wrapper.cc)
add_library(terminal_iface SHARED TerminalInterface.cc)
add_library(kinko_iface_lib SHARED KinkoDriver.cc)
target_link_libraries(kinko_iface_lib terminal_iface ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})


add_executable(
    main
    main.cc
)
target_link_libraries(main terminal_iface kinko_iface_lib)

# add_executable(
#     main
#     main.cc
#     kinkoDriver.cc
#     TerminalInterface.cc
# )
# target_link_libraries(main ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})