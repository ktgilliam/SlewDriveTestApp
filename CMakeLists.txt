# define the project name
project(kinco_iface C CXX)
cmake_minimum_required(VERSION 3.10)
set(CXX_STANDARD 20)

# CXX_STANDARD_REQUIRED
include(GNUInstallDirs)
find_package(PkgConfig)

# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --libs libmodbus")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )

add_compile_definitions(LFAST_TERMINAL)
add_compile_definitions(JSSG_2006_TEST_MODE)
# add_compile_definitions(DEBUG_MODE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/02_CMake_Modules/")
find_package(Threads REQUIRED)
find_package(Modbus REQUIRED)
include_directories(${MODBUS_INCLUDE_DIRS} )

set(PROJ_LIBS "")
# add_library(bash_wrapper SHARED bash_wrapper.cc)
add_library(terminal_iface SHARED TerminalInterface.cc)
list(APPEND PROJ_LIBS terminal_iface)

add_library(kinco_iface_lib SHARED KincoDriver.cc)
target_link_libraries(kinco_iface_lib terminal_iface ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})
list(APPEND PROJ_LIBS kinco_iface_lib)

add_executable(
    main
    main.cc
    SlewDriveTest.cc
)



target_link_libraries(main ${PROJ_LIBS})

# add_executable(
#     main
#     main.cc
#     kincoDriver.cc
#     TerminalInterface.cc
# )
# target_link_libraries(main ${MODBUS_LIBRARIES} ${MODBUS_LIBRARY})