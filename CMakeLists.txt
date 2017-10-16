#
# This is based on orocos sample cmakelists, modified slightly
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.8.3
#
cmake_minimum_required(VERSION 2.8.3)
 
#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(orocos_opensot_ik)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wmaybe-uninitialized -Wuninitialized")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wmaybe-uninitialized -Wuninitialized")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

 
# current source and include:
include_directories(${PROJECT_SOURCE_DIR}/include)

 
# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(YCM QUIET)
find_package(rtt-core-extensions REQUIRED)
find_package(urdf REQUIRED COMPONENTS)
find_package(srdfdom REQUIRED COMPONENTS)
find_package(OpenSoT REQUIRED)
find_package(orocos-ocl REQUIRED)
find_package(XBotInterface REQUIRED)
find_package(rtt_sensor_msgs REQUIRED)
find_package(mpcqp_walking REQUIRED)
find_package(compliant_stabilizer REQUIRED)

message(STATUS "test ${orocos-ocl_LIBRARY_DIRS}")

include_directories(${RTT-CORE-EXTENSIONS_INCLUDE_DIRS}
 ${mpcqp_walking_INCLUDE_DIRS} ${compliant_stabilizer_INCLUDE_DIRS} ${OpenSoT_INCLUDE_DIRS} ${XBotInterface_INCLUDE_DIRS})
link_directories(${RTT-CORE-EXTENSIONS_LIBRARY_DIRS})


# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
 
 
# Creates a component library libexample-<target>.so
# and installs in the directory lib/orocos/example/
#
orocos_component(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/orocos_opensot_ik.cpp
                                       ${PROJECT_SOURCE_DIR}/src/opensot_ik.cpp)
# target_link_libraries()
if(OROCOS_TARGET STREQUAL "xenomai")
	target_link_libraries(${CMAKE_PROJECT_NAME} ${RST-RT_LIBRARIES} ${RTT-CORE-EXTENSIONS_LIBRARIES} ${urdf_LIBRARIES}
                      ${USE_OROCOS_LIBRARIES} ${OROCOS-RTT_LIBRARIES}
                      ${mpcqp_walking_LIBRARIES} ${compliant_stabilizer_LIBRARIES} ${OpenSoT_LIBRARIES}
                      ${XBotInterface_LIBRARIES}
                      $ENV{ROBOTOLOGY_ROOT}/orocos_ws/install_isolated/lib/librtt_ros-xenomai.so
                      $ENV{ROBOTOLOGY_ROOT}/orocos_ws/install_isolated/lib/librtt_rosclock-xenomai.so
                      $ENV{ROBOTOLOGY_ROOT}/orocos_ws/install_isolated/lib/librtt_rostopic-xenomai.so)
else()
	target_link_libraries(${CMAKE_PROJECT_NAME} ${RST-RT_LIBRARIES} ${RTT-CORE-EXTENSIONS_LIBRARIES} ${urdf_LIBRARIES}
                      ${USE_OROCOS_LIBRARIES} ${OROCOS-RTT_LIBRARIES}
                      ${mpcqp_walking_LIBRARIES} ${compliant_stabilizer_LIBRARIES} ${OpenSoT_LIBRARIES}
                      ${XBotInterface_LIBRARIES} ${orocos-ocl_LIBRARY_DIRS}/librtt_rostopic-gnulinux.so
                      ${orocos-ocl_LIBRARY_DIRS}/librtt_rosclock-gnulinux.so)
endif()
 
#
# Additional headers (not in typekit):
#
# Installs in the include/orocos/example/ directory
#
# orocos_install_headers( example-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.
 
#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()

