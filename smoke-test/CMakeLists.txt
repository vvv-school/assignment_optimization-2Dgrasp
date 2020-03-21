# Copyright: (C) 2020 iCub Tech Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)

project(TestAssignmentOptimization2DGrasp)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/plugins)
set(CMAKE_SHARED_MODULE_PREFIX "")

find_package(ICUBcontrib REQUIRED)
find_package(RobotTestingFramework 2.0.0 COMPONENTS DLL)
find_package(YARP 3.3.0 REQUIRED COMPONENTS os sig math robottestingframework)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

find_package(assignment_optimization-2Dgrasplib REQUIRED)

add_library(${PROJECT_NAME} MODULE test.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} PRIVATE ${RobotTestingFramework_LIBRARIES} YARP::YARP_os YARP::YARP_sig YARP::YARP_math YARP::YARP_robottestingframework
                                      assignment_optimization-2Dgrasplib::assignment_optimization-2Dgrasplib)
