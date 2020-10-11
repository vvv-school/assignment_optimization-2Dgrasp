cmake_minimum_required(VERSION 3.5)
project(assignment_optimization-2Dgrasplib LANGUAGES CXX C VERSION 1.0.0)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YCM REQUIRED)
find_package(YARP 3.3.0 REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
find_package(GSL REQUIRED)
find_package(IPOPT REQUIRED)

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" OFF)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

icubcontrib_set_default_prefix()

set(${PROJECT_NAME}_SRC lib/problem.cpp lib/solver.cpp)
set(${PROJECT_NAME}_HDR lib/problem.h lib/solver.h)

include_directories(${CMAKE_SOURCE_DIR}/lib ${IPOPT_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION}
                                      PUBLIC_HEADER "${${PROJECT_NAME}_HDR}")
set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
target_compile_definitions(${PROJECT_NAME} PUBLIC ${IPOPT_DEFINITIONS} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} PUBLIC ${YARP_LIBRARIES} ${IPOPT_LIBRARIES} PRIVATE ${GSL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                  ${IPOPT_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                     COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}" COMPONENT dev)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VERSION ${${PROJECT_NAME}_VERSION}
                                            COMPATIBILITY AnyNewerVersion
                                            EXPORT ${PROJECT_NAME}
                                            VARS_PREFIX ${PROJECT_NAME}
                                            DEPENDENCIES "YARP REQUIRED" "IPOPT REQUIRED"
                                            PRIVATE_DEPENDENCIES "GSL REQUIRED"
                                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)
include(AddUninstallTarget)

project(assignment_optimization-2Dgrasp)
add_executable(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} assignment_optimization-2Dgrasplib)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_custom_target(copy_scripts_in_build ALL)
file(GLOB scripts ${CMAKE_SOURCE_DIR}/scripts/*.*)
add_custom_command(TARGET copy_scripts_in_build POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy ${scripts} ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
                   COMMENT "Copying ${scripts} to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/")
install(PROGRAMS ${scripts} DESTINATION bin)
