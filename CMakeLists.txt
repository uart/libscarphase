################################################################################
# ScarPhase                                                                    #
################################################################################

cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME_STR libscarphase)
project(${PROJECT_NAME_STR})

# The version number.
set(LIB_SCARPHASE_VERSION_MAJOR 3)
set(LIB_SCARPHASE_VERSION_MINOR 0)

include(libscarphase.cmake)

if(${BUILD_SHARED_LIBS})
  set(SCARPHASE_LIBRARY_TYPE SHARED)
else(${BUILD_SHARED_LIBS})
  set(SCARPHASE_LIBRARY_TYPE STATIC)
endif(${BUILD_SHARED_LIBS})

add_definitions(-Wall -Wextra -Wno-deprecated)

# Configure a header file to pass some of the CMake settings
# to the source code
configure_file (
  "${PROJECT_SOURCE_DIR}/libscarphase_config.h.in"
  "${PROJECT_BINARY_DIR}/libscarphase_config.h"
  )

#
include_directories(${PROJECT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(Boost REQUIRED COMPONENTS program_options)
include_directories(${Boost_INCLUDE_DIRS})

set(SCARPHASE_SOURCE
    source/scarphase.cpp
    source/scarphase/scarphase_perfevent.cpp
    source/scarphase/internal/monitor.cpp
    source/scarphase/internal/util/signature.cpp
    source/scarphase/internal/util/perf_event_wrapper.cpp
    source/scarphase/internal/classifier/leader_follower_classifier.cpp
    source/scarphase/internal/classifier/bounded_lf_classifier.cpp
    source/scarphase/internal/classifier/unbounded_lf_classifier.cpp
    source/scarphase/internal/predictor/abstract_predictor.cpp
    source/scarphase/internal/predictor/last_value_predictor.cpp
    source/scarphase/internal/predictor/run_length_predictor.cpp)

add_library(scarphase ${SCARPHASE_LIBRARY_TYPE} ${SCARPHASE_SOURCE})

target_link_libraries(scarphase ${Boost_PROGRAM_OPTIONS_LIBRARY})

################################################################################
# Testing                                                                      #
################################################################################

if(${BUILD_UNITTESTS})

    include(ExternalProject)

    ExternalProject_Add(
        GoogleTest
        PREFIX  ${THIRD_PARTY_DIR}/GoogleTest
        DOWNLOAD_DIR ${THIRD_PARTY_DIR}/GoogleTest
        URL http://googletest.googlecode.com/files/gtest-1.6.0.zip
        URL_MD5 4577b49f2973c90bf9ba69aa8166b786
        CMAKE_ARGS
            -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY:PATH=${THIRD_PARTY_DIR}/libs/gtest
            -DCMAKE_LIBRARY_OUTPUT_DIRECTORY:PATH=${THIRD_PARTY_DIR}/libs/gtest
            -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
            -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
            -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
        INSTALL_COMMAND ""
    )

    set(GTEST_DIR "${THIRD_PARTY_DIR}/GoogleTest/src/GoogleTest")
    set(GTEST_INCLUDE_DIR "${GTEST_DIR}/include")
    set(GTEST_LIB_DIR "${THIRD_PARTY_DIR}/libs/gtest")
  
    if(DEFINED ${PARENT_DIRECTORY})

      set(GTEST_DIR "${GTEST_DIR}" PARENT_SCOPE)
      set(GTEST_INCLUDE_DIR "${GTEST_INCLUDE_DIR}" PARENT_SCOPE)
      set(GTEST_LIB_DIR "${GTEST_LIB_DIR}" PARENT_SCOPE)

    endif()

    ExternalProject_Add(
        GoogleMock
        PREFIX  ${THIRD_PARTY_DIR}/GoogleMock
        DOWNLOAD_DIR ${THIRD_PARTY_DIR}/GoogleMock
        URL http://googlemock.googlecode.com/files/gmock-1.6.0.zip
        URL_MD5 f547f47321ca88d3965ca2efdcc2a3c1
        CMAKE_ARGS
            -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY:PATH=${THIRD_PARTY_DIR}/libs/gmock
            -DCMAKE_LIBRARY_OUTPUT_DIRECTORY:PATH=${THIRD_PARTY_DIR}/libs/gmock
            -DCMAKE_TOOLCHAIN_FILE:PATH=${CMAKE_TOOLCHAIN_FILE}
            -DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}
            -DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}
        INSTALL_COMMAND ""
    )
    set(GMOCK_DIR "${THIRD_PARTY_DIR}/GoogleMock/src/GoogleMock")
    set(GMOCK_INCLUDE_DIR "${GMOCK_DIR}/include")
    set(GMOCK_LIB_DIR "${THIRD_PARTY_DIR}/libs/gmock")

    if(DEFINED ${PARENT_DIRECTORY})

      set(GMOCK_DIR "${GMOCK_DIR}" PARENT_SCOPE)
      set(GMOCK_INCLUDE_DIR "${GMOCK_INCLUDE_DIR}" PARENT_SCOPE)
      set(GMOCK_LIB_DIR "${GMOCK_LIB_DIR}" PARENT_SCOPE)

    endif()

    set(PROJECT_TEST_NAME ${PROJECT_NAME_STR}_test)

    include_directories(
        ${GTEST_INCLUDE_DIR}
        ${GMOCK_INCLUDE_DIR}
    )

    link_directories(
        ${GTEST_LIB_DIR}
        ${GMOCK_LIB_DIR}
    )

    file(GLOB_RECURSE TEST_SRC_FILES ${PROJECT_SOURCE_DIR}/test/*.cpp)

    add_executable(${PROJECT_TEST_NAME} ${TEST_SRC_FILES})

    target_link_libraries(
        ${PROJECT_TEST_NAME}
        pthread
        gtest
        gtest_main
        scarphase)

    enable_testing()
    add_test(test1 ${PROJECT_TEST_NAME})

endif(${BUILD_UNITTESTS})

################################################################################
# API Documentation                                                            #
################################################################################
if(${BUILD_API_DOCUMENTATION})

   # Find doxygen
  FIND_PACKAGE(Doxygen REQUIRED)

  # Configure the template Doxyfile
  configure_file(Doxyfile.in ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)

  # Add target
  add_custom_target(api
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)

endif(${BUILD_API_DOCUMENTATION})

