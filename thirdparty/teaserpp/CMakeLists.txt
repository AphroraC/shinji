cmake_minimum_required(VERSION 3.10)
project(teaserpp VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})
set(TEASERPP_DEPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party" CACHE PATH "Directory for third-party dependencies")

set(CMAKE_BUILD_TYPE "Release")

# Options
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_TEASER_FPFH "Build TEASER++ wrappers for PCL FPFH estimation." ON)
option(BUILD_MATLAB_BINDINGS "Build MATLAB bindings" OFF)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(BUILD_DOC "Build documentation" OFF)
option(BUILD_WITH_MARCH_NATIVE "Build with flag march=native" OFF)
option(ENABLE_MKL "Try to use Eigen with MKL" ON)
option(ENABLE_DIAGNOSTIC_PRINT "Enable printing of diagnostic messages" OFF)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (ENABLE_DIAGNOSTIC_PRINT)
    message(STATUS "Enable printing of diagnostic messages.")
    add_definitions(-DTEASER_DIAG_PRINT)
endif ()

# Cache Variables
if (NOT TEASERPP_PYTHON_VERSION)
    set(TEASERPP_PYTHON_VERSION "" CACHE STRING "Python version to use for TEASER++ bindings.")
endif ()

# Find dependencies
# Eigen3
find_package(Eigen3 3.2 QUIET REQUIRED NO_MODULE)
if (ENABLE_MKL)
    find_package(MKL)
    if (MKL_FOUND)
        message(STATUS "MKL found at: ${MKL_LIBRARIES}")
        include_directories(${MKL_INCLUDE_DIR})
        add_definitions(-DEIGEN_USE_MKL_ALL)
        list(APPEND TEASERPP_BLAS_LAPACK_LIBS ${MKL_LIBRARIES})
    else ()
        message(STATUS "MKL not found.")
    endif ()
endif ()

if (BUILD_TEASER_FPFH)
    find_package(Boost 1.58 QUIET REQUIRED)
    find_package(PCL 1.8 QUIET REQUIRED COMPONENTS common io features kdtree)
endif ()

# pmc (Parallel Maximum Clique)
if(NOT EXISTS "${TEASERPP_DEPS_DIR}/pmc")
    message(STATUS "Downloading PMC...")
    configure_file(cmake/pmc.CMakeLists.txt.in pmc-download/CMakeLists.txt)
    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pmc-download")
    execute_process(COMMAND "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pmc-download")
    file(COPY "${CMAKE_BINARY_DIR}/pmc-src/" DESTINATION "${TEASERPP_DEPS_DIR}/pmc")
endif()
add_subdirectory("${TEASERPP_DEPS_DIR}/pmc" "${CMAKE_BINARY_DIR}/pmc-build")

# tinyply
if(NOT EXISTS "${TEASERPP_DEPS_DIR}/tinyply")
    message(STATUS "Downloading tinyply...")
    configure_file(cmake/tinyply.CMakeLists.txt.in tinyply-download/CMakeLists.txt)
    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tinyply-download")
    execute_process(COMMAND "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/tinyply-download")
    file(COPY "${CMAKE_BINARY_DIR}/tinyply-src/" DESTINATION "${TEASERPP_DEPS_DIR}/tinyply")
endif()
add_subdirectory("${TEASERPP_DEPS_DIR}/tinyply" "${CMAKE_BINARY_DIR}/tinyply-build")
target_include_directories(tinyply PUBLIC
        $<BUILD_INTERFACE:${TEASERPP_DEPS_DIR}/tinyply/source>)

# spectra
if(NOT EXISTS "${TEASERPP_DEPS_DIR}/spectra")
    message(STATUS "Downloading spectra...")
    configure_file(cmake/spectra.CMakeLists.txt.in spectra-download/CMakeLists.txt)
    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/spectra-download")
    execute_process(COMMAND "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/spectra-download")
    file(COPY "${CMAKE_BINARY_DIR}/spectra-src/" DESTINATION "${TEASERPP_DEPS_DIR}/spectra")
endif()
set(SPECTRA_INCLUDE_DIRS "${TEASERPP_DEPS_DIR}/spectra/include")

# Building Targets
set(TEASERPP_ROOT ${CMAKE_CURRENT_LIST_DIR})
add_subdirectory(teaser)

if (BUILD_TESTS)
# googletest
        if(NOT EXISTS "${CMAKE_BINARY_DIR}/googletest-src")
         message(STATUS "Downloading GoogleTest...")
            configure_file(cmake/GoogleTest.CMakeLists.txt.in googletest-download/CMakeLists.txt)
            execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
                WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download")
            execute_process(COMMAND "${CMAKE_COMMAND}" --build .
                WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-download")
            file(COPY "${CMAKE_BINARY_DIR}/googletest-src/" DESTINATION "${CMAKE_BINARY_DIR}/googletest-src")
        endif()
set(GTEST_INCLUDE_DIRS "${CMAKE_BINARY_DIR}/googletest-src/googletest/include"
        "${CMAKE_BINARY_DIR}/googletest-src/googlemock/include")

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
        "${CMAKE_BINARY_DIR}/googletest-build")

    enable_testing()
    add_subdirectory(test)
endif ()

if (BUILD_DOC)
    add_subdirectory(doc EXCLUDE_FROM_ALL)
endif ()

if (BUILD_MATLAB_BINDINGS)
    message(STATUS "Will build MATLAB bindings.")
    add_subdirectory(matlab)
endif ()

if (BUILD_PYTHON_BINDINGS)
    set(PYBIND11_PYTHON_VERSION ${TEASERPP_PYTHON_VERSION})

    # download the pybind11 repo
    configure_file(cmake/pybind11.CMakeLists.txt.in pybind11-download/CMakeLists.txt)
    execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pybind11-download")
    execute_process(COMMAND "${CMAKE_COMMAND}" --build .
            WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/pybind11-download")
    add_subdirectory("${CMAKE_BINARY_DIR}/pybind11-src"
            "${CMAKE_BINARY_DIR}/pybind11-build")

    message(STATUS "TEASER++ Python binding will be built.")
    add_subdirectory(python)
endif ()

catkin_package(
  INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}
  LIBRARIES pmc teaser_registration teaser_features teaser_io
  DEPENDS Eigen3 PCL
)

add_custom_target(copy_teaserpp_libs ALL
  COMMENT "Copying TEASER++ libraries to devel/lib"
)

add_custom_command(TARGET copy_teaserpp_libs POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:teaser_registration>"
    "${CATKIN_DEVEL_PREFIX}/lib/$<TARGET_FILE_NAME:teaser_registration>"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:teaser_features>"
    "${CATKIN_DEVEL_PREFIX}/lib/$<TARGET_FILE_NAME:teaser_features>"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:teaser_io>"
    "${CATKIN_DEVEL_PREFIX}/lib/$<TARGET_FILE_NAME:teaser_io>"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "$<TARGET_FILE:pmc>"
    "${CATKIN_DEVEL_PREFIX}/lib/$<TARGET_FILE_NAME:pmc>"
  DEPENDS teaser_registration teaser_features teaser_io pmc
)

add_dependencies(copy_teaserpp_libs teaser_registration teaser_features teaser_io pmc)


# export targets
# export(TARGETS ${TEASERPP_EXPORTED_TARGETS} FILE teaserpp-exports.cmake)

# install(FILES cmake/teaserppConfig.cmake
#        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/teaserpp
#        )
