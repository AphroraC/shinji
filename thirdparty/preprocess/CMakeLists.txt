cmake_minimum_required(VERSION 3.5)
project(preprocess VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE "Release")

find_package(PCL 1.10 REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_executable(preprocess preprocess.cpp)
target_include_directories(preprocess PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(preprocess
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
target_compile_options(preprocess PRIVATE ${PCL_COMPILE_OPTIONS})