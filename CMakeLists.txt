cmake_minimum_required(VERSION 3.22.1)

project(HighwayBenchmark)

set(CMAKE_BUILD_TYPE Release)

find_package(hwy REQUIRED)
find_package(benchmark REQUIRED)

add_compile_options(-O2 -march=native)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(main main.cpp)
add_executable(benchmark benchmark.cpp)

target_link_libraries(main hwy::hwy)
target_link_libraries(benchmark hwy::hwy benchmark::benchmark)