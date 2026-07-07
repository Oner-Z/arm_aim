CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
project(GlobalParam VERSION 1.0)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

aux_source_directory(./src ALL_SRCS)
add_library(GlobalParam STATIC ${ALL_SRCS})