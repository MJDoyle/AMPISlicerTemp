cmake_minimum_required(VERSION 2.8.12)
project(assembl3r)

include(FetchContent) #added

FetchContent_Declare(
	yaml-cpp
	GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
	GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)

add_library(assembl3r STATIC
	Assembler.hpp
	Assembler.cpp
)

target_include_directories(assembl3r PUBLIC .)

target_link_libraries(assembl3r PUBLIC yaml-cpp::yaml-cpp)

target_link_libraries(assembl3r PRIVATE libslic3r)
