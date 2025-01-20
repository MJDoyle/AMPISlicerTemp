cmake_minimum_required(VERSION 2.8.12)
project(assembl3r)

add_library(assembl3r STATIC
	Assembler.hpp
	Assembler.cpp
)

target_include_directories(assembl3r PUBLIC .)

target_link_libraries(assembl3r PRIVATE libslic3r)
