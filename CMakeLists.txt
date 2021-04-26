cmake_minimum_required(VERSION 3.0.0)

project(wavejets)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(wavejets
  src/binomial/binomial.hpp
	src/main.cpp
)

###################
# Compile options #
###################

target_compile_features(wavejets PRIVATE cxx_std_17)

if(MSVC)
  target_compile_options(wavejets PRIVATE /W4 /WX)
else()
  target_compile_options(wavejets PRIVATE -Wall -Wextra -Werror -Wshadow)
endif()
