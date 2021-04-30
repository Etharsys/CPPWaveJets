cmake_minimum_required(VERSION 3.0.0)

project(wavejets)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_executable(wavejets
  src/operation/binomial.hpp
	src/operation/factorial.hpp
  src/operation/misc.hpp
  src/wavejet.hpp
  src/wavejet.cpp
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
