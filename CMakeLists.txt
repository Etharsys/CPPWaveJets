cmake_minimum_required(VERSION 3.0.0)

project(wavejets)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(OpenCV REQUIRED)

# check opencv version
if(${OpenCV_VERSION} VERSION_LESS 2.0.0)
	message(FATAL_ERROR “OpenCV version is not compatible : ${OpenCV_VERSION}”)
endif()


add_executable(wavejets
  src/wavejet.cpp
  src/CloudDots.cpp
  src/CylinderCloud.cpp
  src/wavejetDisplay.cpp
  src/main.cpp
)

# link library
target_link_libraries(wavejets ${OpenCV_LIBS})

# Compile options
target_compile_features(wavejets PRIVATE cxx_std_17)

if(MSVC)
  target_compile_options(wavejets PRIVATE /W4 /WX)
else()
  target_compile_options(wavejets PRIVATE -Wall -Wextra -Werror -Wshadow)
endif()
