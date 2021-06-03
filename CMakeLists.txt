cmake_minimum_required(VERSION 3.13)

# test opencv exist
find_package(OpenCV REQUIRED)
# check opencv version
if(${OpenCV_VERSION} VERSION_LESS 2.0.0)
	message(FATAL_ERROR “OpenCV version is not compatible : ${OpenCV_VERSION}”)
endif()
message("opencv is up to date and ready to link")


# give a name to the project
project(WavejetProject)

# add myLib
message(STATUS "wavejet lib cmake part ..." )
add_subdirectory(wavejet INTERFACE)
include_directories(${CMAKE_SOURCE_DIR}/wavejet/include)

# add myCode
message(STATUS "wavejet tests cmake part ..." )
add_subdirectory(tests)
