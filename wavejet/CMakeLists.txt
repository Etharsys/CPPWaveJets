cmake_minimum_required(VERSION 3.13)

# give a name to the project
project(Wavejet LANGUAGES C CXX)

# include eigen
find_package(Eigen3 REQUIRED)
if(${EIGEN3_FOUND})
    message(STATUS "lib EIGEN3 found")
    message(STATUS "  version " ${EIGEN3_VERSION_STRING})
    message(STATUS "  include " ${EIGEN3_INCLUDE_DIR})
else()
    include_directories("/usr/include/eigen3") # manually specify the include location
endif()


# automatic get all files in a directory
file(GLOB_RECURSE source_files src/*.cpp)
file(GLOB_RECURSE header_files include/*.hpp)

# add_library(ganQ SHARED ${source_files} ${header_files})
add_library(Wavejet ${source_files} ${header_files})

# link opencv library
target_link_libraries(Wavejet ${OpenCV_LIBS})

# compilation flags
target_compile_features(Wavejet PRIVATE cxx_std_17) 
target_compile_options(Wavejet PRIVATE -Wall -Wextra -Werror)

# include directory
target_include_directories(Wavejet PRIVATE "include")
target_include_directories(Wavejet PRIVATE ${EIGEN3_INCLUDE_DIR})