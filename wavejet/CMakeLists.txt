cmake_minimum_required(VERSION 3.13)

# give a name to the project
project(Wavejet LANGUAGES C CXX)

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