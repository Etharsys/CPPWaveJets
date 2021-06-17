cmake_minimum_required(VERSION 3.13)


# include eigen
find_package(Eigen3 REQUIRED)
if(${EIGEN3_FOUND})
    message(STATUS "lib EIGEN3 found")
    message(STATUS "  version " ${EIGEN3_VERSION_STRING})
    message(STATUS "  include " ${EIGEN3_INCLUDE_DIR})
else()
    include_directories("/usr/include/eigen3") # manually specify the include location
endif()



# give a name to the project
project(samples)

# collect all cpp files
file(GLOB src_samples _folder src/*)
file(GLOB inc_random  randomizer.hpp)

# for each sample file, make an exe
foreach(sample ${src_samples})

    file(GLOB_RECURSE src_files ${sample}/*.cpp)             # all src files for sample

    get_filename_component(file_exe ${sample} NAME_WE)       # define te name of the app as the sample
    add_executable(${file_exe} randomizer.hpp ${src_files})  # file to compile and name of the app
    target_link_libraries(${file_exe} PRIVATE ${OpenCV_LIBS})
    target_link_libraries(${file_exe} PRIVATE Wavejet)
    target_compile_features(${file_exe} PRIVATE cxx_std_17 )
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
    target_compile_options(${file_exe} PRIVATE -Wall -O2 -Wextra -Werror)
    target_include_directories(${file_exe} PRIVATE ${EIGEN3_INCLUDE_DIR})

    message(STATUS "exe file  " ${file_exe})
    message(STATUS "src file  " ${src_files})

endforeach()
