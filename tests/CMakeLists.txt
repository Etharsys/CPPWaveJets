cmake_minimum_required(VERSION 3.13)

# give a name to the project
project(samples)

# check opencv version
#find_library(wave_lib ../build/INTERFACE/libWavejet.a NAMES Wavejet)
#message(STATUS "Wavejet lib : " ${wave_lib})


# collect all cpp files (sometimes considered as bad practice)
file(GLOB src_samples _folder src/*)

# for each sample file, make an exe
foreach(sample ${src_samples})

    file(GLOB_RECURSE src_files ${sample}/*.cpp)        # all src files for sample

    get_filename_component(file_exe ${sample} NAME_WE)  # define te name of the app as the sample
    add_executable(${file_exe} ${src_files})            # file to compile and name of the app
    target_link_libraries(${file_exe} PRIVATE ${OpenCV_LIBS})
    target_link_libraries(${file_exe} PRIVATE Wavejet)
    target_compile_features(${file_exe} PRIVATE cxx_std_17)
    target_compile_options(${file_exe} PRIVATE -Wall -O2 -Wextra -Werror)

    message(STATUS "exe file  " ${file_exe})
    message(STATUS "src file  " ${src_files})

endforeach()
