# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/maxime/Documents/C++/Stage/CPPWaveJets/tests

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/build

# Include any dependencies generated for this target.
include CMakeFiles/Kwavejet.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Kwavejet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Kwavejet.dir/flags.make

CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.o: CMakeFiles/Kwavejet.dir/flags.make
CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.o: ../src/Kwavejet/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maxime/Documents/C++/Stage/CPPWaveJets/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.o"
	/usr/bin/g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.o -c /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/src/Kwavejet/main.cpp

CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.i"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/src/Kwavejet/main.cpp > CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.i

CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.s"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/src/Kwavejet/main.cpp -o CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.s

# Object files for target Kwavejet
Kwavejet_OBJECTS = \
"CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.o"

# External object files for target Kwavejet
Kwavejet_EXTERNAL_OBJECTS =

Kwavejet: CMakeFiles/Kwavejet.dir/src/Kwavejet/main.cpp.o
Kwavejet: CMakeFiles/Kwavejet.dir/build.make
Kwavejet: CMakeFiles/Kwavejet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maxime/Documents/C++/Stage/CPPWaveJets/tests/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Kwavejet"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Kwavejet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Kwavejet.dir/build: Kwavejet

.PHONY : CMakeFiles/Kwavejet.dir/build

CMakeFiles/Kwavejet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Kwavejet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Kwavejet.dir/clean

CMakeFiles/Kwavejet.dir/depend:
	cd /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maxime/Documents/C++/Stage/CPPWaveJets/tests /home/maxime/Documents/C++/Stage/CPPWaveJets/tests /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/build /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/build /home/maxime/Documents/C++/Stage/CPPWaveJets/tests/build/CMakeFiles/Kwavejet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Kwavejet.dir/depend
