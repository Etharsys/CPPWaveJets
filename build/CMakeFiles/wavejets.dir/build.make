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


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/maxime/Documents/C++/Stage/CPPWaveJets

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maxime/Documents/C++/Stage/CPPWaveJets/build

# Include any dependencies generated for this target.
include CMakeFiles/wavejets.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/wavejets.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/wavejets.dir/flags.make

CMakeFiles/wavejets.dir/src/wavejet.cpp.o: CMakeFiles/wavejets.dir/flags.make
CMakeFiles/wavejets.dir/src/wavejet.cpp.o: ../src/wavejet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maxime/Documents/C++/Stage/CPPWaveJets/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/wavejets.dir/src/wavejet.cpp.o"
	/usr/bin/g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/wavejets.dir/src/wavejet.cpp.o -c /home/maxime/Documents/C++/Stage/CPPWaveJets/src/wavejet.cpp

CMakeFiles/wavejets.dir/src/wavejet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/wavejets.dir/src/wavejet.cpp.i"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maxime/Documents/C++/Stage/CPPWaveJets/src/wavejet.cpp > CMakeFiles/wavejets.dir/src/wavejet.cpp.i

CMakeFiles/wavejets.dir/src/wavejet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/wavejets.dir/src/wavejet.cpp.s"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maxime/Documents/C++/Stage/CPPWaveJets/src/wavejet.cpp -o CMakeFiles/wavejets.dir/src/wavejet.cpp.s

CMakeFiles/wavejets.dir/src/main.cpp.o: CMakeFiles/wavejets.dir/flags.make
CMakeFiles/wavejets.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maxime/Documents/C++/Stage/CPPWaveJets/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/wavejets.dir/src/main.cpp.o"
	/usr/bin/g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/wavejets.dir/src/main.cpp.o -c /home/maxime/Documents/C++/Stage/CPPWaveJets/src/main.cpp

CMakeFiles/wavejets.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/wavejets.dir/src/main.cpp.i"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maxime/Documents/C++/Stage/CPPWaveJets/src/main.cpp > CMakeFiles/wavejets.dir/src/main.cpp.i

CMakeFiles/wavejets.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/wavejets.dir/src/main.cpp.s"
	/usr/bin/g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maxime/Documents/C++/Stage/CPPWaveJets/src/main.cpp -o CMakeFiles/wavejets.dir/src/main.cpp.s

# Object files for target wavejets
wavejets_OBJECTS = \
"CMakeFiles/wavejets.dir/src/wavejet.cpp.o" \
"CMakeFiles/wavejets.dir/src/main.cpp.o"

# External object files for target wavejets
wavejets_EXTERNAL_OBJECTS =

wavejets: CMakeFiles/wavejets.dir/src/wavejet.cpp.o
wavejets: CMakeFiles/wavejets.dir/src/main.cpp.o
wavejets: CMakeFiles/wavejets.dir/build.make
wavejets: CMakeFiles/wavejets.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maxime/Documents/C++/Stage/CPPWaveJets/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable wavejets"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/wavejets.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/wavejets.dir/build: wavejets

.PHONY : CMakeFiles/wavejets.dir/build

CMakeFiles/wavejets.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/wavejets.dir/cmake_clean.cmake
.PHONY : CMakeFiles/wavejets.dir/clean

CMakeFiles/wavejets.dir/depend:
	cd /home/maxime/Documents/C++/Stage/CPPWaveJets/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maxime/Documents/C++/Stage/CPPWaveJets /home/maxime/Documents/C++/Stage/CPPWaveJets /home/maxime/Documents/C++/Stage/CPPWaveJets/build /home/maxime/Documents/C++/Stage/CPPWaveJets/build /home/maxime/Documents/C++/Stage/CPPWaveJets/build/CMakeFiles/wavejets.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/wavejets.dir/depend

