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
CMAKE_SOURCE_DIR = /home/liam/git/CppExercises/circlefitting

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liam/git/CppExercises/circlefitting/build

# Include any dependencies generated for this target.
include CMakeFiles/TestAlgebraicCircleFits.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TestAlgebraicCircleFits.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TestAlgebraicCircleFits.dir/flags.make

CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.o: CMakeFiles/TestAlgebraicCircleFits.dir/flags.make
CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.o: ../TestAlgebraicCircleFits.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liam/git/CppExercises/circlefitting/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.o -c /home/liam/git/CppExercises/circlefitting/TestAlgebraicCircleFits.cpp

CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liam/git/CppExercises/circlefitting/TestAlgebraicCircleFits.cpp > CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.i

CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liam/git/CppExercises/circlefitting/TestAlgebraicCircleFits.cpp -o CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.s

# Object files for target TestAlgebraicCircleFits
TestAlgebraicCircleFits_OBJECTS = \
"CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.o"

# External object files for target TestAlgebraicCircleFits
TestAlgebraicCircleFits_EXTERNAL_OBJECTS =

TestAlgebraicCircleFits: CMakeFiles/TestAlgebraicCircleFits.dir/TestAlgebraicCircleFits.cpp.o
TestAlgebraicCircleFits: CMakeFiles/TestAlgebraicCircleFits.dir/build.make
TestAlgebraicCircleFits: libCircleFitByLeastSquare_library.a
TestAlgebraicCircleFits: libCircleFitByHyper_library.a
TestAlgebraicCircleFits: libCircleFitByKasa_library.a
TestAlgebraicCircleFits: libCircleFitByPratt_library.a
TestAlgebraicCircleFits: libCircleFitByTaubin_library.a
TestAlgebraicCircleFits: CMakeFiles/TestAlgebraicCircleFits.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liam/git/CppExercises/circlefitting/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable TestAlgebraicCircleFits"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TestAlgebraicCircleFits.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TestAlgebraicCircleFits.dir/build: TestAlgebraicCircleFits

.PHONY : CMakeFiles/TestAlgebraicCircleFits.dir/build

CMakeFiles/TestAlgebraicCircleFits.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TestAlgebraicCircleFits.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TestAlgebraicCircleFits.dir/clean

CMakeFiles/TestAlgebraicCircleFits.dir/depend:
	cd /home/liam/git/CppExercises/circlefitting/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liam/git/CppExercises/circlefitting /home/liam/git/CppExercises/circlefitting /home/liam/git/CppExercises/circlefitting/build /home/liam/git/CppExercises/circlefitting/build /home/liam/git/CppExercises/circlefitting/build/CMakeFiles/TestAlgebraicCircleFits.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TestAlgebraicCircleFits.dir/depend

