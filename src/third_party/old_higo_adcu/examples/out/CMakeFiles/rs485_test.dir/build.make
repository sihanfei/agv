# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /usr/local/adcusdk/examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/local/adcusdk/examples/out

# Include any dependencies generated for this target.
include CMakeFiles/rs485_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rs485_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rs485_test.dir/flags.make

CMakeFiles/rs485_test.dir/rs485_test.cpp.o: CMakeFiles/rs485_test.dir/flags.make
CMakeFiles/rs485_test.dir/rs485_test.cpp.o: ../rs485_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/usr/local/adcusdk/examples/out/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rs485_test.dir/rs485_test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rs485_test.dir/rs485_test.cpp.o -c /usr/local/adcusdk/examples/rs485_test.cpp

CMakeFiles/rs485_test.dir/rs485_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rs485_test.dir/rs485_test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/local/adcusdk/examples/rs485_test.cpp > CMakeFiles/rs485_test.dir/rs485_test.cpp.i

CMakeFiles/rs485_test.dir/rs485_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rs485_test.dir/rs485_test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/local/adcusdk/examples/rs485_test.cpp -o CMakeFiles/rs485_test.dir/rs485_test.cpp.s

CMakeFiles/rs485_test.dir/rs485_test.cpp.o.requires:

.PHONY : CMakeFiles/rs485_test.dir/rs485_test.cpp.o.requires

CMakeFiles/rs485_test.dir/rs485_test.cpp.o.provides: CMakeFiles/rs485_test.dir/rs485_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/rs485_test.dir/build.make CMakeFiles/rs485_test.dir/rs485_test.cpp.o.provides.build
.PHONY : CMakeFiles/rs485_test.dir/rs485_test.cpp.o.provides

CMakeFiles/rs485_test.dir/rs485_test.cpp.o.provides.build: CMakeFiles/rs485_test.dir/rs485_test.cpp.o


# Object files for target rs485_test
rs485_test_OBJECTS = \
"CMakeFiles/rs485_test.dir/rs485_test.cpp.o"

# External object files for target rs485_test
rs485_test_EXTERNAL_OBJECTS =

rs485_test: CMakeFiles/rs485_test.dir/rs485_test.cpp.o
rs485_test: CMakeFiles/rs485_test.dir/build.make
rs485_test: CMakeFiles/rs485_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/usr/local/adcusdk/examples/out/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable rs485_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rs485_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rs485_test.dir/build: rs485_test

.PHONY : CMakeFiles/rs485_test.dir/build

CMakeFiles/rs485_test.dir/requires: CMakeFiles/rs485_test.dir/rs485_test.cpp.o.requires

.PHONY : CMakeFiles/rs485_test.dir/requires

CMakeFiles/rs485_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rs485_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rs485_test.dir/clean

CMakeFiles/rs485_test.dir/depend:
	cd /usr/local/adcusdk/examples/out && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/local/adcusdk/examples /usr/local/adcusdk/examples /usr/local/adcusdk/examples/out /usr/local/adcusdk/examples/out /usr/local/adcusdk/examples/out/CMakeFiles/rs485_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rs485_test.dir/depend
