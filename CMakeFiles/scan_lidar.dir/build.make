# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.14.5/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.14.5/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/jae/Desktop/lidar/lidar_implementation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/jae/Desktop/lidar/lidar_implementation

# Include any dependencies generated for this target.
include CMakeFiles/scan_lidar.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/scan_lidar.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/scan_lidar.dir/flags.make

CMakeFiles/scan_lidar.dir/sample.cpp.o: CMakeFiles/scan_lidar.dir/flags.make
CMakeFiles/scan_lidar.dir/sample.cpp.o: sample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/jae/Desktop/lidar/lidar_implementation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/scan_lidar.dir/sample.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/scan_lidar.dir/sample.cpp.o -c /Users/jae/Desktop/lidar/lidar_implementation/sample.cpp

CMakeFiles/scan_lidar.dir/sample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scan_lidar.dir/sample.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/jae/Desktop/lidar/lidar_implementation/sample.cpp > CMakeFiles/scan_lidar.dir/sample.cpp.i

CMakeFiles/scan_lidar.dir/sample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scan_lidar.dir/sample.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/jae/Desktop/lidar/lidar_implementation/sample.cpp -o CMakeFiles/scan_lidar.dir/sample.cpp.s

# Object files for target scan_lidar
scan_lidar_OBJECTS = \
"CMakeFiles/scan_lidar.dir/sample.cpp.o"

# External object files for target scan_lidar
scan_lidar_EXTERNAL_OBJECTS =

scan_lidar: CMakeFiles/scan_lidar.dir/sample.cpp.o
scan_lidar: CMakeFiles/scan_lidar.dir/build.make
scan_lidar: CMakeFiles/scan_lidar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/jae/Desktop/lidar/lidar_implementation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable scan_lidar"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scan_lidar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/scan_lidar.dir/build: scan_lidar

.PHONY : CMakeFiles/scan_lidar.dir/build

CMakeFiles/scan_lidar.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/scan_lidar.dir/cmake_clean.cmake
.PHONY : CMakeFiles/scan_lidar.dir/clean

CMakeFiles/scan_lidar.dir/depend:
	cd /Users/jae/Desktop/lidar/lidar_implementation && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/jae/Desktop/lidar/lidar_implementation /Users/jae/Desktop/lidar/lidar_implementation /Users/jae/Desktop/lidar/lidar_implementation /Users/jae/Desktop/lidar/lidar_implementation /Users/jae/Desktop/lidar/lidar_implementation/CMakeFiles/scan_lidar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/scan_lidar.dir/depend

