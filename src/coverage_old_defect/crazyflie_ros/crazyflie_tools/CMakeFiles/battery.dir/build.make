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
CMAKE_SOURCE_DIR = /home/qingchen/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/qingchen/catkin_ws/src/coverage

# Include any dependencies generated for this target.
include crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/depend.make

# Include the progress variables for this target.
include crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/progress.make

# Include the compile flags for this target's objects.
include crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/flags.make

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o: crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/flags.make
crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o: ../crazyflie_ros/crazyflie_tools/src/battery.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/qingchen/catkin_ws/src/coverage/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o"
	cd /home/qingchen/catkin_ws/src/coverage/crazyflie_ros/crazyflie_tools && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/battery.dir/src/battery.cpp.o -c /home/qingchen/catkin_ws/src/crazyflie_ros/crazyflie_tools/src/battery.cpp

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/battery.dir/src/battery.cpp.i"
	cd /home/qingchen/catkin_ws/src/coverage/crazyflie_ros/crazyflie_tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/qingchen/catkin_ws/src/crazyflie_ros/crazyflie_tools/src/battery.cpp > CMakeFiles/battery.dir/src/battery.cpp.i

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/battery.dir/src/battery.cpp.s"
	cd /home/qingchen/catkin_ws/src/coverage/crazyflie_ros/crazyflie_tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/qingchen/catkin_ws/src/crazyflie_ros/crazyflie_tools/src/battery.cpp -o CMakeFiles/battery.dir/src/battery.cpp.s

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.requires:

.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.requires

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.provides: crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.requires
	$(MAKE) -f crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/build.make crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.provides.build
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.provides

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.provides.build: crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o


# Object files for target battery
battery_OBJECTS = \
"CMakeFiles/battery.dir/src/battery.cpp.o"

# External object files for target battery
battery_EXTERNAL_OBJECTS =

devel/lib/crazyflie_tools/battery: crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o
devel/lib/crazyflie_tools/battery: crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/build.make
devel/lib/crazyflie_tools/battery: devel/lib/libcrazyflie_cpp.so
devel/lib/crazyflie_tools/battery: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/crazyflie_tools/battery: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
devel/lib/crazyflie_tools/battery: crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/qingchen/catkin_ws/src/coverage/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../devel/lib/crazyflie_tools/battery"
	cd /home/qingchen/catkin_ws/src/coverage/crazyflie_ros/crazyflie_tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/battery.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/build: devel/lib/crazyflie_tools/battery

.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/build

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/requires: crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o.requires

.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/requires

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/clean:
	cd /home/qingchen/catkin_ws/src/coverage/crazyflie_ros/crazyflie_tools && $(CMAKE_COMMAND) -P CMakeFiles/battery.dir/cmake_clean.cmake
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/clean

crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/depend:
	cd /home/qingchen/catkin_ws/src/coverage && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/qingchen/catkin_ws/src /home/qingchen/catkin_ws/src/crazyflie_ros/crazyflie_tools /home/qingchen/catkin_ws/src/coverage /home/qingchen/catkin_ws/src/coverage/crazyflie_ros/crazyflie_tools /home/qingchen/catkin_ws/src/coverage/crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyflie_ros/crazyflie_tools/CMakeFiles/battery.dir/depend

