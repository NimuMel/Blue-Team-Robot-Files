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
CMAKE_SOURCE_DIR = /home/blue_team/Blue-Team-Robot-Files/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build

# Include any dependencies generated for this target.
include master/CMakeFiles/xbox_controller_node.dir/depend.make

# Include the progress variables for this target.
include master/CMakeFiles/xbox_controller_node.dir/progress.make

# Include the compile flags for this target's objects.
include master/CMakeFiles/xbox_controller_node.dir/flags.make

master/CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.o: master/CMakeFiles/xbox_controller_node.dir/flags.make
master/CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.o: /home/blue_team/Blue-Team-Robot-Files/catkin_ws/src/master/src/xbox_controller_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object master/CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.o"
	cd /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/master && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.o -c /home/blue_team/Blue-Team-Robot-Files/catkin_ws/src/master/src/xbox_controller_node.cpp

master/CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.i"
	cd /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/blue_team/Blue-Team-Robot-Files/catkin_ws/src/master/src/xbox_controller_node.cpp > CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.i

master/CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.s"
	cd /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/blue_team/Blue-Team-Robot-Files/catkin_ws/src/master/src/xbox_controller_node.cpp -o CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.s

# Object files for target xbox_controller_node
xbox_controller_node_OBJECTS = \
"CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.o"

# External object files for target xbox_controller_node
xbox_controller_node_EXTERNAL_OBJECTS =

/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: master/CMakeFiles/xbox_controller_node.dir/src/xbox_controller_node.cpp.o
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: master/CMakeFiles/xbox_controller_node.dir/build.make
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/libroscpp.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/librosconsole.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/librostime.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /opt/ros/noetic/lib/libcpp_common.so
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node: master/CMakeFiles/xbox_controller_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node"
	cd /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/xbox_controller_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
master/CMakeFiles/xbox_controller_node.dir/build: /home/blue_team/Blue-Team-Robot-Files/catkin_ws/devel/lib/master/xbox_controller_node

.PHONY : master/CMakeFiles/xbox_controller_node.dir/build

master/CMakeFiles/xbox_controller_node.dir/clean:
	cd /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/master && $(CMAKE_COMMAND) -P CMakeFiles/xbox_controller_node.dir/cmake_clean.cmake
.PHONY : master/CMakeFiles/xbox_controller_node.dir/clean

master/CMakeFiles/xbox_controller_node.dir/depend:
	cd /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/blue_team/Blue-Team-Robot-Files/catkin_ws/src /home/blue_team/Blue-Team-Robot-Files/catkin_ws/src/master /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/master /home/blue_team/Blue-Team-Robot-Files/catkin_ws/build/master/CMakeFiles/xbox_controller_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : master/CMakeFiles/xbox_controller_node.dir/depend

