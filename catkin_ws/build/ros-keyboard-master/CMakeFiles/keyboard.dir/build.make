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
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/build

# Include any dependencies generated for this target.
include ros-keyboard-master/CMakeFiles/keyboard.dir/depend.make

# Include the progress variables for this target.
include ros-keyboard-master/CMakeFiles/keyboard.dir/progress.make

# Include the compile flags for this target's objects.
include ros-keyboard-master/CMakeFiles/keyboard.dir/flags.make

ros-keyboard-master/CMakeFiles/keyboard.dir/src/main.cpp.o: ros-keyboard-master/CMakeFiles/keyboard.dir/flags.make
ros-keyboard-master/CMakeFiles/keyboard.dir/src/main.cpp.o: /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ros-keyboard-master/CMakeFiles/keyboard.dir/src/main.cpp.o"
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/src/main.cpp.o -c /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/main.cpp

ros-keyboard-master/CMakeFiles/keyboard.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/main.cpp.i"
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/main.cpp > CMakeFiles/keyboard.dir/src/main.cpp.i

ros-keyboard-master/CMakeFiles/keyboard.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/main.cpp.s"
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/main.cpp -o CMakeFiles/keyboard.dir/src/main.cpp.s

ros-keyboard-master/CMakeFiles/keyboard.dir/src/keyboard.cpp.o: ros-keyboard-master/CMakeFiles/keyboard.dir/flags.make
ros-keyboard-master/CMakeFiles/keyboard.dir/src/keyboard.cpp.o: /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/keyboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ros-keyboard-master/CMakeFiles/keyboard.dir/src/keyboard.cpp.o"
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/src/keyboard.cpp.o -c /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/keyboard.cpp

ros-keyboard-master/CMakeFiles/keyboard.dir/src/keyboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/keyboard.cpp.i"
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/keyboard.cpp > CMakeFiles/keyboard.dir/src/keyboard.cpp.i

ros-keyboard-master/CMakeFiles/keyboard.dir/src/keyboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/keyboard.cpp.s"
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/ros-keyboard-master/src/keyboard.cpp -o CMakeFiles/keyboard.dir/src/keyboard.cpp.s

# Object files for target keyboard
keyboard_OBJECTS = \
"CMakeFiles/keyboard.dir/src/main.cpp.o" \
"CMakeFiles/keyboard.dir/src/keyboard.cpp.o"

# External object files for target keyboard
keyboard_EXTERNAL_OBJECTS =

/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard-master/CMakeFiles/keyboard.dir/src/main.cpp.o
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard-master/CMakeFiles/keyboard.dir/src/keyboard.cpp.o
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard-master/CMakeFiles/keyboard.dir/build.make
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libSDLmain.a
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libSDL.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/libroscpp.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/librosconsole.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.71.0
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/librostime.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.71.0
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/noetic/lib/libcpp_common.so
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.71.0
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard-master/CMakeFiles/keyboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard"
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keyboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ros-keyboard-master/CMakeFiles/keyboard.dir/build: /home/ubuntu/catkin_ws/devel/lib/keyboard/keyboard

.PHONY : ros-keyboard-master/CMakeFiles/keyboard.dir/build

ros-keyboard-master/CMakeFiles/keyboard.dir/clean:
	cd /home/ubuntu/catkin_ws/build/ros-keyboard-master && $(CMAKE_COMMAND) -P CMakeFiles/keyboard.dir/cmake_clean.cmake
.PHONY : ros-keyboard-master/CMakeFiles/keyboard.dir/clean

ros-keyboard-master/CMakeFiles/keyboard.dir/depend:
	cd /home/ubuntu/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src /home/ubuntu/catkin_ws/src/ros-keyboard-master /home/ubuntu/catkin_ws/build /home/ubuntu/catkin_ws/build/ros-keyboard-master /home/ubuntu/catkin_ws/build/ros-keyboard-master/CMakeFiles/keyboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ros-keyboard-master/CMakeFiles/keyboard.dir/depend

