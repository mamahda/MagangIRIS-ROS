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
CMAKE_SOURCE_DIR = /home/gilbran/MagangIRIS/MagangIRIS-ROS/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gilbran/MagangIRIS/MagangIRIS-ROS/build

# Include any dependencies generated for this target.
include nama/CMakeFiles/i.dir/depend.make

# Include the progress variables for this target.
include nama/CMakeFiles/i.dir/progress.make

# Include the compile flags for this target's objects.
include nama/CMakeFiles/i.dir/flags.make

nama/CMakeFiles/i.dir/src/I2.cpp.o: nama/CMakeFiles/i.dir/flags.make
nama/CMakeFiles/i.dir/src/I2.cpp.o: /home/gilbran/MagangIRIS/MagangIRIS-ROS/src/nama/src/I2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gilbran/MagangIRIS/MagangIRIS-ROS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object nama/CMakeFiles/i.dir/src/I2.cpp.o"
	cd /home/gilbran/MagangIRIS/MagangIRIS-ROS/build/nama && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/i.dir/src/I2.cpp.o -c /home/gilbran/MagangIRIS/MagangIRIS-ROS/src/nama/src/I2.cpp

nama/CMakeFiles/i.dir/src/I2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/i.dir/src/I2.cpp.i"
	cd /home/gilbran/MagangIRIS/MagangIRIS-ROS/build/nama && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gilbran/MagangIRIS/MagangIRIS-ROS/src/nama/src/I2.cpp > CMakeFiles/i.dir/src/I2.cpp.i

nama/CMakeFiles/i.dir/src/I2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/i.dir/src/I2.cpp.s"
	cd /home/gilbran/MagangIRIS/MagangIRIS-ROS/build/nama && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gilbran/MagangIRIS/MagangIRIS-ROS/src/nama/src/I2.cpp -o CMakeFiles/i.dir/src/I2.cpp.s

# Object files for target i
i_OBJECTS = \
"CMakeFiles/i.dir/src/I2.cpp.o"

# External object files for target i
i_EXTERNAL_OBJECTS =

/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: nama/CMakeFiles/i.dir/src/I2.cpp.o
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: nama/CMakeFiles/i.dir/build.make
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/libroscpp.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/librosconsole.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/librostime.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /opt/ros/noetic/lib/libcpp_common.so
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i: nama/CMakeFiles/i.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gilbran/MagangIRIS/MagangIRIS-ROS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i"
	cd /home/gilbran/MagangIRIS/MagangIRIS-ROS/build/nama && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/i.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
nama/CMakeFiles/i.dir/build: /home/gilbran/MagangIRIS/MagangIRIS-ROS/devel/lib/nama/i

.PHONY : nama/CMakeFiles/i.dir/build

nama/CMakeFiles/i.dir/clean:
	cd /home/gilbran/MagangIRIS/MagangIRIS-ROS/build/nama && $(CMAKE_COMMAND) -P CMakeFiles/i.dir/cmake_clean.cmake
.PHONY : nama/CMakeFiles/i.dir/clean

nama/CMakeFiles/i.dir/depend:
	cd /home/gilbran/MagangIRIS/MagangIRIS-ROS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gilbran/MagangIRIS/MagangIRIS-ROS/src /home/gilbran/MagangIRIS/MagangIRIS-ROS/src/nama /home/gilbran/MagangIRIS/MagangIRIS-ROS/build /home/gilbran/MagangIRIS/MagangIRIS-ROS/build/nama /home/gilbran/MagangIRIS/MagangIRIS-ROS/build/nama/CMakeFiles/i.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : nama/CMakeFiles/i.dir/depend

