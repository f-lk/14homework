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
CMAKE_COMMAND = /home/flk/yuanma/clion-2019.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/flk/yuanma/clion-2019.1.3/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/flk/桌面/课件/第03讲/xiezuoye

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/flk/桌面/课件/第03讲/xiezuoye/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/tt.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/tt.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/tt.dir/flags.make

CMakeFiles/tt.dir/draw_trajectory.cpp.o: CMakeFiles/tt.dir/flags.make
CMakeFiles/tt.dir/draw_trajectory.cpp.o: ../draw_trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/flk/桌面/课件/第03讲/xiezuoye/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/tt.dir/draw_trajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tt.dir/draw_trajectory.cpp.o -c /home/flk/桌面/课件/第03讲/xiezuoye/draw_trajectory.cpp

CMakeFiles/tt.dir/draw_trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tt.dir/draw_trajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/flk/桌面/课件/第03讲/xiezuoye/draw_trajectory.cpp > CMakeFiles/tt.dir/draw_trajectory.cpp.i

CMakeFiles/tt.dir/draw_trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tt.dir/draw_trajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/flk/桌面/课件/第03讲/xiezuoye/draw_trajectory.cpp -o CMakeFiles/tt.dir/draw_trajectory.cpp.s

# Object files for target tt
tt_OBJECTS = \
"CMakeFiles/tt.dir/draw_trajectory.cpp.o"

# External object files for target tt
tt_EXTERNAL_OBJECTS =

tt: CMakeFiles/tt.dir/draw_trajectory.cpp.o
tt: CMakeFiles/tt.dir/build.make
tt: /home/flk/yuanma/slambook-master/3rdparty/Sophus/build/libSophus.so
tt: /home/flk/yuanma/slambook-master/3rdparty/Pangolin/build/src/libpangolin.so
tt: /usr/lib/x86_64-linux-gnu/libGL.so
tt: /usr/lib/x86_64-linux-gnu/libGLU.so
tt: /usr/lib/x86_64-linux-gnu/libGLEW.so
tt: /usr/lib/x86_64-linux-gnu/libpython2.7.so
tt: CMakeFiles/tt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/flk/桌面/课件/第03讲/xiezuoye/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable tt"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/tt.dir/build: tt

.PHONY : CMakeFiles/tt.dir/build

CMakeFiles/tt.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/tt.dir/cmake_clean.cmake
.PHONY : CMakeFiles/tt.dir/clean

CMakeFiles/tt.dir/depend:
	cd /home/flk/桌面/课件/第03讲/xiezuoye/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/flk/桌面/课件/第03讲/xiezuoye /home/flk/桌面/课件/第03讲/xiezuoye /home/flk/桌面/课件/第03讲/xiezuoye/cmake-build-debug /home/flk/桌面/课件/第03讲/xiezuoye/cmake-build-debug /home/flk/桌面/课件/第03讲/xiezuoye/cmake-build-debug/CMakeFiles/tt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/tt.dir/depend

