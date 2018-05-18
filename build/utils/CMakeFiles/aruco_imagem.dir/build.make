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
CMAKE_SOURCE_DIR = /home/alantavares/aruco

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alantavares/aruco/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_imagem.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_imagem.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_imagem.dir/flags.make

utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o: utils/CMakeFiles/aruco_imagem.dir/flags.make
utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o: ../utils/aruco_imagem.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alantavares/aruco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o"
	cd /home/alantavares/aruco/build/utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o -c /home/alantavares/aruco/utils/aruco_imagem.cpp

utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.i"
	cd /home/alantavares/aruco/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alantavares/aruco/utils/aruco_imagem.cpp > CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.i

utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.s"
	cd /home/alantavares/aruco/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alantavares/aruco/utils/aruco_imagem.cpp -o CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.s

utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.requires:

.PHONY : utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.requires

utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.provides: utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_imagem.dir/build.make utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.provides

utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.provides.build: utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o


# Object files for target aruco_imagem
aruco_imagem_OBJECTS = \
"CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o"

# External object files for target aruco_imagem
aruco_imagem_EXTERNAL_OBJECTS =

utils/aruco_imagem: utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o
utils/aruco_imagem: utils/CMakeFiles/aruco_imagem.dir/build.make
utils/aruco_imagem: src/libaruco.so.3.0.10
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
utils/aruco_imagem: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
utils/aruco_imagem: utils/CMakeFiles/aruco_imagem.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alantavares/aruco/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable aruco_imagem"
	cd /home/alantavares/aruco/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_imagem.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_imagem.dir/build: utils/aruco_imagem

.PHONY : utils/CMakeFiles/aruco_imagem.dir/build

utils/CMakeFiles/aruco_imagem.dir/requires: utils/CMakeFiles/aruco_imagem.dir/aruco_imagem.cpp.o.requires

.PHONY : utils/CMakeFiles/aruco_imagem.dir/requires

utils/CMakeFiles/aruco_imagem.dir/clean:
	cd /home/alantavares/aruco/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_imagem.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_imagem.dir/clean

utils/CMakeFiles/aruco_imagem.dir/depend:
	cd /home/alantavares/aruco/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alantavares/aruco /home/alantavares/aruco/utils /home/alantavares/aruco/build /home/alantavares/aruco/build/utils /home/alantavares/aruco/build/utils/CMakeFiles/aruco_imagem.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_imagem.dir/depend

