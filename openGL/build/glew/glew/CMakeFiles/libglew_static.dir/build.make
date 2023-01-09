# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.24

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/maanz-ai/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/maanz-ai/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build

# Include any dependencies generated for this target.
include glew/glew/CMakeFiles/libglew_static.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include glew/glew/CMakeFiles/libglew_static.dir/compiler_depend.make

# Include the progress variables for this target.
include glew/glew/CMakeFiles/libglew_static.dir/progress.make

# Include the compile flags for this target's objects.
include glew/glew/CMakeFiles/libglew_static.dir/flags.make

glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.o: glew/glew/CMakeFiles/libglew_static.dir/flags.make
glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.o: /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glew/glew/src/glew.c
glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.o: glew/glew/CMakeFiles/libglew_static.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.o"
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.o -MF CMakeFiles/libglew_static.dir/src/glew.c.o.d -o CMakeFiles/libglew_static.dir/src/glew.c.o -c /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glew/glew/src/glew.c

glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/libglew_static.dir/src/glew.c.i"
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glew/glew/src/glew.c > CMakeFiles/libglew_static.dir/src/glew.c.i

glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/libglew_static.dir/src/glew.c.s"
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glew/glew/src/glew.c -o CMakeFiles/libglew_static.dir/src/glew.c.s

# Object files for target libglew_static
libglew_static_OBJECTS = \
"CMakeFiles/libglew_static.dir/src/glew.c.o"

# External object files for target libglew_static
libglew_static_EXTERNAL_OBJECTS =

glew/glew/lib/libglew.a: glew/glew/CMakeFiles/libglew_static.dir/src/glew.c.o
glew/glew/lib/libglew.a: glew/glew/CMakeFiles/libglew_static.dir/build.make
glew/glew/lib/libglew.a: glew/glew/CMakeFiles/libglew_static.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library lib/libglew.a"
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew && $(CMAKE_COMMAND) -P CMakeFiles/libglew_static.dir/cmake_clean_target.cmake
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/libglew_static.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "create libGLEW symbolic link"
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew/lib && /home/maanz-ai/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E create_symlink libglew.a libGLEW.a

# Rule to build all files generated by this target.
glew/glew/CMakeFiles/libglew_static.dir/build: glew/glew/lib/libglew.a
.PHONY : glew/glew/CMakeFiles/libglew_static.dir/build

glew/glew/CMakeFiles/libglew_static.dir/clean:
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew && $(CMAKE_COMMAND) -P CMakeFiles/libglew_static.dir/cmake_clean.cmake
.PHONY : glew/glew/CMakeFiles/libglew_static.dir/clean

glew/glew/CMakeFiles/libglew_static.dir/depend:
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glew/glew /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glew/glew/CMakeFiles/libglew_static.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : glew/glew/CMakeFiles/libglew_static.dir/depend

