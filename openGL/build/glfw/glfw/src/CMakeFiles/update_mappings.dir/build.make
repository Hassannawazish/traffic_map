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

# Utility rule file for update_mappings.

# Include any custom commands dependencies for this target.
include glfw/glfw/src/CMakeFiles/update_mappings.dir/compiler_depend.make

# Include the progress variables for this target.
include glfw/glfw/src/CMakeFiles/update_mappings.dir/progress.make

glfw/glfw/src/CMakeFiles/update_mappings:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Updating gamepad mappings from upstream repository"
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glfw/glfw/src && /home/maanz-ai/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -P /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glfw/glfw/CMake/GenerateMappings.cmake mappings.h.in mappings.h

update_mappings: glfw/glfw/src/CMakeFiles/update_mappings
update_mappings: glfw/glfw/src/CMakeFiles/update_mappings.dir/build.make
.PHONY : update_mappings

# Rule to build all files generated by this target.
glfw/glfw/src/CMakeFiles/update_mappings.dir/build: update_mappings
.PHONY : glfw/glfw/src/CMakeFiles/update_mappings.dir/build

glfw/glfw/src/CMakeFiles/update_mappings.dir/clean:
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glfw/glfw/src && $(CMAKE_COMMAND) -P CMakeFiles/update_mappings.dir/cmake_clean.cmake
.PHONY : glfw/glfw/src/CMakeFiles/update_mappings.dir/clean

glfw/glfw/src/CMakeFiles/update_mappings.dir/depend:
	cd /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/glfw/glfw/src /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glfw/glfw/src /home/maanz-ai/Downloads/traffic_map_opengl/traffic_map/openGL/build/glfw/glfw/src/CMakeFiles/update_mappings.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : glfw/glfw/src/CMakeFiles/update_mappings.dir/depend

