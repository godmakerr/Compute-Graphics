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
CMAKE_SOURCE_DIR = /home/hyf/graphics/pj1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hyf/graphics/pj1/build

# Include any dependencies generated for this target.
include glfw/examples/CMakeFiles/splitview.dir/depend.make

# Include the progress variables for this target.
include glfw/examples/CMakeFiles/splitview.dir/progress.make

# Include the compile flags for this target's objects.
include glfw/examples/CMakeFiles/splitview.dir/flags.make

glfw/examples/CMakeFiles/splitview.dir/splitview.c.o: glfw/examples/CMakeFiles/splitview.dir/flags.make
glfw/examples/CMakeFiles/splitview.dir/splitview.c.o: ../glfw/examples/splitview.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hyf/graphics/pj1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object glfw/examples/CMakeFiles/splitview.dir/splitview.c.o"
	cd /home/hyf/graphics/pj1/build/glfw/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/splitview.dir/splitview.c.o   -c /home/hyf/graphics/pj1/glfw/examples/splitview.c

glfw/examples/CMakeFiles/splitview.dir/splitview.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/splitview.dir/splitview.c.i"
	cd /home/hyf/graphics/pj1/build/glfw/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hyf/graphics/pj1/glfw/examples/splitview.c > CMakeFiles/splitview.dir/splitview.c.i

glfw/examples/CMakeFiles/splitview.dir/splitview.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/splitview.dir/splitview.c.s"
	cd /home/hyf/graphics/pj1/build/glfw/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hyf/graphics/pj1/glfw/examples/splitview.c -o CMakeFiles/splitview.dir/splitview.c.s

glfw/examples/CMakeFiles/splitview.dir/__/deps/glad.c.o: glfw/examples/CMakeFiles/splitview.dir/flags.make
glfw/examples/CMakeFiles/splitview.dir/__/deps/glad.c.o: ../glfw/deps/glad.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hyf/graphics/pj1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object glfw/examples/CMakeFiles/splitview.dir/__/deps/glad.c.o"
	cd /home/hyf/graphics/pj1/build/glfw/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/splitview.dir/__/deps/glad.c.o   -c /home/hyf/graphics/pj1/glfw/deps/glad.c

glfw/examples/CMakeFiles/splitview.dir/__/deps/glad.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/splitview.dir/__/deps/glad.c.i"
	cd /home/hyf/graphics/pj1/build/glfw/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/hyf/graphics/pj1/glfw/deps/glad.c > CMakeFiles/splitview.dir/__/deps/glad.c.i

glfw/examples/CMakeFiles/splitview.dir/__/deps/glad.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/splitview.dir/__/deps/glad.c.s"
	cd /home/hyf/graphics/pj1/build/glfw/examples && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/hyf/graphics/pj1/glfw/deps/glad.c -o CMakeFiles/splitview.dir/__/deps/glad.c.s

# Object files for target splitview
splitview_OBJECTS = \
"CMakeFiles/splitview.dir/splitview.c.o" \
"CMakeFiles/splitview.dir/__/deps/glad.c.o"

# External object files for target splitview
splitview_EXTERNAL_OBJECTS =

glfw/examples/splitview: glfw/examples/CMakeFiles/splitview.dir/splitview.c.o
glfw/examples/splitview: glfw/examples/CMakeFiles/splitview.dir/__/deps/glad.c.o
glfw/examples/splitview: glfw/examples/CMakeFiles/splitview.dir/build.make
glfw/examples/splitview: glfw/src/libglfw3.a
glfw/examples/splitview: /usr/lib/x86_64-linux-gnu/librt.so
glfw/examples/splitview: /usr/lib/x86_64-linux-gnu/libm.so
glfw/examples/splitview: /usr/lib/x86_64-linux-gnu/libX11.so
glfw/examples/splitview: /usr/lib/x86_64-linux-gnu/libXrandr.so
glfw/examples/splitview: /usr/lib/x86_64-linux-gnu/libXinerama.so
glfw/examples/splitview: /usr/lib/x86_64-linux-gnu/libXxf86vm.so
glfw/examples/splitview: /usr/lib/x86_64-linux-gnu/libXcursor.so
glfw/examples/splitview: glfw/examples/CMakeFiles/splitview.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hyf/graphics/pj1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C executable splitview"
	cd /home/hyf/graphics/pj1/build/glfw/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/splitview.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
glfw/examples/CMakeFiles/splitview.dir/build: glfw/examples/splitview

.PHONY : glfw/examples/CMakeFiles/splitview.dir/build

glfw/examples/CMakeFiles/splitview.dir/clean:
	cd /home/hyf/graphics/pj1/build/glfw/examples && $(CMAKE_COMMAND) -P CMakeFiles/splitview.dir/cmake_clean.cmake
.PHONY : glfw/examples/CMakeFiles/splitview.dir/clean

glfw/examples/CMakeFiles/splitview.dir/depend:
	cd /home/hyf/graphics/pj1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hyf/graphics/pj1 /home/hyf/graphics/pj1/glfw/examples /home/hyf/graphics/pj1/build /home/hyf/graphics/pj1/build/glfw/examples /home/hyf/graphics/pj1/build/glfw/examples/CMakeFiles/splitview.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : glfw/examples/CMakeFiles/splitview.dir/depend

