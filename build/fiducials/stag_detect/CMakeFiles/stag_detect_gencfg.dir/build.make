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
CMAKE_SOURCE_DIR = /home/student/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/catkin_ws/build

# Utility rule file for stag_detect_gencfg.

# Include the progress variables for this target.
include fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/progress.make

fiducials/stag_detect/CMakeFiles/stag_detect_gencfg: /home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h
fiducials/stag_detect/CMakeFiles/stag_detect_gencfg: /home/student/catkin_ws/devel/lib/python3/dist-packages/stag_detect/cfg/DetectorParamsConfig.py


/home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h: /home/student/catkin_ws/src/fiducials/stag_detect/cfg/DetectorParams.cfg
/home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.py.template
/home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/student/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dynamic reconfigure files from cfg/DetectorParams.cfg: /home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h /home/student/catkin_ws/devel/lib/python3/dist-packages/stag_detect/cfg/DetectorParamsConfig.py"
	cd /home/student/catkin_ws/build/fiducials/stag_detect && ../../catkin_generated/env_cached.sh /home/student/catkin_ws/build/fiducials/stag_detect/setup_custom_pythonpath.sh /home/student/catkin_ws/src/fiducials/stag_detect/cfg/DetectorParams.cfg /opt/ros/noetic/share/dynamic_reconfigure/cmake/.. /home/student/catkin_ws/devel/share/stag_detect /home/student/catkin_ws/devel/include/stag_detect /home/student/catkin_ws/devel/lib/python3/dist-packages/stag_detect

/home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig.dox: /home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig.dox

/home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig-usage.dox: /home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig-usage.dox

/home/student/catkin_ws/devel/lib/python3/dist-packages/stag_detect/cfg/DetectorParamsConfig.py: /home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/student/catkin_ws/devel/lib/python3/dist-packages/stag_detect/cfg/DetectorParamsConfig.py

/home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig.wikidoc: /home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig.wikidoc

stag_detect_gencfg: fiducials/stag_detect/CMakeFiles/stag_detect_gencfg
stag_detect_gencfg: /home/student/catkin_ws/devel/include/stag_detect/DetectorParamsConfig.h
stag_detect_gencfg: /home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig.dox
stag_detect_gencfg: /home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig-usage.dox
stag_detect_gencfg: /home/student/catkin_ws/devel/lib/python3/dist-packages/stag_detect/cfg/DetectorParamsConfig.py
stag_detect_gencfg: /home/student/catkin_ws/devel/share/stag_detect/docs/DetectorParamsConfig.wikidoc
stag_detect_gencfg: fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/build.make

.PHONY : stag_detect_gencfg

# Rule to build all files generated by this target.
fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/build: stag_detect_gencfg

.PHONY : fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/build

fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/clean:
	cd /home/student/catkin_ws/build/fiducials/stag_detect && $(CMAKE_COMMAND) -P CMakeFiles/stag_detect_gencfg.dir/cmake_clean.cmake
.PHONY : fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/clean

fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/depend:
	cd /home/student/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/catkin_ws/src /home/student/catkin_ws/src/fiducials/stag_detect /home/student/catkin_ws/build /home/student/catkin_ws/build/fiducials/stag_detect /home/student/catkin_ws/build/fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : fiducials/stag_detect/CMakeFiles/stag_detect_gencfg.dir/depend
