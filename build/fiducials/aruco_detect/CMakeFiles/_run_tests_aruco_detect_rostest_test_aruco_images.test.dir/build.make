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

# Utility rule file for _run_tests_aruco_detect_rostest_test_aruco_images.test.

# Include the progress variables for this target.
include fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/progress.make

fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test:
	cd /home/student/catkin_ws/build/fiducials/aruco_detect && ../../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/student/catkin_ws/build/test_results/aruco_detect/rostest-test_aruco_images.xml "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/student/catkin_ws/src/fiducials/aruco_detect --package=aruco_detect --results-filename test_aruco_images.xml --results-base-dir \"/home/student/catkin_ws/build/test_results\" /home/student/catkin_ws/src/fiducials/aruco_detect/test/aruco_images.test "

_run_tests_aruco_detect_rostest_test_aruco_images.test: fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test
_run_tests_aruco_detect_rostest_test_aruco_images.test: fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/build.make

.PHONY : _run_tests_aruco_detect_rostest_test_aruco_images.test

# Rule to build all files generated by this target.
fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/build: _run_tests_aruco_detect_rostest_test_aruco_images.test

.PHONY : fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/build

fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/clean:
	cd /home/student/catkin_ws/build/fiducials/aruco_detect && $(CMAKE_COMMAND) -P CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/cmake_clean.cmake
.PHONY : fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/clean

fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/depend:
	cd /home/student/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/catkin_ws/src /home/student/catkin_ws/src/fiducials/aruco_detect /home/student/catkin_ws/build /home/student/catkin_ws/build/fiducials/aruco_detect /home/student/catkin_ws/build/fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : fiducials/aruco_detect/CMakeFiles/_run_tests_aruco_detect_rostest_test_aruco_images.test.dir/depend

