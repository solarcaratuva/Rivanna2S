# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild

# Utility rule file for greentea-client-populate.

# Include any custom commands dependencies for this target.
include CMakeFiles/greentea-client-populate.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/greentea-client-populate.dir/progress.make

CMakeFiles/greentea-client-populate: CMakeFiles/greentea-client-populate-complete

CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-install
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-mkdir
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-download
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-patch
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-configure
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-build
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-install
CMakeFiles/greentea-client-populate-complete: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-test
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'greentea-client-populate'"
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles/greentea-client-populate-complete
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-done

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update:
.PHONY : greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-build: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "No build step for 'greentea-client-populate'"
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E echo_append
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-build

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-configure: greentea-client-populate-prefix/tmp/greentea-client-populate-cfgcmd.txt
greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-configure: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "No configure step for 'greentea-client-populate'"
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E echo_append
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-configure

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-download: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-gitinfo.txt
greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-download: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'greentea-client-populate'"
	cd /root/Rivanna2/BUILD/unittests/_deps && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -P /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp/greentea-client-populate-gitclone.cmake
	cd /root/Rivanna2/BUILD/unittests/_deps && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-download

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-install: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'greentea-client-populate'"
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E echo_append
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-install

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'greentea-client-populate'"
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-src
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E make_directory /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-mkdir

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-patch: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'greentea-client-populate'"
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E echo_append
	/usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-patch

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update:
.PHONY : greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-test: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No test step for 'greentea-client-populate'"
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E echo_append
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-build && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -E touch /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-test

greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Performing update step for 'greentea-client-populate'"
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-src && /usr/local/lib/python3.8/dist-packages/cmake/data/bin/cmake -P /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp/greentea-client-populate-gitupdate.cmake

greentea-client-populate: CMakeFiles/greentea-client-populate
greentea-client-populate: CMakeFiles/greentea-client-populate-complete
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-build
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-configure
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-download
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-install
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-mkdir
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-patch
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-test
greentea-client-populate: greentea-client-populate-prefix/src/greentea-client-populate-stamp/greentea-client-populate-update
greentea-client-populate: CMakeFiles/greentea-client-populate.dir/build.make
.PHONY : greentea-client-populate

# Rule to build all files generated by this target.
CMakeFiles/greentea-client-populate.dir/build: greentea-client-populate
.PHONY : CMakeFiles/greentea-client-populate.dir/build

CMakeFiles/greentea-client-populate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/greentea-client-populate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/greentea-client-populate.dir/clean

CMakeFiles/greentea-client-populate.dir/depend:
	cd /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild /root/Rivanna2/BUILD/unittests/_deps/greentea-client-subbuild/CMakeFiles/greentea-client-populate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/greentea-client-populate.dir/depend

