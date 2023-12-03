cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(RivannaTests)

# Add the GoogleTest submodule
add_subdirectory(googletest)

# Include the GoogleTest headers
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# Specify the test source files
set(TEST_SOURCES
    tests/exampleTest.cpp
    # Add other test files here
)

set(SOURCE_FILES
    ../PowerAux/lib/include/SignalControl.h
    ../PowerAux/lib/include/SignalFlashHandler.h
    ../PowerAux/lib/src/SignalFlashHandler.cpp
    ../PowerAux/lib/include/TestSignalControl.h
    # Add other source files here
)

# Create the test executable
add_executable(runUnitTests ${TEST_SOURCES} ${SOURCE_FILES})

# Link GoogleTest to the test executable
target_link_libraries(runUnitTests gtest gtest_main)
