cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(RivannaTests)

# Add the GoogleTest submodule
add_subdirectory(googletest)

# Include the GoogleTest headers
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(../PowerAux/lib/include)

# Specify the test source files
set(TEST_SOURCES
    exampleTest.cpp
    # Add other test files here
)

set(SOURCE_FILES
    ../PowerAux/lib/include/SignalFlashHandler.h
    ../PowerAux/lib/include/TestSignalControl.h
    #../PowerAux/lib/src/SignalFlashHandler.cpp
    ../PowerAux/lib/src/TestSignalControl.cpp
    ethan.h
    ethan.cpp
    # Add other source files here
)

enable_testing()

add_executable(
  SignalFlashHandlerTest
  ${TEST_SOURCES} ${SOURCE_FILES}
)
target_link_libraries(
  SignalFlashHandlerTest
  gtest_main
)

include(GoogleTest)
gtest_discover_tests(SignalFlashHandlerTest)
