include_directories(include)
include_directories(PowerAux lib/src)

add_subdirectory(lib)

add_executable(PowerAux src/main.cpp lib/src/SignalFlashHandler.cpp)

target_link_libraries(PowerAux PRIVATE mbed-os-lib Common-lib PowerAux-lib)

mbed_set_post_build(PowerAux)
