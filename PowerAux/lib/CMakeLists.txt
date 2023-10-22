add_library(PowerAux-lib)

target_include_directories(PowerAux-lib PUBLIC include)

target_link_libraries(PowerAux-lib mbed-os-lib Common-lib)
