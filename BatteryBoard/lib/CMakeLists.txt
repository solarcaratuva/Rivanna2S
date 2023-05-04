add_library(BatteryBoard-lib
    src/BPSCANInterface.cpp
    src/BPSRelayController.cpp
)

target_include_directories(BatteryBoard-lib PUBLIC include)

target_link_libraries(BatteryBoard-lib mbed-os-lib Common-lib)
