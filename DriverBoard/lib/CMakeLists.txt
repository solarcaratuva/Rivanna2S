add_library(DriverBoard-lib
    src/DriverCANInterface.cpp
    src/DriverRateLimiter.cpp
)

target_include_directories(DriverBoard-lib PUBLIC include)

target_link_libraries(DriverBoard-lib mbed-os-lib Common-lib)
