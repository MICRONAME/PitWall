add_library(pitwall_simulation src/vehicle.cpp src/fixed_step.cpp)
add_library(pitwall::simulation ALIAS pitwall_simulation)
target_include_directories(pitwall_simulation PUBLIC include)
target_link_libraries(pitwall_simulation PUBLIC pitwall::domain)
