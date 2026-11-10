add_library(pitwall_domain src/telemetry.cpp)
add_library(pitwall::domain ALIAS pitwall_domain)
target_include_directories(pitwall_domain PUBLIC include)
target_link_libraries(pitwall_domain PUBLIC pitwall_options)
