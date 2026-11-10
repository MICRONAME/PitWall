add_executable(pitwall_simulator main.cpp)
target_link_libraries(pitwall_simulator PRIVATE pitwall::simulation)
target_compile_definitions(pitwall_simulator PRIVATE PITWALL_VERSION="${PROJECT_VERSION}")
