add_executable(pitwall_analyzer main.cpp)
target_link_libraries(pitwall_analyzer PRIVATE pitwall::domain)
target_compile_definitions(pitwall_analyzer PRIVATE PITWALL_VERSION="${PROJECT_VERSION}")
