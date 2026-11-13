add_library(pitwall_track src/track.cpp src/lap_tracker.cpp)
add_library(pitwall::track ALIAS pitwall_track)
target_include_directories(pitwall_track PUBLIC include)
target_link_libraries(pitwall_track PUBLIC pitwall::domain)
