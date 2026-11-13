if(PITWALL_USE_GOOGLETEST)
    find_package(GTest CONFIG REQUIRED)
    add_executable(pitwall_domain_tests domain_tests_gtest.cpp)
    target_link_libraries(pitwall_domain_tests PRIVATE pitwall::domain GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(pitwall_domain_tests)
    add_executable(pitwall_physics_tests physics_tests_gtest.cpp)
    target_link_libraries(pitwall_physics_tests PRIVATE pitwall::simulation GTest::gtest_main)
    gtest_discover_tests(pitwall_physics_tests)
    add_executable(pitwall_track_tests track_tests_gtest.cpp)
    target_link_libraries(pitwall_track_tests PRIVATE pitwall::track pitwall::simulation GTest::gtest_main)
    gtest_discover_tests(pitwall_track_tests)
else()
    add_executable(pitwall_domain_tests domain_tests_offline.cpp)
    target_link_libraries(pitwall_domain_tests PRIVATE pitwall::domain)
    add_test(NAME domain.contract COMMAND pitwall_domain_tests)
    add_executable(pitwall_physics_tests physics_tests_offline.cpp)
    target_link_libraries(pitwall_physics_tests PRIVATE pitwall::simulation)
    foreach(case IN ITEMS rest analytic_acceleration braking partial_tick_stop resistance
            speed_limit turn_symmetry analytic_circle steering_rate grip_limit bad_controls
            bad_config bad_state_and_duration fps_independence accumulator overload reset
            telemetry invalid_frame_is_atomic heading_wrap_and_long_run)
        add_test(NAME physics.${case} COMMAND pitwall_physics_tests ${case})
    endforeach()
    add_executable(pitwall_track_tests track_tests_offline.cpp)
    target_link_libraries(pitwall_track_tests PRIVATE pitwall::track pitwall::simulation)
    foreach(case IN ITEMS geometry projection wrapping boundaries swept_shortcut rounded_corner
            bad_geometry bad_queries oval_roundtrip wait_for_start directed_start ordered_lap
            best_lap off_track shortcut_lap teleport reverse_gate line_endpoints missing_checkpoints
            recovery reset bad_time_atomic bad_rules interpolated_time multiple_gates high_timestamp
            vehicle_integration)
        add_test(NAME track.${case} COMMAND pitwall_track_tests ${case})
    endforeach()
endif()

foreach(app IN ITEMS simulator analyzer)
    add_test(NAME ${app}.startup COMMAND pitwall_${app})
    set_tests_properties(${app}.startup PROPERTIES PASS_REGULAR_EXPRESSION "foundation ready")
    add_test(NAME ${app}.help COMMAND pitwall_${app} --help)
    set_tests_properties(${app}.help PROPERTIES PASS_REGULAR_EXPRESSION "Usage:")
    add_test(NAME ${app}.version COMMAND pitwall_${app} --version)
    set_tests_properties(${app}.version PROPERTIES PASS_REGULAR_EXPRESSION "${PROJECT_VERSION}")
    add_test(NAME ${app}.reject_unknown COMMAND pitwall_${app} --unknown)
    set_tests_properties(${app}.reject_unknown PROPERTIES WILL_FAIL TRUE)
endforeach()

set_tests_properties(simulator.startup PROPERTIES PASS_REGULAR_EXPRESSION "vehicle physics ready")
add_test(NAME simulator.demo COMMAND pitwall_simulator --demo)
set_tests_properties(simulator.demo PROPERTIES PASS_REGULAR_EXPRESSION "demo completed: steps=1200")
add_test(NAME simulator.track_demo COMMAND pitwall_simulator --track-demo)
set_tests_properties(simulator.track_demo PROPERTIES
    PASS_REGULAR_EXPRESSION "track demo completed: laps=2, valid=2")
