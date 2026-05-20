include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

include(Catch)

add_executable(test_transformations test_transformations.cpp)
target_link_libraries(test_transformations PRIVATE nucontrol_core Catch2::Catch2WithMain)
catch_discover_tests(test_transformations)

add_executable(test_helpers test_helpers.cpp)
target_link_libraries(test_helpers PRIVATE nucontrol_core Catch2::Catch2WithMain)
catch_discover_tests(test_helpers)

add_executable(test_anticog test_anticog.cpp)
target_link_libraries(test_anticog PRIVATE nucontrol_core Catch2::Catch2WithMain)
catch_discover_tests(test_anticog)

add_executable(test_brushless_controller test_brushless_controller.cpp)
target_link_libraries(test_brushless_controller PRIVATE nucontrol_core Catch2::Catch2WithMain)
catch_discover_tests(test_brushless_controller)
