include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

add_executable(test_transformations test_transformations.cpp)
target_link_libraries(test_transformations PRIVATE nucontrol_core Catch2::Catch2WithMain)

include(Catch)
catch_discover_tests(test_transformations)
