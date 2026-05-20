#include <catch2/catch_test_macros.hpp>
#include "cogging_mapper.hpp"

// --- Mocks (same as test_brushless_controller) ---

struct MockEncoder : public IAbsoluteEncoder
{
  float angle = 0.f;
  float read() override { return angle; }
};

struct MockDriver : public IBrushlessDriver
{
  bool enabled = false;
  bool init() override { return true; }
  void enable()  override { enabled = true; }
  void disable() override { enabled = false; }
  PhaseValues<int> set_phase_voltages(PhaseValues<float>) override { return {0, 0, 0}; }
};

struct MockSensorPackage : public ICurrentSensorPackage
{
  bool init_sensors() override { return true; }
  PhaseValues<float> get_phase_currents(bool) override { return {0.f, 0.f, 0.f}; }
  void set_filters(DiscreteFilter<float, float>) override {}
  void print_calibration() override {}
  bool align_sensors(IBrushlessDriver &, float) override { return true; }
  bool load_calibration(PhaseValues<int>, PhaseValues<int>) override { return true; }
};

static auto no_sleep = [](int){};

// Helper: build a ready-to-map controller + mapper pair.
struct Fixture
{
  MockEncoder       enc;
  MockDriver        drv;
  MockSensorPackage cs;
  BrushlessController ctrl;
  CoggingMapper<4> mapper;   // 4 steps keeps tests fast

  Fixture()
  : ctrl{EC45_Flat, drv, cs, enc, no_sleep},
    mapper{ctrl, no_sleep}
  {
    ctrl.init_components();
  }
};

// --- Tests ---

TEST_CASE("map_cogging: is_done starts false and driver is enabled")
{
  Fixture f;
  f.mapper.map_cogging(1);
  REQUIRE_FALSE(f.mapper.is_done());
  REQUIRE(f.drv.enabled == true);
}

TEST_CASE("timeout: sentinel recorded and mapper advances after timeout_max ticks")
{
  Fixture f;
  // Short timeout so the test runs quickly
  constexpr size_t TIMEOUT = 50;
  f.mapper.set_timeout_state(true, TIMEOUT, -127.f);
  f.mapper.map_cogging(1);

  // Encoder stays at 0 — rotor never reaches any target, so every position times out.
  // Forward pass: 4 positions * TIMEOUT ticks each
  for (size_t tick = 0; tick < 4 * TIMEOUT + 10; ++tick) {
    f.mapper.step();
    if (f.mapper.is_done()) break;
  }

  // All forward positions should be sentinel values
  for (size_t i = 0; i < 4; ++i) {
    REQUIRE(f.mapper.torques().at(i) == -127.f);
  }
}

TEST_CASE("completes: is_done true after both passes finish via timeout")
{
  Fixture f;
  constexpr size_t TIMEOUT = 10;
  f.mapper.set_timeout_state(true, TIMEOUT, -127.f);
  f.mapper.map_cogging(1);

  // 2 passes * 4 steps * TIMEOUT ticks, with a small margin
  const size_t max_ticks = 2 * 4 * TIMEOUT * 2;
  for (size_t tick = 0; tick < max_ticks; ++tick) {
    f.mapper.step();
    if (f.mapper.is_done()) break;
  }

  REQUIRE(f.mapper.is_done());
}

TEST_CASE("map_cogging: re-entrant reset clears done flag")
{
  Fixture f;
  constexpr size_t TIMEOUT = 10;
  f.mapper.set_timeout_state(true, TIMEOUT, -127.f);
  f.mapper.map_cogging(1);

  const size_t max_ticks = 2 * 4 * TIMEOUT * 2;
  for (size_t tick = 0; tick < max_ticks; ++tick) {
    f.mapper.step();
    if (f.mapper.is_done()) break;
  }
  REQUIRE(f.mapper.is_done());

  // Calling map_cogging again should reset the done flag
  f.mapper.map_cogging(1);
  REQUIRE_FALSE(f.mapper.is_done());
}
