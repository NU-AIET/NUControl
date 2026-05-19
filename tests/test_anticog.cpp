#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "anticog_helpers.hpp"

using Catch::Matchers::WithinAbs;
static constexpr float kTol = 1e-5f;
static constexpr float kPi  = static_cast<float>(M_PI);

// --- Zero map ----------------------------------------------------------------

TEST_CASE("anticog torque: empty map always returns zero", "[anticog]")
{
    AnticoggingCompensator<0> comp(default_anticog_torque_map);
    REQUIRE_THAT(comp.get_cogging_torque(0.f),    WithinAbs(0.f, kTol));
    REQUIRE_THAT(comp.get_cogging_torque(1.5f),   WithinAbs(0.f, kTol));
    REQUIRE_THAT(comp.get_cogging_torque(-kPi),   WithinAbs(0.f, kTol));
}

TEST_CASE("anticog voltage: empty map always returns zero phases", "[anticog]")
{
    AnticoggingCompensator<0> comp(default_anticog_volt_map);
    auto v = comp.get_cogging_voltage(1.f);
    REQUIRE_THAT(v.a, WithinAbs(0.f, kTol));
    REQUIRE_THAT(v.b, WithinAbs(0.f, kTol));
    REQUIRE_THAT(v.c, WithinAbs(0.f, kTol));
}

// --- Exact table hits --------------------------------------------------------

TEST_CASE("anticog torque: exact table entries are returned unchanged", "[anticog]")
{
    // 4-entry table evenly spaced over [0, 2pi): indices at 0, pi/2, pi, 3pi/2
    std::array<float, 4> map{1.f, 2.f, 3.f, 4.f};
    AnticoggingCompensator<4> comp(map);

    REQUIRE_THAT(comp.get_cogging_torque(0.f),          WithinAbs(1.f, kTol));
    REQUIRE_THAT(comp.get_cogging_torque(kPi / 2.f),    WithinAbs(2.f, kTol));
    REQUIRE_THAT(comp.get_cogging_torque(kPi),          WithinAbs(3.f, kTol));
    REQUIRE_THAT(comp.get_cogging_torque(3.f * kPi / 2.f), WithinAbs(4.f, kTol));
}

// --- Linear interpolation ----------------------------------------------------

TEST_CASE("anticog torque: midpoint between entries is their average", "[anticog]")
{
    std::array<float, 4> map{0.f, 4.f, 0.f, -4.f};
    AnticoggingCompensator<4> comp(map);

    // Midpoint between index 0 (0 rad) and index 1 (pi/2 rad) => angle pi/4
    REQUIRE_THAT(comp.get_cogging_torque(kPi / 4.f), WithinAbs(2.f, kTol));

    // Midpoint between index 1 (pi/2) and index 2 (pi) => angle 3pi/4
    REQUIRE_THAT(comp.get_cogging_torque(3.f * kPi / 4.f), WithinAbs(2.f, kTol));
}

TEST_CASE("anticog torque: interpolation is linear between entries", "[anticog]")
{
    // Two-entry table: 0 at 0 rad, 10 at pi rad. Quarter of the way => 2.5
    std::array<float, 2> map{0.f, 10.f};
    AnticoggingCompensator<2> comp(map);

    // 1/4 of the way between entry 0 and entry 1
    REQUIRE_THAT(comp.get_cogging_torque(kPi / 4.f), WithinAbs(2.5f, kTol));
    // 3/4 of the way between entry 0 and entry 1
    REQUIRE_THAT(comp.get_cogging_torque(3.f * kPi / 4.f), WithinAbs(7.5f, kTol));
}

// --- Angle wrapping ----------------------------------------------------------

TEST_CASE("anticog torque: negative angles are wrapped correctly", "[anticog]")
{
    std::array<float, 4> map{1.f, 2.f, 3.f, 4.f};
    AnticoggingCompensator<4> comp(map);

    // -pi/2 wraps to 3pi/2 => index 3 => value 4
    REQUIRE_THAT(comp.get_cogging_torque(-kPi / 2.f), WithinAbs(4.f, kTol));

    // -pi wraps to pi => index 2 => value 3
    REQUIRE_THAT(comp.get_cogging_torque(-kPi), WithinAbs(3.f, kTol));
}

TEST_CASE("anticog torque: angles beyond 2pi are wrapped correctly", "[anticog]")
{
    std::array<float, 4> map{1.f, 2.f, 3.f, 4.f};
    AnticoggingCompensator<4> comp(map);

    // 2pi + 0 == 0 => index 0 => value 1
    REQUIRE_THAT(comp.get_cogging_torque(2.f * kPi), WithinAbs(1.f, kTol));

    // 2pi + pi/2 == pi/2 => index 1 => value 2
    REQUIRE_THAT(comp.get_cogging_torque(2.f * kPi + kPi / 2.f), WithinAbs(2.f, kTol));
}

// --- Circular wrap-around between last and first entry -----------------------

TEST_CASE("anticog torque: interpolates correctly across the 2pi boundary", "[anticog]")
{
    // Table: index 0 = 0.0, index 1 = 10.0 (two entries, boundary at pi)
    // The wrap-around is between index 1 (at pi) and index 0 (at 2pi/0).
    // Midpoint of that segment is at 3pi/2 => average of 10 and 0 = 5.
    std::array<float, 2> map{0.f, 10.f};
    AnticoggingCompensator<2> comp(map);

    REQUIRE_THAT(comp.get_cogging_torque(3.f * kPi / 2.f), WithinAbs(5.f, kTol));
}

// --- Voltage map -------------------------------------------------------------

TEST_CASE("anticog voltage: exact table entries are returned unchanged", "[anticog]")
{
    PhaseValues<std::array<float, 2>> vmap{
        std::array<float, 2>{1.f, 2.f},
        std::array<float, 2>{3.f, 4.f},
        std::array<float, 2>{5.f, 6.f}
    };
    AnticoggingCompensator<2> comp(vmap);

    auto v0 = comp.get_cogging_voltage(0.f);
    REQUIRE_THAT(v0.a, WithinAbs(1.f, kTol));
    REQUIRE_THAT(v0.b, WithinAbs(3.f, kTol));
    REQUIRE_THAT(v0.c, WithinAbs(5.f, kTol));

    auto v1 = comp.get_cogging_voltage(kPi);
    REQUIRE_THAT(v1.a, WithinAbs(2.f, kTol));
    REQUIRE_THAT(v1.b, WithinAbs(4.f, kTol));
    REQUIRE_THAT(v1.c, WithinAbs(6.f, kTol));
}

TEST_CASE("anticog voltage: midpoint interpolates all three phases", "[anticog]")
{
    PhaseValues<std::array<float, 2>> vmap{
        std::array<float, 2>{0.f, 4.f},
        std::array<float, 2>{0.f, -4.f},
        std::array<float, 2>{2.f, 6.f}
    };
    AnticoggingCompensator<2> comp(vmap);

    auto v = comp.get_cogging_voltage(kPi / 2.f);  // midpoint between index 0 and 1
    REQUIRE_THAT(v.a, WithinAbs(2.f,  kTol));
    REQUIRE_THAT(v.b, WithinAbs(-2.f, kTol));
    REQUIRE_THAT(v.c, WithinAbs(4.f,  kTol));
}
