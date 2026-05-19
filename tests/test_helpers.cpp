#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "helpers.hpp"

using Catch::Matchers::WithinAbs;
static constexpr float kTol = 1e-5f;

// --- Constants ---------------------------------------------------------------

TEST_CASE("Constants: values are correct", "[constants]")
{
    REQUIRE_THAT(_1__SQRT_3_, WithinAbs(1.f / std::sqrt(3.f), kTol));
    REQUIRE_THAT(_2__SQRT_3_, WithinAbs(2.f / std::sqrt(3.f), kTol));
    REQUIRE_THAT(_SQRT_3__2_, WithinAbs(std::sqrt(3.f) / 2.f, kTol));
    REQUIRE_THAT(_2_PI_,      WithinAbs(2.f * static_cast<float>(M_PI), kTol));
    REQUIRE_THAT(_SQRT_2_,    WithinAbs(std::sqrt(2.f), kTol));
}

// --- normalize_angle ---------------------------------------------------------

TEST_CASE("normalize_angle: already-normalized angles are unchanged", "[normalize_angle]")
{
    REQUIRE_THAT(normalize_angle(0.f),                          WithinAbs(0.f, kTol));
    REQUIRE_THAT(normalize_angle(1.f),                          WithinAbs(1.f, kTol));
    REQUIRE_THAT(normalize_angle(-1.f),                         WithinAbs(-1.f, kTol));
    REQUIRE_THAT(normalize_angle(static_cast<float>(M_PI_2)),   WithinAbs(static_cast<float>(M_PI_2), kTol));
    REQUIRE_THAT(normalize_angle(-static_cast<float>(M_PI_2)),  WithinAbs(-static_cast<float>(M_PI_2), kTol));
}

TEST_CASE("normalize_angle: output is always within (-pi, pi)", "[normalize_angle]")
{
    // Sweep a wide range and verify bounds
    for (int i = -20; i <= 20; ++i) {
        float angle = static_cast<float>(i) * static_cast<float>(M_PI) * 0.7f;
        float result = normalize_angle(angle);
        REQUIRE(result > -static_cast<float>(M_PI) - kTol);
        REQUIRE(result <= static_cast<float>(M_PI) + kTol);
    }
}

TEST_CASE("normalize_angle: wraps positive overflow", "[normalize_angle]")
{
    REQUIRE_THAT(normalize_angle(2.f * static_cast<float>(M_PI)),
                 WithinAbs(0.f, kTol));
    REQUIRE_THAT(normalize_angle(3.f * static_cast<float>(M_PI)),
                 WithinAbs(static_cast<float>(M_PI), kTol));
    REQUIRE_THAT(normalize_angle(2.5f * static_cast<float>(M_PI)),
                 WithinAbs(static_cast<float>(M_PI_2), kTol));
}

TEST_CASE("normalize_angle: wraps negative overflow", "[normalize_angle]")
{
    REQUIRE_THAT(normalize_angle(-2.f * static_cast<float>(M_PI)),
                 WithinAbs(0.f, kTol));
    REQUIRE_THAT(normalize_angle(-2.5f * static_cast<float>(M_PI)),
                 WithinAbs(-static_cast<float>(M_PI_2), kTol));
}

TEST_CASE("normalize_angle: multiple full rotations", "[normalize_angle]")
{
    float base = 0.8f;
    for (int n = -5; n <= 5; ++n) {
        float shifted = base + n * _2_PI_;
        REQUIRE_THAT(normalize_angle(shifted), WithinAbs(base, kTol));
    }
}

// --- nu_sincos ---------------------------------------------------------------

TEST_CASE("nu_sincos: matches sinf/cosf at key angles", "[nu_sincos]")
{
    const float angles[] = {0.f, static_cast<float>(M_PI_2), static_cast<float>(M_PI),
                             -static_cast<float>(M_PI_2), 1.23f, -2.71f};
    for (float a : angles) {
        auto [s, c] = nu_sincos(a);
        REQUIRE_THAT(s, WithinAbs(sinf(a), kTol));
        REQUIRE_THAT(c, WithinAbs(cosf(a), kTol));
    }
}

TEST_CASE("nu_sincos: wraps angles before computing", "[nu_sincos]")
{
    // sin/cos are 2pi-periodic, so nu_sincos(a + 2pi) == nu_sincos(a)
    float a = 1.1f;
    auto [s0, c0] = nu_sincos(a);
    auto [s1, c1] = nu_sincos(a + _2_PI_);
    REQUIRE_THAT(s1, WithinAbs(s0, kTol));
    REQUIRE_THAT(c1, WithinAbs(c0, kTol));
}

TEST_CASE("nu_sincos: Pythagorean identity holds", "[nu_sincos]")
{
    const float angles[] = {0.f, 0.5f, 1.f, 2.f, -1.5f, 100.f};
    for (float a : angles) {
        auto [s, c] = nu_sincos(a);
        REQUIRE_THAT(s * s + c * c, WithinAbs(1.f, kTol));
    }
}

// --- near_zero ---------------------------------------------------------------

TEST_CASE("near_zero: equal values pass", "[near_zero]")
{
    REQUIRE(near_zero(1.f, 1.f));
    REQUIRE(near_zero(0.f, 0.f));
    REQUIRE(near_zero(-3.f, -3.f));
}

TEST_CASE("near_zero: values within default tolerance pass", "[near_zero]")
{
    REQUIRE(near_zero(1.f, 1.0009f));
    REQUIRE(near_zero(0.f, 0.0009f));
}

TEST_CASE("near_zero: values outside default tolerance fail", "[near_zero]")
{
    REQUIRE_FALSE(near_zero(1.f, 1.002f));
    REQUIRE_FALSE(near_zero(0.f, 0.002f));
}

TEST_CASE("near_zero: custom tolerance is respected", "[near_zero]")
{
    REQUIRE(near_zero(0.f, 0.05f, 0.1f));
    REQUIRE_FALSE(near_zero(0.f, 0.05f, 0.01f));
}
