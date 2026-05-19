#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "transformations.hpp"

using Catch::Matchers::WithinAbs;
static constexpr float kTol = 1e-5f;

// --- Clarke (phases -> alpha/beta) -------------------------------------------

TEST_CASE("Clarke: balanced three-phase maps to correct alpha/beta", "[clarke]")
{
    // ia + ib + ic = 0 is the balanced condition used in FOC.
    // At angle=0: ia=1, ib=ic=-0.5  =>  alpha=1, beta=0
    PhaseValues<float> phases{1.f, -0.5f, -0.5f};
    auto ab = phases_to_alphabeta(phases);
    REQUIRE_THAT(ab.alpha, WithinAbs(1.f, kTol));
    REQUIRE_THAT(ab.beta,  WithinAbs(0.f, kTol));
}

TEST_CASE("Clarke: zero input maps to zero output", "[clarke]")
{
    PhaseValues<float> phases{0.f, 0.f, 0.f};
    auto ab = phases_to_alphabeta(phases);
    REQUIRE_THAT(ab.alpha, WithinAbs(0.f, kTol));
    REQUIRE_THAT(ab.beta,  WithinAbs(0.f, kTol));
}

// --- Inverse Clarke (alpha/beta -> phases) ------------------------------------

TEST_CASE("Inverse Clarke: round-trip is identity", "[clarke]")
{
    PhaseValues<float> original{0.6f, -0.2f, -0.4f};
    auto ab     = phases_to_alphabeta(original);
    auto result = alphabeta_to_phases(ab);
    REQUIRE_THAT(result.a, WithinAbs(original.a, kTol));
    REQUIRE_THAT(result.b, WithinAbs(original.b, kTol));
    REQUIRE_THAT(result.c, WithinAbs(original.c, kTol));
}

TEST_CASE("Inverse Clarke: output phases sum to zero", "[clarke]")
{
    // Reconstructed balanced phases must still sum to zero.
    AlphaBetaValues<float> ab{0.8f, 0.3f};
    auto phases = alphabeta_to_phases(ab);
    REQUIRE_THAT(phases.a + phases.b + phases.c, WithinAbs(0.f, kTol));
}

// --- Park (alpha/beta -> q/d) -------------------------------------------------

TEST_CASE("Park: at eangle=0, alpha maps to d and beta maps to q", "[park]")
{
    // At theta=0: sin=0, cos=1
    // Park: d = cos*alpha + sin*beta = alpha
    //       q = -sin*alpha + cos*beta = beta
    AlphaBetaValues<float> ab{0.7f, 0.3f};
    auto qd = alphabeta_to_quaddirect(ab, 0.f);
    REQUIRE_THAT(qd.d, WithinAbs(ab.alpha, kTol));
    REQUIRE_THAT(qd.q, WithinAbs(ab.beta,  kTol));
}

TEST_CASE("Park: at eangle=pi/2, alpha maps to -q and beta maps to d", "[park]")
{
    // At theta=pi/2: sin=1, cos=0
    // d = cos*alpha + sin*beta = beta
    // q = -sin*alpha + cos*beta = -alpha
    AlphaBetaValues<float> ab{0.5f, 0.4f};
    auto qd = alphabeta_to_quaddirect(ab, static_cast<float>(M_PI) / 2.f);
    REQUIRE_THAT(qd.d, WithinAbs(ab.beta,   kTol));
    REQUIRE_THAT(qd.q, WithinAbs(-ab.alpha, kTol));
}

TEST_CASE("Park/inverse-Park: round-trip is identity", "[park]")
{
    AlphaBetaValues<float> original{0.6f, -0.3f};
    float eangle = 1.1f;
    auto qd     = alphabeta_to_quaddirect(original, eangle);
    auto result = quaddirect_to_alphabeta(qd, eangle);
    REQUIRE_THAT(result.alpha, WithinAbs(original.alpha, kTol));
    REQUIRE_THAT(result.beta,  WithinAbs(original.beta,  kTol));
}

// --- Full pipeline (phases -> q/d -> phases) ----------------------------------

TEST_CASE("Full pipeline round-trip preserves balanced phases", "[pipeline]")
{
    PhaseValues<float> original{1.f, -0.5f, -0.5f};
    float eangle = 0.42f;
    auto qd     = phases_to_quaddirect(original, eangle);
    auto result = quaddirect_to_phases(qd, eangle);
    REQUIRE_THAT(result.a, WithinAbs(original.a, kTol));
    REQUIRE_THAT(result.b, WithinAbs(original.b, kTol));
    REQUIRE_THAT(result.c, WithinAbs(original.c, kTol));
}

TEST_CASE("Full pipeline: output magnitude preserved under rotation", "[pipeline]")
{
    // The Park transform is orthonormal: sqrt(q^2 + d^2) == sqrt(alpha^2 + beta^2)
    PhaseValues<float> phases{0.8f, -0.3f, -0.5f};
    float eangle = 2.1f;
    auto ab = phases_to_alphabeta(phases);
    auto qd = alphabeta_to_quaddirect(ab, eangle);
    float mag_ab = std::sqrt(ab.alpha * ab.alpha + ab.beta * ab.beta);
    float mag_qd = std::sqrt(qd.q * qd.q + qd.d * qd.d);
    REQUIRE_THAT(mag_qd, WithinAbs(mag_ab, kTol));
}
