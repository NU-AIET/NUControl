#ifndef TRANSFORMATION_HPP
#define TRANSFORMATION_HPP
#include "helpers.h"

template<typename T>
struct PhaseValues
{
  T a;
  T b;
  T c;
};

template<typename T>
struct AlphaBetaValues
{
  T alpha;
  T beta;
};

template<typename T>
struct QuadDirectValues
{
  T q;
  T d;
};

template<typename T>
PhaseValues<T> operator*(PhaseValues<T> phs, T s)
{
  return {s * phs.a, s * phs.b, s * phs.c};
}

template<typename T>
PhaseValues<T> operator*(T s, PhaseValues<T> phs)
{
  return {s * phs.a, s * phs.b, s * phs.c};
}

template<typename T>
PhaseValues<T> operator+(PhaseValues<T> phs_a, PhaseValues<T> phs_b)
{
  return {phs_a.a + phs_b.a, phs_a.b + phs_b.b, phs_a.c + phs_b.c};
}

template<typename T>
PhaseValues<T> operator-(PhaseValues<T> phs_a, PhaseValues<T> phs_b)
{
  return {phs_a.a - phs_b.a, phs_a.b - phs_b.b, phs_a.c - phs_b.c};
}


template<typename T>
QuadDirectValues<T> operator*(QuadDirectValues<T> qd, T s)
{
  return {s * qd.q, s * qd.d};
}

template<typename T>
QuadDirectValues<T> operator*(T s, QuadDirectValues<T> qd)
{
  return {s * qd.q, s * qd.d};
}


template<typename T>
QuadDirectValues<T> operator+(QuadDirectValues<T> qd_a, QuadDirectValues<T> qd_b)
{
  return {qd_a.q + qd_b.q, qd_a.d + qd_b.d};
}

template<typename T>
QuadDirectValues<T> operator-(QuadDirectValues<T> qd_a, QuadDirectValues<T> qd_b)
{
  return {qd_a.q - qd_b.q, qd_a.d - qd_b.d};
}


template<typename T>
AlphaBetaValues<T> phases_to_alphabeta(PhaseValues<T> phases)
{
  AlphaBetaValues<T> alphabeta;
  alphabeta.alpha = phases.a;
  alphabeta.beta = _1__SQRT_3_ * phases.a + _2__SQRT_3_ * phases.b;
  return alphabeta;
}

template<typename T>
PhaseValues<T> alphabeta_to_phases(AlphaBetaValues<T> alphabeta)
{
  PhaseValues<T> phases;
  phases.a = alphabeta.alpha;
  phases.b = -0.5 * alphabeta.alpha + _SQRT_3__2_ * alphabeta.beta;
  phases.c = -0.5 * alphabeta.alpha - _SQRT_3__2_ * alphabeta.beta;
  return phases;
}
template<typename T>
QuadDirectValues<T> alphabeta_to_quaddirect(AlphaBetaValues<T> alphabeta, float eangle_rads)
{
  auto sc = sincos(eangle_rads);
  return {-sc.first * alphabeta.alpha + sc.second * alphabeta.beta,
    sc.second * alphabeta.alpha + sc.first * alphabeta.beta};
}

template<typename T>
AlphaBetaValues<T> quaddirect_to_alphabeta(QuadDirectValues<T> quaddirect, float eangle_rads)
{
  auto sc = sincos(eangle_rads);
  return {sc.second * quaddirect.d - sc.first * quaddirect.q,
    sc.first * quaddirect.d + sc.second * quaddirect.q};
}

template<typename T>
PhaseValues<T> quaddirect_to_phases(QuadDirectValues<T> quaddirect, float eangle_rads)
{
  return alphabeta_to_phases(quaddirect_to_alphabeta(quaddirect, eangle_rads));
}

template<typename T>
QuadDirectValues<T> phases_to_quaddirect(PhaseValues<T> phases, float eangle_rads)
{
  return alphabeta_to_quaddirect(phases_to_alphabeta(phases), eangle_rads);
}


#endif
