/**
 * @file motors.hpp
 * @brief Motor electrical parameter structs and pre-defined motor instances.
 *
 * @details `MotorParameters` aggregates all per-motor constants needed by the
 * FOC controller. Pre-defined instances are provided for the motors used in
 * the NUControl exoskeleton platform.
 *
 * ## kT / kV relationship
 * For a sinusoidal BLDC motor in the amplitude-invariant Park frame:
 * @f[ k_T \approx \frac{1}{k_V \cdot \sqrt{3/2}} @f]
 * The U2523/U2535 entries have an unusually small `kV` (0.001 V·s/rad), which
 * may require scaling by 0.5 — see the `@todo` note on `EC45_Flat`.
 */

#ifndef MOTORS_HPP
#define MOTORS_HPP

/**
 * @brief Complete set of electrical, mechanical, and thermal parameters for one motor.
 *
 * @details All resistance and inductance values are **single-phase** quantities
 * (not phase-to-phase). The controller's internal RL model uses these directly:
 * @f[ V = R \cdot I + L \cdot \frac{dI}{dt} + V_{\text{BEMF}} @f]
 *
 * ## Thermal parameters
 * The four thermal fields parameterise the two-node thermal model in
 * `TwoNodeThermalModel` (winding → case → ambient):
 *
 * | Field                | Symbol          | Typical range  | Notes                                |
 * |----------------------|-----------------|----------------|--------------------------------------|
 * | `R_th_winding_case`  | @f$R_{wc}@f$    | 0.5 – 5 °C/W   | From datasheet or thermal measurement |
 * | `C_th_winding`       | @f$C_w@f$       | 5 – 50 J/°C    | Winding thermal mass                 |
 * | `R_th_case_ambient`  | @f$R_{ca}@f$    | 1 – 20 °C/W    | Depends heavily on mounting / airflow |
 * | `C_th_case`          | @f$C_c@f$       | 50 – 500 J/°C  | Housing/heatsink thermal mass        |
 *
 * Default values of 0 and 1 are intentional no-ops that produce an unbounded
 * temperature rise — replace them with real values before enabling
 * `BrushlessController::enable_thermal_limiting()`.
 *
 * @note `R_th_winding_case` and `C_th_winding` are the most critical parameters
 *       for protecting against fast I²R transients.  `R_th_case_ambient` and
 *       `C_th_case` govern long-term steady-state temperature.
 */
struct MotorParameters
{
  int   pole_pairs;     ///< Number of pole pairs (poles / 2).
  float phase_R;        ///< Single-phase winding resistance [Ω]. Not phase-to-phase.
  float phase_L;        ///< Single-phase winding inductance [H]. Not phase-to-phase.
  float SAFE_CURRENT;   ///< Continuous safe current [A] — motor will not overheat at this level.
  float MAX_CURRENT;    ///< Peak current [A] — safe for short durations only.
  float kT;             ///< Torque constant [Nm/A] — torque = kT × I_q.
  float kV;             ///< Back-EMF constant [V·s/rad] — BEMF = kV × ω.

  // -------------------------------------------------------------------------
  // Thermal parameters (used by TwoNodeThermalModel in thermal_model.hpp)
  // -------------------------------------------------------------------------
  float R_th_winding_case = 0.f;  ///< Thermal resistance winding → case [°C/W].
  float C_th_winding      = 1.f;  ///< Thermal capacitance of winding [J/°C].
  float R_th_case_ambient = 0.f;  ///< Thermal resistance case → ambient [°C/W].
  float C_th_case         = 1.f;  ///< Thermal capacitance of case/housing [J/°C].
  float T_max_winding     = 130.f;///< Maximum safe winding temperature [°C].
                                   ///< Derating reaches 0 A at this value.
                                   ///< Typical: 130 °C (class B insulation),
                                   ///<           155 °C (class F), 180 °C (class H).
};

/**
 * @brief Maxon EC45 Flat BLDC motor parameters.
 *
 * @note `kV = 0.0369 V·s/rad` matches the manufacturer datasheet.
 *       The commented-out `kV = 0` variant disables back-EMF compensation.
 *
 * @todo Verify whether `kV` should be multiplied by 0.5 for the amplitude-invariant
 *       Park-frame convention used by this controller. Forum discussions suggest the
 *       scaling factor depends on the chosen normalisation; the back-EMF decoupler
 *       already uses `0.5 * kV * ω`, which may or may not be correct for this motor.
 */
MotorParameters EC45_Flat{8, 0.2992f, 111.f * 1e-6f, 3.f, 8.f, 0.034f, 0.0369};
// MotorParameters EC45_Flat{8, 0.2992f, 0.f, 3.f, 8.f, 0.034f, 0.0369};

/**
 * @brief T-Motor U2535 BLDC motor parameters.
 *
 * @details Used for the proximal joint of the NUControl exoskeleton.
 * High inductance (510 µH) results in relatively slow electrical dynamics
 * (RL time constant τ = L/R ≈ 700 µs ≈ 7 control steps at 10 kHz).
 *
 * @note `kV = 0.001 V·s/rad` is very small; the back-EMF contribution is
 *       negligible at typical operating speeds (<30 rad/s mechanical).
 */
MotorParameters U2535{7, 0.72487f, 510.f * 1e-6f, 3.f, 9.f, 0.040f, 0.001f};
// MotorParameters U2535{7, 0.72487f, 0.f, 3.f, 9.f, 0.040f, 0.001f};


#endif // MOTORS_HPP
