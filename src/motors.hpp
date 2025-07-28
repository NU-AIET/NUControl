#ifndef MOTORS_HPP
#define MOTORS_HPP

struct MotorParameters
{
  int pole_pairs;
  float phase_R;   // Ohm = Single Phase Resistance
  float phase_L;   // Ohm *s = Henry = Single Phase Inductance
  float SAFE_CURRENT;   // A = I can use this all the time and it will be okay
  float MAX_CURRENT;   // A = I can only use this for a short period
  float kT;   // Nm / A
  float kV;   // V / rad /s
};

// Note: Maybe mulitply kV by 0.5;
MotorParameters EC45_Flat{8, 0.2992f, 1.0f * 111.f * 1e-6, 3.f, 8.f, 0.034, 0.0369};






#endif