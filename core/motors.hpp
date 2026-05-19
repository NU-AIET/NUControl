#ifndef MOTORS_HPP
#define MOTORS_HPP

struct MotorParameters
{
  int pole_pairs;
  float phase_R;   // Ohm : Single Phase Resistance (not Phase to Phase)
  float phase_L;   // Ohm *s = Henry : Single Phase Inductance (not Phase to Phase)
  float SAFE_CURRENT;   // A : I can use this all the time and it will be okay
  float MAX_CURRENT;   // A : I can only use this for a short period
  float kT;   // Nm / A : How much torque per amp
  float kV;   // V / rad /s : Backemf
};

// Note: Maybe mulitply kV by 0.5;
MotorParameters EC45_Flat{8, 0.2992f, 111.f * 1e-6f, 3.f, 8.f, 0.034f, 0.0369};
// MotorParameters EC45_Flat{8, 0.2992f, 0.f, 3.f, 8.f, 0.034f, 0.0369};

MotorParameters U2535{7, 0.72487f, 510.f * 1e-6f, 3.f, 9.f, 0.040f, 0.001f};
// MotorParameters U2535{7, 0.72487f, 0.f, 3.f, 9.f, 0.040f, 0.001f};


#endif
