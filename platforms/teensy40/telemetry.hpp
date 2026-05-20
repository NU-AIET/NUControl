#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP
#include <Arduino.h>
#include "../../core/brushless_controller.hpp"

// Snapshot of motor state for live plotting over SerialUSB1 (USB dual-serial).
// Usage: call populate() each time you want a sample, then serialize() to emit
// a JSON line that PlotJuggler / Foxglove can parse directly.
struct MotorTelemetry
{
  uint32_t t_us = 0;
  float pos = 0.f;                              // shaft angle, rad
  float vel = 0.f;                              // shaft velocity, rad/s
  float target = 0.f;                           // current setpoint (units depend on mode)
  PhaseValues<float> phase_currents{0.f, 0.f, 0.f}; // ia, ib, ic in Amps
  QuadDirectValues<float> qd_currents{0.f, 0.f};    // iq, id in Amps

  void populate(const BrushlessController & ctrl)
  {
    t_us = micros();
    pos = ctrl.get_shaft_angle();
    vel = ctrl.get_shaft_velocity();
    target = ctrl.get_target();
    phase_currents = ctrl.get_phase_currents();
    qd_currents = ctrl.get_quaddirect_currents();
  }

  void serialize(Stream & stream) const
  {
    char buf[192];
    snprintf(buf, sizeof(buf),
      "{\"t\":%lu,\"pos\":%.4f,\"vel\":%.4f,\"target\":%.4f"
      ",\"ia\":%.4f,\"ib\":%.4f,\"ic\":%.4f,\"iq\":%.4f,\"id\":%.4f}\n",
      (unsigned long)t_us, pos, vel, target,
      phase_currents.a, phase_currents.b, phase_currents.c,
      qd_currents.q, qd_currents.d);
    stream.print(buf);
  }
};

#endif
