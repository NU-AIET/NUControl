#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP
#include <Arduino.h>
#include "brushless_controller.hpp"

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
    stream.print(F("{\"t\":"));
    stream.print(t_us);
    stream.print(F(",\"pos\":"));
    stream.print(pos, 4);
    stream.print(F(",\"vel\":"));
    stream.print(vel, 4);
    stream.print(F(",\"target\":"));
    stream.print(target, 4);
    stream.print(F(",\"ia\":"));
    stream.print(phase_currents.a, 4);
    stream.print(F(",\"ib\":"));
    stream.print(phase_currents.b, 4);
    stream.print(F(",\"ic\":"));
    stream.print(phase_currents.c, 4);
    stream.print(F(",\"iq\":"));
    stream.print(qd_currents.q, 4);
    stream.print(F(",\"id\":"));
    stream.print(qd_currents.d, 4);
    stream.println(F("}"));
  }
};

#endif
