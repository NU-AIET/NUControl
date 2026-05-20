#ifndef DRIVER_INTERFACE_HPP
#define DRIVER_INTERFACE_HPP
#include "transformations.hpp"

class IBrushlessDriver
{
public:
  virtual ~IBrushlessDriver() = default;
  virtual bool init() = 0;
  virtual void enable() = 0;
  virtual void disable() = 0;
  virtual PhaseValues<int> set_phase_voltages(PhaseValues<float> voltages) = 0;
};

#endif
