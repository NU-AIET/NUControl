#ifndef CURRENT_SENSOR_INTERFACE_HPP
#define CURRENT_SENSOR_INTERFACE_HPP
#include "transformations.hpp"
#include "discrete_filter.hpp"

class IBrushlessDriver;

class ICurrentSensorPackage
{
public:
  virtual ~ICurrentSensorPackage() = default;
  virtual bool init_sensors() = 0;
  virtual PhaseValues<float> get_phase_currents(bool filter = true) = 0;
  virtual void set_filters(DiscreteFilter<float, float> filter) = 0;
  virtual void print_calibration() = 0;
  virtual bool align_sensors(IBrushlessDriver & driver, float align_volts = 0.5f) = 0;
  virtual bool load_calibration(PhaseValues<int> phase_idx, PhaseValues<int> phase_dirs) = 0;
};

#endif
