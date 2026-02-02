#ifndef CHIRP_CONTROL_HPP
#define CHIRP_CONTROL_HPP
#include "brushless_controller.hpp"


class ChirpController
{

public:
  ChirpController() = default;
  ~ChirpController() = default;

  void link_controller(BrushedController & controller)
  {
    controller_ = &controller;
  }

  void start_voltage_chirp(float freg_offset, float freq_gain)
  {

  }

private:
  BrushedController * controller_;


};

#endif
