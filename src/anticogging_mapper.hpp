#ifndef ANTICOGGING_HPP
#define ANTICOGGING_HPP
#include "helpers.hpp"
#include <cmath>
#include "transformations.hpp"

const size_t COGGING_STEPS = 1000;

std::vector<float> default_anticog_map(COGGING_STEPS, 0.f);
PhaseValues<std::vector<float>> default_anticog_volt_map{std::vector<float>(COGGING_STEPS, 0.f),std::vector<float>(COGGING_STEPS, 0.f),std::vector<float>(COGGING_STEPS, 0.f)};

float get_cogging_torque(const float rads, const std::vector<float> & anticog_map)
{

  float ang = normalize_angle(rads);
  if(ang < 0) { ang += _2_PI_; }

  float idx = (static_cast<float>(COGGING_STEPS) / _2_PI_ * ang);
  const int idl = (floor(idx) == static_cast<float>(COGGING_STEPS)) ? 0 : static_cast<int>(floor(idx));
  const int idu = (idl == static_cast<int>(COGGING_STEPS) - 1) ? 0 : idl + 1;

  const float alpha = ceil(idx) - idx;
  const float beta = 1 - alpha;

  return anticog_map.at(idl) * alpha + anticog_map.at(idu) * beta;
}


PhaseValues<float> get_cogging_voltage(const float rads, const PhaseValues<std::vector<float>> & anticog_map)
{
  float ang = normalize_angle(rads);
  if(ang < 0) { ang += _2_PI_; }

  float idx = (static_cast<float>(COGGING_STEPS) / _2_PI_ * ang);
  const int idl = (floor(idx) == static_cast<float>(COGGING_STEPS)) ? 0 : static_cast<int>(floor(idx));
  const int idu = (idl == static_cast<int>(COGGING_STEPS) - 1) ? 0 : idl + 1;

  const float alpha = ceil(idx) - idx;
  const float beta = 1 - alpha;

  return {anticog_map.a.at(idl) * alpha + anticog_map.a.at(idu) * beta,
          anticog_map.b.at(idl) * alpha + anticog_map.b.at(idu) * beta,
          anticog_map.c.at(idl) * alpha + anticog_map.c.at(idu) * beta};
}

#endif
