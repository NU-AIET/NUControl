#ifndef ANTICOG_HELPERS_HPP
#define ANTICOG_HELPERS_HPP
#include "helpers.hpp"
#include <cmath>
#include <array>
#include "transformations.hpp"

inline std::array<float, 0> default_anticog_torque_map{};
inline PhaseValues<std::array<float, 0>> default_anticog_volt_map{};

template<std::size_t steps_>
class AnticoggingCompensator{
  public:
    ~AnticoggingCompensator() = default;

    AnticoggingCompensator(const std::array<float, steps_> & anticog_torque_map)
    : anticog_torque_map_(anticog_torque_map),
      anticog_volt_map_(empty_volt_map())
    {}

    AnticoggingCompensator(const PhaseValues<std::array<float, steps_>> & anticog_volt_map)
    : anticog_torque_map_(empty_torque_map()),
      anticog_volt_map_(anticog_volt_map)
    {}

    AnticoggingCompensator(const std::array<float, steps_> & anticog_torque_map, PhaseValues<std::array<float, steps_>> & anticog_volt_map)
    : anticog_torque_map_(anticog_torque_map),
      anticog_volt_map_(anticog_volt_map)
    {}

  float get_cogging_torque(const float rads) const
  {
    if(steps_ == 0){
      return 0.f;
    }
    float ang = normalize_angle(rads);
    if(ang < 0) { ang += _2_PI_; }

    float idx = (static_cast<float>(steps_) / _2_PI_ * ang);
    const int idl = (floor(idx) == static_cast<float>(steps_)) ? 0 : static_cast<int>(floor(idx));
    const int idu = (idl == static_cast<int>(steps_) - 1) ? 0 : idl + 1;

    // weight_l: fraction of the interval remaining to the upper index (weights lower)
    // weight_u: fraction of the interval consumed from the lower index (weights upper)
    const float weight_u = idx - floor(idx);
    const float weight_l = 1.f - weight_u;

    return anticog_torque_map_.at(idl) * weight_l + anticog_torque_map_.at(idu) * weight_u;
  }

  PhaseValues<float> get_cogging_voltage(const float rads)
  {
    if(steps_ == 0){
      return {0.f, 0.f, 0.f};
    }
    float ang = normalize_angle(rads);
    if(ang < 0) { ang += _2_PI_; }

    float idx = (static_cast<float>(steps_) / _2_PI_ * ang);
    const int idl = (floor(idx) == static_cast<float>(steps_)) ? 0 : static_cast<int>(floor(idx));
    const int idu = (idl == static_cast<int>(steps_) - 1) ? 0 : idl + 1;

    const float weight_u = idx - floor(idx);
    const float weight_l = 1.f - weight_u;

    return {anticog_volt_map_.a.at(idl) * weight_l + anticog_volt_map_.a.at(idu) * weight_u,
            anticog_volt_map_.b.at(idl) * weight_l + anticog_volt_map_.b.at(idu) * weight_u,
            anticog_volt_map_.c.at(idl) * weight_l + anticog_volt_map_.c.at(idu) * weight_u};
  }

  private:
    const std::array<float, steps_> & anticog_torque_map_;
    const PhaseValues<std::array<float, steps_>> & anticog_volt_map_;

    static const std::array<float, steps_> & empty_torque_map() {
      static const std::array<float, steps_> empty{};
      return empty;
    }

    static const PhaseValues<std::array<float, steps_>> & empty_volt_map() {
      static const PhaseValues<std::array<float, steps_>> empty{};
      return empty;
    }


};


#endif
