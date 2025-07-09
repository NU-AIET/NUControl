#ifndef FILTER_HPP
#define FILTER_HPP
#include "helpers.hpp"
#include <vector>
#include <deque>

class Butterworth2
{
public:
  Butterworth2() = default;
  ~Butterworth2() = default;

  Butterworth2(float cutoff_hz, float sampling_hz)
  {
    // https://tttapa.github.io/Pages/Mathematics/Systems-and-Control-Theory/Digital-filters/Discretization/Discretization-of-a-fourth-order-Butterworth-filter.html
    const auto gamma = 1.f / tanf(M_PI * cutoff_hz / sampling_hz);

    const auto temp = gamma * gamma + _SQRT_2_ * gamma + 1;

    a_s_.at(0) = (-2.f * gamma * gamma + 2) / temp;
    a_s_.at(1) = (gamma * gamma - _SQRT_2_ * gamma + 1) / temp;

    b_s_.at(0) = 1 / temp;
    b_s_.at(1) = 2 / temp;
    b_s_.at(2) = 1 / temp;
  }

  float filter(float sample)
  {
    old_samples_.push_front(sample);
    old_samples_.pop_back();

    auto top_sum = old_samples_.at(0) * b_s_.at(0) + old_samples_.at(1) * b_s_.at(1) +
      old_samples_.at(2) * b_s_.at(2);
    auto bot_sum = old_filtered_.at(0) * a_s_.at(0) + old_filtered_.at(1) * a_s_.at(1);

    const auto new_filt = top_sum - bot_sum;

    old_filtered_.push_front(new_filt);
    old_filtered_.pop_back();

    return new_filt;
  }

private:
  std::deque<float> old_filtered_{0.f, 0.f};
  std::vector<float> a_s_{0.f, 0.f};

  std::deque<float> old_samples_{0.f, 0.f, 0.f};
  std::vector<float> b_s_{0.f, 0.f, 0.f};


};


#endif
