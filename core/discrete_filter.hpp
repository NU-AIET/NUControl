#ifndef DISCRETE_FILTER_HPP
#define DISCRETE_FILTER_HPP
#include <vector>
#include <deque>
#include "helpers.hpp"

template<typename T, typename G>
class DiscreteFilter
{
public:
  DiscreteFilter() = default;
  ~DiscreteFilter() = default;

  DiscreteFilter(std::vector<G> input_coeffs, std::vector<G> output_coeffs)
  : input_coeffs_(input_coeffs),
    output_coeffs_(output_coeffs),
    inputs_num_(input_coeffs.size()),
    outputs_num_(output_coeffs.size())
  {
    inputs_ = std::deque<T>(inputs_num_, static_cast<T>(0.f));
    outputs_ = std::deque<T>(outputs_num_, static_cast<T>(0.f));
  }

  virtual T update(T new_input)
  {
    // Add new sample, remove oldest sample
    inputs_.push_front(new_input);
    inputs_.pop_back();

    T sum_ = static_cast<T>(0.f);
    for (size_t i = 0; i < inputs_num_; ++i) {
      sum_ += (input_coeffs_.at(i) * inputs_.at(i));
    }

    if(outputs_num_ == 0) {
      return sum_;
    }

    for (size_t j = 0; j < outputs_num_; ++j) {
      sum_ -= (output_coeffs_.at(j) * outputs_.at(j));
    }

    // Add newest command, remove oldest command
    outputs_.push_front(sum_);
    outputs_.pop_back();

    return sum_;
  }

  virtual void reset(T input_fill = static_cast<T>(0.f), T output_fill = static_cast<T>(0.f))
  {
    inputs_ = std::deque<T>(inputs_num_, static_cast<T>(input_fill));
    outputs_ = std::deque<T>(outputs_num_, static_cast<T>(output_fill));
  }

private:
  std::vector<G> input_coeffs_;
  std::vector<G> output_coeffs_;

  size_t inputs_num_;
  size_t outputs_num_;
  std::deque<T> inputs_;
  std::deque<T> outputs_;
};


template<typename T>
class Butterworth2nd : public DiscreteFilter<T, float>
{
public:
  Butterworth2nd() = default;
  ~Butterworth2nd() = default;

  Butterworth2nd(float cutoff_hz, float sampling_hz)
  : DiscreteFilter<T, float>(input_coeffs(cutoff_hz, sampling_hz),
      output_coeffs(cutoff_hz, sampling_hz))
  {}

  // T update(T new_input) {DiscreteFilter<T>::update(new_input);}

private:
  std::vector<float> output_coeffs(float f_c, float f_s)
  {
    T alpha = 1.f / tanf(M_PI * f_c / f_s);
    T beta = alpha * alpha + _SQRT_2_ * alpha + 1;

    return {(-2.f * alpha * alpha + 2.f) / beta, (alpha * alpha - _SQRT_2_ * alpha + 1.f) / beta};

  }

  std::vector<float> input_coeffs(float f_c, float f_s)
  {
    T alpha = 1.f / tanf(M_PI * f_c / f_s);
    T beta = alpha * alpha + _SQRT_2_ * alpha + 1;

    return {1.f / beta, 2.f / beta, 1.f / beta};

  }

};

template<typename T>
class PIController : public DiscreteFilter<T, float>
{
public:
  PIController() = default;
  ~PIController() = default;

  PIController(float Kp, float Ki, float control_period_s)
  : DiscreteFilter<T, float>({Kp + 0.5f * Ki * control_period_s,
        -Kp + 0.5f * Ki * control_period_s},
      {-1.f})
  {}
};

class MotorFeedforward : public DiscreteFilter<PhaseValues<float>, float>
{
  MotorFeedforward() = default;
  ~MotorFeedforward() = default;

  MotorFeedforward(float phase_R, float phase_L, float control_period_s)
  : DiscreteFilter<PhaseValues<float>, float>({2.f * phase_L / control_period_s + phase_R,
        -2.f * phase_L / control_period_s + phase_R}, {1.f})
  {}

};

#endif
