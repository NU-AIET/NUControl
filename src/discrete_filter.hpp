#ifndef DISCRETE_FILTER_HPP
#define DISCRETE_FILTER_HPP
#include <vector>
#include <deque>
#include "helpers.hpp"

template <typename T>
class DiscreteFilter
{
  public:
    DiscreteFilter() = default;
    ~DiscreteFilter() = default;

    DiscreteFilter(std::vector<T> input_coeffs, std::vector<T> output_coeffs)
    : input_coeffs_(input_coeffs),
      output_coeffs_(output_coeffs),
      inputs_num_(input_coeffs.size()),
      outputs_num_(output_coeffs.size())
    {
        inputs_ = std::deque<T>(inputs_num_, static_cast<T>(0.f));
        outputs_ = std::deque<T>(outputs_num_, static_cast<T>(0.f));
    }

    T update(T new_input)
    {
        // Add new sample, remove oldest sample
        inputs_.push_front();
        inputs_.pop_back();

        T sum_ = static_cast<T>(0.f);
        for (size_t i = 0; i < inputs_num_; ++i) {
            sum_ += (input_coeffs_.at(i) * inputs_.at(i));
        }

        for (size_t j = 0; j < outputs_num_; ++j) {
            sum_ -= (output_coeffs_.at(j) * outputs_.at(j));
        }

        // Add newest command, remove oldest command
        outputs_.push_front(sum_);
        outputs_.pop_back();

        return sum_;
    }

  private:
    const std::vector<T> input_coeffs_;
    const std::vector<T> output_coeffs_;
    
    const size_t inputs_num_;
    const size_t outputs_num_;
    std::deque<T> inputs_;
    std::deque<T> outputs_;
};


template <typename T>
class Buttersworth2nd : public DiscreteFilter<T>
{
  public:
    Buttersworth2nd() = default;
    ~Buttersworth2nd() = default;

    Buttersworth2nd(T cutoff_hz, T sampling_hz)
    : DiscreteFilter<T>(input_coeffs(cutoff_hz, sampling_hz), output_coeffs(cutoff_hz, sampling_hz))
    {}

    T update(T new_input) {DiscreteFilter<T>::update(new_input);}

  private:
    std::vector<T> output_coeffs(T f_c, T f_s) {
        T alpha = 1.f / tanf(M_PI * f_c / f_s);
        T beta = alpha * alpha + _SQRT_2_ * alpha + 1;

        return {(-2.f * alpha * alpha + 2.f) / beta, (alpha * alpha - _SQRT_2_ * alpha + 1.f) / beta};

    }

    std::vector<T> input_coeffs(T f_c, T f_s) {
        T alpha = 1.f / tanf(M_PI * f_c / f_s);
        T beta = alpha * alpha + _SQRT_2_ * alpha + 1;
        
        return {1.f / beta, 2.f / beta, 1.f / beta};
        
    }

};


#endif