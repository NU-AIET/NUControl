#ifndef CURRENT_SENSE_HPP
#define CURRENT_SENSE_HPP
#include <vector>
#include <Arduino.h>
#include "driver.hpp"
#include "helpers.hpp"
#include "filter.hpp"


/// @brief Inline current sensor placed on a phase of a motor
class InlineCurrentSensor
{
public:
  InlineCurrentSensor() = default;
  ~InlineCurrentSensor() = default;

  /// @brief
  /// @param pin - Which pin to analogRead for the current data
  /// @param amps_per_volt - how manys amps / volt from the ADC
  /// @note Sets warning when current exceeds 1.5 * the gain, (Sensor approaches is near maximum read (1.5 / 1.65))
  InlineCurrentSensor(int pin, float amps_per_volt, int ADC_res = 10)
  : pin_(pin),
    gain_(amps_per_volt),
    MAX_READING_(1.5f * gain_),
    ADC_GAIN_(3.3f / (1 << ADC_res))
  {
    analogReadRes(ADC_res);
  }

  InlineCurrentSensor(int pin, float shunt_resistance_ohms, float op_amp_gain, int ADC_res = 10)
  : pin_(pin),
    gain_(1.f / (shunt_resistance_ohms * op_amp_gain)),
    MAX_READING_(1.5f * gain_),
    ADC_GAIN_(3.3f / (1 << ADC_res))
  {
    analogReadRes(ADC_res);
  }

  bool init_sensor() const
  {
    pinMode(pin_, INPUT);
    return validate_offset();
  }

  float read() const
  {
    auto amps = gain_ * (analogRead(pin_) * ADC_GAIN_ - offset_);
    if (fabs(amps) > MAX_READING_) {
      Serial.println("Current Sensor Saturated!");
    }
    return amps;
  }

  void set_filter(Butterworth2 filter)
  {
    filter_ = filter;
  }

  float read_filtered()
  {
    return filter_.filter(read());
  }

private:
  const int pin_;
  const float gain_;       //A / V_adc
  float offset_ = 1.65f;       // Volts
  const float MAX_READING_; // A
  Butterworth2 filter_;


  const float ADC_GAIN_;

  bool validate_offset(size_t n = 10000) const
  {
    int sum_ = 0;
    for (size_t i = 0; i < n; ++i) {
      sum_ += analogRead(pin_);
    }
    float offset = static_cast<float>(sum_) / n * ADC_GAIN_;

    // Serial.println(offset);

    // Uh oh!
    if (fabs(offset_ - offset) > 1e-2) {
      return false;
    }
    return true;

  }

};

class InlineCurrentSensorPackage
{
public:
  InlineCurrentSensorPackage() = default;
  ~InlineCurrentSensorPackage() = default;

  InlineCurrentSensorPackage(std::vector<InlineCurrentSensor *> sensors)
  : sensors_(sensors),
    num_sensors_(sensors_.size())
  {
    Serial.println(num_sensors_);
    if (num_sensors_ < 2) {
      //Bad bad bad!
      return;
    }
  }

  bool init_sensors()
  {
    bool all_inited = true;
    for (size_t i = 0; i < num_sensors_; ++i) {
      all_inited &= sensors_.at(i)->init_sensor();
    }
    return all_inited;
  }

  bool align_sensors(Driver & driver, float align_volts = 0.5f)
  {

    driver.enable();

    driver.set_phase_voltages({align_volts, 0, 0});
    delay(100);
    auto reads_a = read_sensors();
    driver.set_phase_voltages({0, 0, 0});
    delay(100);

    driver.set_phase_voltages({0, align_volts, 0});
    delay(100);
    auto reads_b = read_sensors();
    driver.set_phase_voltages({0, 0, 0});
    delay(100);

    driver.set_phase_voltages({0, 0, align_volts});
    delay(100);
    auto reads_c = read_sensors();
    driver.set_phase_voltages({0, 0, 0});

    driver.disable();

    std::vector<PhaseValues<float>> sensor_readings;

    for (size_t i = 0; i < num_sensors_; ++i) {
      Serial.print("Sensor number ");
      Serial.println(i);
      Serial.print(reads_a.at(i));
      Serial.print("\t");
      Serial.print(reads_b.at(i));
      Serial.print("\t");
      Serial.println(reads_c.at(i));

      sensor_readings.push_back({reads_a.at(i), reads_b.at(i), reads_c.at(i)});
    }

    for (size_t i = 0; i < num_sensors_; ++i) {
      const auto max_ =
        max(
        fabs(sensor_readings.at(i).a),
        max(fabs(sensor_readings.at(i).b), fabs(sensor_readings.at(i).c)));
      // const auto min_ =
      //   min(
      //   fabs(sensor_readings.at(i).a),
      //   min(fabs(sensor_readings.at(i).b), fabs(sensor_readings.at(i).c)));

      if (max_ < 0.05f) {
        Serial.print("No current detected on sensor number ");
        Serial.println(i);
        Serial.print("Read Amps: ");
        Serial.println(max_);
        // This sensor doesn't read anything!!
        return false;
      }

      if (max_ == fabs(sensor_readings.at(i).a)) {
        phase_idx_.a = i;
        if (max_ > sensor_readings.at(i).a) {
          phase_dirs_.a = -1;
        } else {
          phase_dirs_.a = 1;
        }
        continue;
      }

      if (max_ == fabs(sensor_readings.at(i).b)) {
        phase_idx_.b = i;
        if (max_ > sensor_readings.at(i).b) {
          phase_dirs_.b = -1;
        } else {
          phase_dirs_.b = 1;
        }
        continue;
      }

      if (max_ == fabs(sensor_readings.at(i).c)) {
        phase_idx_.c = i;
        if (max_ > sensor_readings.at(i).c) {
          phase_dirs_.c = -1;
        } else {
          phase_dirs_.c = 1;
        }
        continue;
      }
    }

    Serial.println("Phase Indicies: ");
    Serial.println(phase_idx_.a);
    Serial.println(phase_idx_.b);
    Serial.println(phase_idx_.c);

    Serial.println("Phase Directions: ");
    Serial.println(phase_dirs_.a);
    Serial.println(phase_dirs_.b);
    Serial.println(phase_dirs_.c);

    aligned_ = true;
    return aligned_;

  }

  PhaseValues<float> get_phase_currents(bool filter = true)
  {
    if (!aligned_) {
      Serial.println("Sensor Package Not Aligned To Driver");
      return {};
    }

    PhaseValues<float> phase_amps;

    std::vector<float> amps;

    if (filter) {
      amps = read_filtered_sensors();
    } else {
      amps = read_sensors();
    }

    if (phase_idx_.a > -1) {
      phase_amps.a = phase_dirs_.c * amps.at(phase_idx_.a);
    }
    if (phase_idx_.b > -1) {
      phase_amps.b = phase_dirs_.b * amps.at(phase_idx_.b);
    }
    if (phase_idx_.c > -1) {
      phase_amps.c = phase_dirs_.c * amps.at(phase_idx_.c);
    }

    if (num_sensors_ == 3) {
      // Assume no one would have more than 3 sensors
      return phase_amps;
    }

    // We now know that we only have 2 sensors
    // We can then figure out the last phase
    if (phase_idx_.a == -1) {
      phase_amps.a = -phase_amps.b - phase_amps.c;
    }
    if (phase_idx_.b == -1) {
      phase_amps.b = -phase_amps.a - phase_amps.c;
    }
    if (phase_idx_.c == -1) {
      phase_amps.c = -phase_amps.a - phase_amps.b;
    }

    return phase_amps;
  }

  void set_filters(Butterworth2 filter)
  {
    for (size_t i = 0; i < num_sensors_; ++i) {
      sensors_.at(i)->set_filter(filter);
    }
  }

private:
  std::vector<InlineCurrentSensor *> sensors_;
  size_t num_sensors_;
  PhaseValues<int> phase_idx_{-1, -1, -1};
  PhaseValues<int> phase_dirs_{0, 0, 0};

  bool aligned_ = false;

  std::vector<float> read_sensors() const
  {
    std::vector<float> reads(num_sensors_, 0);
    for (size_t i = 0; i < num_sensors_; ++i) {
      reads.at(i) = sensors_.at(i)->read();
    }
    return reads;
  }

  std::vector<float> read_filtered_sensors()
  {
    std::vector<float> reads(num_sensors_, 0);
    for (size_t i = 0; i < num_sensors_; ++i) {
      reads.at(i) = sensors_.at(i)->read_filtered();
    }
    return reads;
  }


};

#endif
