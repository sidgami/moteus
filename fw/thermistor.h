// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace moteus {

/// Pre-calculate a thermistor table, then use it online to
/// efficiently determine the temperature given a thermistor ADC
/// reading.
class Thermistor {
 public:
  Thermistor() {}

  void Reset(float resistance_ohm) {
    for (size_t i = 0; i < table_.size(); i++) {
      table_[i] = MakeTable(i * 4096 / table_.size(), resistance_ohm);
    }
  }

  float Calculate(uint16_t adc_raw) MOTEUS_CCM_ATTRIBUTE {
    constexpr int adc_max = 4096;
    const size_t offset = std::max<size_t>(
        1, std::min<size_t>(
            table_.size() - 2,
            adc_raw * table_.size() / adc_max));
    const int16_t this_value = offset * adc_max / table_.size();
    const int16_t next_value = (offset + 1) * adc_max / table_.size();
    const float temp1 = table_[offset];
    const float temp2 = table_[offset + 1];
    return temp1 + (temp2 - temp1) *
        static_cast<float>(adc_raw - this_value) /
        static_cast<float>(next_value - this_value);
  }

 private:
  float MakeTable(size_t count, float thermistor_value) {
    const float v = 3.3f * count / 4096.0f;
    constexpr float B = 4050.0f;
    constexpr float R = 10000.0f;  // The resistor divider pair.

    float r_t = 3.3f * R / v - R;
    return (1.0f / (1.0f / (273.15f + 25.0f) +
                    (1.0f / B) *  std::log(r_t / thermistor_value)) -
            273.15f);
  }

  std::array<float, 32> table_ = {};
};

/// Like Thermistor, but for a PT1000 RTD wired as the low side of a
/// divider: 3.3V -- pullup -- (ADC node) -- PT1000 -- GND.
///
/// An open sensor reads full scale and resolves to the maximum table
/// temperature, so a disconnected probe trips the over-temperature
/// fault rather than reading as cold.
class Pt1000 {
 public:
  Pt1000() {}

  void Reset(float pullup_ohm) {
    for (size_t i = 0; i < table_.size(); i++) {
      table_[i] = MakeTable(i * 4096 / table_.size(), pullup_ohm);
    }
  }

  float Calculate(uint16_t adc_raw) MOTEUS_CCM_ATTRIBUTE {
    constexpr int adc_max = 4096;
    const size_t offset = std::max<size_t>(
        1, std::min<size_t>(
            table_.size() - 2,
            adc_raw * table_.size() / adc_max));
    const int16_t this_value = offset * adc_max / table_.size();
    const int16_t next_value = (offset + 1) * adc_max / table_.size();
    const float temp1 = table_[offset];
    const float temp2 = table_[offset + 1];
    return temp1 + (temp2 - temp1) *
        static_cast<float>(adc_raw - this_value) /
        static_cast<float>(next_value - this_value);
  }

 private:
  float MakeTable(size_t count, float pullup_ohm) {
    const float v = 3.3f * count / 4096.0f;
    const float r_raw = v * pullup_ohm / (3.3f - v);
    // Stay within the domain of the inverse Callendar-Van Dusen
    // relation below (roughly -240C to 850C).
    const float r_t = std::max(100.0f, std::min(3900.0f, r_raw));

    // IEC 60751, T >= 0C branch of R(T) = R0*(1 + A*T + B*T^2),
    // inverted.  Below 0C this is a degree-level approximation, which
    // is fine for a motor sensor.
    constexpr float R0 = 1000.0f;
    constexpr float A = 3.9083e-3f;
    constexpr float B = -5.775e-7f;
    return (-A + std::sqrt(A * A - 4.0f * B * (1.0f - r_t / R0))) /
        (2.0f * B);
  }

  std::array<float, 32> table_ = {};
};
}
