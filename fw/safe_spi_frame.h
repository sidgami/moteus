// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <cstdint>

/// Hardware independent frame construction and parsing for the
/// SafeSPI open standard (https://safespi.org), as implemented by
/// devices like the Renesas RAA2P3200 inductive position sensor.
/// Field layout and CRC details follow the RAA2P3200 datasheet
/// R36DS0061ED0130.
///
/// Frames are 32 bits, transmitted MSB first.
///
/// MOSI (master command) frame:
///   [31:29] Command
///   [28:3]  N/A
///   [2:0]   CRC-3
///
/// MISO (sensor response) frame:
///   [31]    D - 1 if the response contains sensor data
///   [30:28] Command echo
///   [27:21] N/A
///   [20]    S1 status bit
///   [19:4]  Data
///   [3]     S0 status bit
///   [2:0]   CRC-3
///
/// Status bits (S1, S0):
///   00 - valid sensor data
///   01 - sensor is in error state
///   10 - sensor is in programming mode
///   11 - sensor is in initialization state
///
/// The CRC-3 uses polynomial x^3 + x + 1 computed over bits 31:3 with
/// the "augmented" algorithm and a seed of 0b101 (RAA2P3200 datasheet
/// section 6.1.2.9).  With this formulation, the CRC computed over
/// all 32 bits of a valid frame is 0b000.

namespace moteus {

// SafeSPI master commands.
enum class SafeSpiCommand : uint8_t {
  kSensorRead = 0b000,
  kReadRegister = 0b001,
  kWriteRegister = 0b010,
  kBurstWrite = 0b011,
};

// Compute the CRC-3 over all 32 bits of the given frame.  To generate
// a CRC, pass the frame with the CRC field set to 0 and place the
// result in bits 2:0.  To verify a received frame, pass it unmodified
// and check for a result of 0.
inline uint8_t SafeSpiComputeCrc3(uint32_t frame) {
  uint8_t crc = 0b101;
  for (int bit = 31; bit >= 0; bit--) {
    const uint8_t in = (frame >> bit) & 0x01;
    const uint8_t msb = (crc >> 2) & 0x01;
    crc = ((crc << 1) | in) & 0x07;
    if (msb) { crc ^= 0b011; }  // x^3 + x + 1
  }
  return crc;
}

// Construct a complete 32 bit command frame, including CRC, with all
// N/A and payload bits set to 0.
inline uint32_t SafeSpiMakeCommandFrame(SafeSpiCommand command) {
  const uint32_t frame = static_cast<uint32_t>(command) << 29;
  return frame | SafeSpiComputeCrc3(frame);
}

struct SafeSpiParseResult {
  bool crc_ok = false;

  // True if the frame contains sensor data (the D bit).
  bool sensor_data = false;

  uint8_t command = 0;
  uint8_t s1 = 0;
  uint8_t s0 = 0;
  uint16_t data = 0;

  // True if this frame contains a valid position: the CRC matched,
  // the D bit was set, and both status bits report normal operation.
  bool valid_position = false;
};

inline SafeSpiParseResult SafeSpiParseFrame(uint32_t frame) {
  SafeSpiParseResult result;

  result.crc_ok = (SafeSpiComputeCrc3(frame) == 0);
  result.sensor_data = ((frame >> 31) & 0x01) != 0;
  result.command = (frame >> 28) & 0x07;
  result.s1 = (frame >> 20) & 0x01;
  result.s0 = (frame >> 3) & 0x01;
  result.data = (frame >> 4) & 0xffff;

  result.valid_position =
      result.crc_ok &&
      result.sensor_data &&
      result.s1 == 0 &&
      result.s0 == 0;

  return result;
}

}
