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

#include "fw/safe_spi_frame.h"

#include <boost/test/auto_unit_test.hpp>

#include <cstdlib>
#include <initializer_list>

using namespace moteus;

namespace {
// Construct a MISO frame from fields, with a correct CRC.
uint32_t MakeMisoFrame(bool d, uint8_t command, uint8_t s1,
                       uint16_t data, uint8_t s0) {
  const uint32_t frame =
      (static_cast<uint32_t>(d ? 1 : 0) << 31) |
      (static_cast<uint32_t>(command & 0x07) << 28) |
      (static_cast<uint32_t>(s1 & 0x01) << 20) |
      (static_cast<uint32_t>(data) << 4) |
      (static_cast<uint32_t>(s0 & 0x01) << 3);
  return frame | SafeSpiComputeCrc3(frame);
}
}

BOOST_AUTO_TEST_CASE(SafeSpiCrcGenerateVerify) {
  // For any message, generating a CRC and then verifying the complete
  // frame must yield 0.
  std::srand(42);
  for (int i = 0; i < 10000; i++) {
    const uint32_t message =
        ((static_cast<uint32_t>(std::rand()) << 17) ^
         static_cast<uint32_t>(std::rand())) & ~7u;
    const uint8_t crc = SafeSpiComputeCrc3(message);
    BOOST_TEST(crc < 8);
    BOOST_TEST(SafeSpiComputeCrc3(message | crc) == 0);
  }
}

BOOST_AUTO_TEST_CASE(SafeSpiCrcDetectsSingleBitErrors) {
  // A CRC-3 with polynomial x^3 + x + 1 detects all single bit
  // errors.
  std::srand(43);
  for (int i = 0; i < 1000; i++) {
    const uint32_t message =
        ((static_cast<uint32_t>(std::rand()) << 17) ^
         static_cast<uint32_t>(std::rand())) & ~7u;
    const uint32_t frame = message | SafeSpiComputeCrc3(message);
    for (int bit = 0; bit < 32; bit++) {
      BOOST_TEST(SafeSpiComputeCrc3(frame ^ (1u << bit)) != 0);
    }
  }
}

BOOST_AUTO_TEST_CASE(SafeSpiCommandFrame) {
  // The sensor read command frame is all zeros aside from the CRC.
  const uint32_t frame =
      SafeSpiMakeCommandFrame(SafeSpiCommand::kSensorRead);
  BOOST_TEST((frame & ~7u) == 0u);
  BOOST_TEST(SafeSpiComputeCrc3(frame) == 0);

  // A register read command occupies the top 3 bits.
  const uint32_t read_frame =
      SafeSpiMakeCommandFrame(SafeSpiCommand::kReadRegister);
  BOOST_TEST((read_frame >> 29) == 0b001u);
  BOOST_TEST(SafeSpiComputeCrc3(read_frame) == 0);
}

BOOST_AUTO_TEST_CASE(SafeSpiParseValidPosition) {
  const uint32_t frame = MakeMisoFrame(true, 0b000, 0, 0x1234, 0);
  const auto result = SafeSpiParseFrame(frame);

  BOOST_TEST(result.crc_ok == true);
  BOOST_TEST(result.sensor_data == true);
  BOOST_TEST(result.s1 == 0);
  BOOST_TEST(result.s0 == 0);
  BOOST_TEST(result.data == 0x1234);
  BOOST_TEST(result.valid_position == true);
}

BOOST_AUTO_TEST_CASE(SafeSpiParseStatusBits) {
  {
    // Sensor in error state: S1=0, S0=1.
    const auto result =
        SafeSpiParseFrame(MakeMisoFrame(true, 0b000, 0, 0x0100, 1));
    BOOST_TEST(result.crc_ok == true);
    BOOST_TEST(result.s0 == 1);
    BOOST_TEST(result.valid_position == false);
  }
  {
    // Sensor in programming mode: S1=1, S0=0.
    const auto result =
        SafeSpiParseFrame(MakeMisoFrame(true, 0b000, 1, 0x0100, 0));
    BOOST_TEST(result.crc_ok == true);
    BOOST_TEST(result.s1 == 1);
    BOOST_TEST(result.valid_position == false);
  }
  {
    // Sensor in initialization state: S1=1, S0=1.
    const auto result =
        SafeSpiParseFrame(MakeMisoFrame(true, 0b000, 1, 0x0100, 1));
    BOOST_TEST(result.crc_ok == true);
    BOOST_TEST(result.valid_position == false);
  }
}

BOOST_AUTO_TEST_CASE(SafeSpiParseNonSensorData) {
  // D=0 indicates a register response, not sensor data.
  const auto result =
      SafeSpiParseFrame(MakeMisoFrame(false, 0b001, 0, 0xabcd, 0));
  BOOST_TEST(result.crc_ok == true);
  BOOST_TEST(result.sensor_data == false);
  BOOST_TEST(result.data == 0xabcd);
  BOOST_TEST(result.valid_position == false);
}

BOOST_AUTO_TEST_CASE(SafeSpiParseCorruptFrame) {
  const uint32_t frame = MakeMisoFrame(true, 0b000, 0, 0x2222, 0);
  const auto result = SafeSpiParseFrame(frame ^ (1u << 10));
  BOOST_TEST(result.crc_ok == false);
  BOOST_TEST(result.valid_position == false);
}

BOOST_AUTO_TEST_CASE(SafeSpiParseDataRange) {
  // The 16-bit data field passes through unmodified at its extremes.
  for (const uint16_t data : {uint16_t(0x0000), uint16_t(0x3fff),
                              uint16_t(0xffff)}) {
    const auto result =
        SafeSpiParseFrame(MakeMisoFrame(true, 0b000, 0, data, 0));
    BOOST_TEST(result.crc_ok == true);
    BOOST_TEST(result.data == data);
  }
}
