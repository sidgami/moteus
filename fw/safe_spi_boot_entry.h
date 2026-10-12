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

/// BENCH EXPERIMENT: RAA2P3200 programming mode entry from the POR
/// window.
///
/// The RAA2P3200 datasheet (R36DS0061ED0130 Rev 1.3, tProgEn) requires
/// the first programming enable command to be sent within 5ms of the
/// sensor's power on reset, no earlier than tstup_cmd (1.5ms).  In
/// normal operation this silicon rejects every SafeSPI write command
/// with a content-id 0b110 response, so the only way to enter
/// programming mode without external power control is to send the
/// entry write (0x00CA to register 0x0606) from very early boot, while
/// the shared 3.3V rail has just brought both the STM32 and the sensor
/// out of POR.
///
/// A high priority static constructor bit-bangs the entry sequence on
/// the aux2 pins (moteus-c1 / family 2: SCK=PF1, MISO=PA10, MOSI=PA11,
/// CS=PB7) before mbed's own initialization runs, then records what
/// happened for later inspection over the console ("aux2 sspi-boot").

namespace moteus {

struct SafeSpiBootEntryResult {
  uint8_t ran = 0;         // the hook executed
  uint8_t confirmed = 0;   // register 0x0606 read back 0x00CA
  uint8_t alive_at_start = 0;  // first poll already got a CRC-valid frame
  uint8_t init_seen = 0;       // observed S1/S0 == 11 (sensor init state)
  uint16_t last_readback = 0xffff;   // last observed 0x0606 payload
  uint32_t confirm_elapsed_us = 0;   // time of confirmation
  uint32_t wake_us = 0;              // first CRC-valid frame observed
  uint32_t init_last_us = 0;         // last time an init-state frame was seen
  uint32_t write_attempts = 0;       // entry write pairs sent
  uint32_t read_polls = 0;           // confirm read pairs sent

  // After a confirmed entry: result of a tight-gap SRB write test
  // (0x0123 to zero_pos_angle1 0x0220, then read back and restore).
  uint16_t test_write_readback = 0xffff;

  // Deduplicated capture of MISO frames with timestamps, recording
  // each change in what the sensor sends back (init pattern, first
  // response to a write, etc).
  static constexpr unsigned kNumSamples = 16;
  uint32_t rx[kNumSamples] = {};
  uint32_t rx_us[kNumSamples] = {};
  uint8_t rx_count = 0;
};

extern SafeSpiBootEntryResult g_safe_spi_boot_entry;

}
