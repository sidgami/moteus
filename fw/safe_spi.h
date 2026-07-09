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

#include "mbed.h"

#include "hal/spi_api.h"

#include "fw/aux_common.h"
#include "fw/ccm.h"
#include "fw/safe_spi_frame.h"
#include "fw/stm32_spi.h"

namespace moteus {

/// Driver for absolute position sensors using the SafeSPI open
/// standard (https://safespi.org), such as the Renesas RAA2P3200
/// inductive position sensor and encoders based upon it.
///
/// Protocol details (per the RAA2P3200 datasheet R36DS0061ED0130),
/// see safe_spi_frame.h for the frame layout:
/// - SPI Mode 0 (CPOL=0, CPHA=0); the sensor's CPOL/CPHA are NVM
///   programmable and must match
/// - Clock: 10 MHz typical, 12.5 MHz maximum
/// - 32-bit frames, MSB first, position data latched at the falling
///   edge of CS
/// - Out-of-frame protocol: each response is to the command sent in
///   the previous frame.  Since we always send "Sensor Read" (0b000),
///   every frame returns the position latched at its own CS falling
///   edge.
/// - The "Sensor Read" command returns the 14-bit linearized, speed
///   compensated position in the 16-bit DATA field
/// - A minimum of 700ns of CS high time is required between frames;
///   this is guaranteed by the control loop period
class SafeSpi {
 public:
  using Options = Stm32Spi::Options;

  SafeSpi(const Options& options)
      : spi_([&]() {
               auto options_copy = options;
               options_copy.width = 8;
               options_copy.mode = 0;  // CPOL=0, CPHA=0
               return options_copy;
             }()) {
    // Our command never changes: "Sensor Read" with all N/A bits 0.
    const uint32_t tx_frame =
        SafeSpiMakeCommandFrame(SafeSpiCommand::kSensorRead);
    tx_buffer_[0] = (tx_frame >> 24) & 0xff;
    tx_buffer_[1] = (tx_frame >> 16) & 0xff;
    tx_buffer_[2] = (tx_frame >> 8) & 0xff;
    tx_buffer_[3] = (tx_frame >> 0) & 0xff;
  }

  void ISR_StartSample() MOTEUS_CCM_ATTRIBUTE {
    if (active_) { return; }

    spi_.start_dma_transfer(
        std::string_view(reinterpret_cast<const char*>(tx_buffer_), 4),
        mjlib::base::string_span(reinterpret_cast<char*>(rx_buffer_), 4));
    active_ = true;
  }

  void ISR_MaybeFinishSample(aux::Spi::Status* status) MOTEUS_CCM_ATTRIBUTE {
    if (!active_) { return; }
    if (!spi_.is_dma_finished()) { return; }

    spi_.finish_dma_transfer();
    active_ = false;

    const uint32_t frame =
        (static_cast<uint32_t>(rx_buffer_[0]) << 24) |
        (static_cast<uint32_t>(rx_buffer_[1]) << 16) |
        (static_cast<uint32_t>(rx_buffer_[2]) << 8) |
        (static_cast<uint32_t>(rx_buffer_[3]) << 0);

    const auto result = SafeSpiParseFrame(frame);

    if (!result.crc_ok) {
      // Don't change our active state, just note the error and don't
      // increment the nonce.
      status->checksum_errors++;
      return;
    }

    if (!result.sensor_data) {
      // A response to something other than a sensor read command.
      // This can happen for the very first frame after power on.
      // Ignore it.
      return;
    }

    // S1/S0 == 00 indicates valid sensor data.  Anything else is an
    // error, programming, or initialization state.
    status->active = result.valid_position;
    status->ic_pz_bits = (result.s1 << 1) | result.s0;
    status->value = result.data;
    status->nonce++;
  }

 private:
  Stm32Spi spi_;

  bool active_ = false;
  uint8_t tx_buffer_[4] = {};
  uint8_t rx_buffer_[4] = {};
};

}
