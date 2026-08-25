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

#include <optional>

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
    // Our steady state command never changes: "Sensor Read" with all
    // N/A bits 0.
    StoreFrame(SafeSpiMakeCommandFrame(SafeSpiCommand::kSensorRead),
               sensor_tx_buffer_);
  }

  /// Result of the most recently completed register operation.
  struct RegisterResult {
    // If false, no echo arrived within the timeout window and the
    // remaining fields are not meaningful.
    bool ok = false;
    // The command bits echoed by the sensor.
    uint8_t command = 0;
    // The register contents for a read; for a write, whatever word
    // the sensor echoed.  A write should be verified with a
    // subsequent read.
    uint16_t data = 0;
    // For a write, the echo of the address frame (the first of the
    // two command frames); not meaningful for a read.
    uint8_t first_command = 0;
    uint16_t first_data = 0;
    // For a write, the response to the leading read of the target
    // register: its previous contents.
    uint8_t pre_command = 0;
    uint16_t pre_data = 0;
  };

  /// Queue a single register operation to be interleaved with the
  /// streaming sensor reads.  Returns false if another operation is
  /// still in flight.  The result is available via register_result()
  /// a few control cycles later.
  ///
  /// Note that writes to NVM registers additionally require the
  /// caller to respect the sensor's minimum programming time (6ms for
  /// the RAA2P3200) before issuing another command.
  bool StartRegisterRead(uint16_t address) {
    return StartOperation(SafeSpiCommand::kReadRegister, address, 0);
  }

  bool StartRegisterWrite(uint16_t address, uint16_t data) {
    return StartOperation(SafeSpiCommand::kWriteRegister, address, data);
  }

  /// The RAA2P3200 accepts the programming mode entry command (write
  /// 0xCA to register 0x0606) only within a limited window after its
  /// power on reset.  When armed (the default, so that a rig wide
  /// power cycle enters programming mode with no host involvement),
  /// the entry write is issued automatically on the first CRC-valid
  /// frame received from the sensor.
  bool StartProgrammingModeEntry() {
    return StartRegisterWrite(0x0606, 0x00CA);
  }

  /// Issue a write using the burst write command encoding instead of
  /// the plain register write, for protocol experimentation.
  bool StartBurstWrite(uint16_t address, uint16_t data) {
    return StartOperation(SafeSpiCommand::kBurstWrite, address, data);
  }

  /// Transmit one arbitrary frame (CRC is recomputed over the given
  /// bits) and capture the surrounding responses in the trace, for
  /// probing undocumented commands.
  bool StartRawFrame(uint32_t frame) {
    if (raw_pending_) { return false; }
    raw_frame_ = frame & ~7u;
    raw_frame_ |= SafeSpiComputeCrc3(raw_frame_);
    trace_pos_ = 0;
    trace_armed_ = true;
    asm volatile("" ::: "memory");
    raw_pending_ = true;
    return true;
  }

  /// Return the result of the last operation, or nullopt if one is
  /// still in flight (or none was ever started).
  std::optional<RegisterResult> register_result() const {
    if (reg_state_ != RegState::kComplete) { return {}; }
    return result_;
  }

  /// Raw MISO frames captured starting with the transfer that carried
  /// the most recent operation's first command frame.
  static constexpr size_t kTraceSize = 8;
  const uint32_t* trace() const { return trace_; }

  void ISR_StartSample() MOTEUS_CCM_ATTRIBUTE {
    if (active_) { return; }

    const uint8_t* tx = sensor_tx_buffer_;
    if (raw_pending_) {
      StoreFrame(raw_frame_, reg_tx_buffer_);
      raw_pending_ = false;
      spi_.start_dma_transfer(
          std::string_view(reinterpret_cast<const char*>(reg_tx_buffer_), 4),
          mjlib::base::string_span(reinterpret_cast<char*>(rx_buffer_), 4));
      active_ = true;
      return;
    }
    switch (reg_state_) {
      case RegState::kQueued: {
        if (op_command_ == SafeSpiCommand::kWriteRegister ||
            op_command_ == SafeSpiCommand::kBurstWrite) {
          // The RAA2P3200 only accepts a register write whose target
          // was read immediately beforehand (the read defines the
          // write address); lead every write with a read of the
          // target, with no other frames interleaved.
          StoreFrame(SafeSpiMakeRegisterFrame(
                         SafeSpiCommand::kReadRegister, op_address_),
                     reg_tx_buffer_);
          reg_state_ = RegState::kWriteAddr;
        } else {
          StoreFrame(SafeSpiMakeRegisterFrame(op_command_, op_address_),
                     reg_tx_buffer_);
          reg_state_ = RegState::kAwaitEcho;
        }
        tx = reg_tx_buffer_;
        break;
      }
      case RegState::kWriteAddr: {
        StoreFrame(SafeSpiMakeRegisterFrame(op_command_, op_address_),
                   reg_tx_buffer_);
        tx = reg_tx_buffer_;
        reg_state_ = RegState::kWriteData;
        break;
      }
      case RegState::kWriteData: {
        StoreFrame(SafeSpiMakeRegisterFrame(op_command_, op_data_),
                   reg_tx_buffer_);
        tx = reg_tx_buffer_;
        if (op_address_ < 0x0200) {
          // NVM write: the sensor needs at least 6ms with NO further
          // commands (not even sensor reads) to program the cell;
          // any traffic aborts the burn with d_internal_logic_fail.
          // Stay silent for a comfortable margin, then complete.
          quiet_cycles_ = kNvmQuietCycles;
          reg_state_ = RegState::kNvmQuiet;
        } else {
          reg_state_ = RegState::kAwaitEcho;
        }
        break;
      }
      case RegState::kNvmQuiet: {
        // Bus silence while the sensor programs its NVM.  The
        // transfer that carried the data frame has already completed
        // (we are only entered after ISR_MaybeFinishSample).
        if (quiet_cycles_ > 0) {
          quiet_cycles_--;
          return;  // no transfer at all this cycle
        }
        result_.ok = true;
        reg_state_ = RegState::kComplete;
        break;
      }
      default: {
        break;
      }
    }

    spi_.start_dma_transfer(
        std::string_view(reinterpret_cast<const char*>(tx), 4),
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

    if (trace_armed_) {
      trace_[trace_pos_++] = frame;
      if (trace_pos_ >= kTraceSize) { trace_armed_ = false; }
    }

    // If a register operation is in flight, give up on it after
    // enough cycles that its echo cannot still be pending.
    if (reg_state_ == RegState::kWriteAddr ||
        reg_state_ == RegState::kWriteData ||
        reg_state_ == RegState::kAwaitEcho) {
      op_cycles_++;
      if (op_cycles_ > kRegisterOpTimeoutCycles) {
        result_ = {};
        reg_state_ = RegState::kComplete;
      }
    }

    if (!result.crc_ok) {
      // Don't change our active state, just note the error and don't
      // increment the nonce.
      status->checksum_errors++;
      return;
    }

    if (!result.sensor_data) {
      // A response to something other than a sensor read command.
      // With no operation in flight this can happen for the very
      // first frame after power on; otherwise it is the out-of-frame
      // echo of one of our register command frames.  A write emits
      // two command frames and thus two echoes; only the last one
      // completes the operation.
      if (reg_state_ == RegState::kAwaitEcho ||
          reg_state_ == RegState::kWriteAddr ||
          reg_state_ == RegState::kWriteData) {
        if (pending_echoes_ == 3) {
          result_.pre_command = result.command;
          result_.pre_data = result.data;
        }
        if (pending_echoes_ == 2) {
          result_.first_command = result.command;
          result_.first_data = result.data;
        }
        if (--pending_echoes_ == 0) {
          result_.ok = true;
          result_.command = result.command;
          result_.data = result.data;
          reg_state_ = RegState::kComplete;
        }
      }
      return;
    }

    // S1/S0 == 00 indicates valid sensor data.  Anything else is an
    // error, programming, or initialization state.
    status->active = result.valid_position;
    status->ic_pz_bits = (result.s1 << 1) | result.s0;
    status->value = result.data;
    status->nonce++;

    // Fire the automatic programming mode entry on the very first
    // CRC-valid frame from the sensor, even while it still reports
    // the initialization state: the sensor holds in init until SPI
    // traffic arrives, and its post-POR programming window may
    // already be ticking.
    if (auto_entry_armed_ && reg_state_ == RegState::kIdle) {
      auto_entry_armed_ = false;
      StartOperation(SafeSpiCommand::kWriteRegister, 0x0606, 0x00CA);
    }
  }

 private:
  enum class RegState : uint8_t {
    kIdle,       // no operation pending
    kQueued,     // command frame goes out with the next transfer
    kWriteAddr,  // leading read of the target sent; address frame next
    kWriteData,  // write address sent; data frame goes out next
    kAwaitEcho,  // command frames sent; awaiting the final echo
    kNvmQuiet,   // NVM data frame sent; bus held silent while burning
    kComplete,   // result_ is valid
  };

  // Bus-silent cycles after an NVM write; must exceed the sensor's
  // 6ms programming time at any plausible control rate.
  static constexpr uint32_t kNvmQuietCycles = 300;

  // An operation's echo arrives one frame after its last command
  // frame, so a handful of cycles is already generous.
  static constexpr uint32_t kRegisterOpTimeoutCycles = 16;

  static void StoreFrame(uint32_t frame, uint8_t* buffer) {
    buffer[0] = (frame >> 24) & 0xff;
    buffer[1] = (frame >> 16) & 0xff;
    buffer[2] = (frame >> 8) & 0xff;
    buffer[3] = (frame >> 0) & 0xff;
  }

  bool StartOperation(SafeSpiCommand command, uint16_t address,
                      uint16_t data) {
    const auto state = reg_state_;
    if (state == RegState::kQueued ||
        state == RegState::kWriteAddr ||
        state == RegState::kWriteData ||
        state == RegState::kAwaitEcho ||
        state == RegState::kNvmQuiet) {
      return false;
    }

    op_command_ = command;
    op_address_ = address;
    op_data_ = data;
    op_cycles_ = 0;
    trace_pos_ = 0;
    trace_armed_ = true;
    pending_echoes_ =
        (command == SafeSpiCommand::kWriteRegister ||
         command == SafeSpiCommand::kBurstWrite) ? 3 : 1;
    result_ = {};

    // Publish the operation to the ISR only after all of its
    // parameters are in place.
    asm volatile("" ::: "memory");
    reg_state_ = RegState::kQueued;
    return true;
  }

  Stm32Spi spi_;

  bool active_ = false;
  uint8_t sensor_tx_buffer_[4] = {};
  uint8_t reg_tx_buffer_[4] = {};
  uint8_t rx_buffer_[4] = {};

  volatile RegState reg_state_ = RegState::kIdle;
  uint32_t quiet_cycles_ = 0;
  // Automatic post-POR programming mode entry.  Disarmed: this
  // silicon NACKs all SafeSPI write traffic in normal operation (see
  // the project notes), and write traffic during sensor init risks
  // holding init.  Kept for potential future hardware that power
  // cycles the sensor on command.
  bool auto_entry_armed_ = false;
  SafeSpiCommand op_command_ = SafeSpiCommand::kSensorRead;
  uint16_t op_address_ = 0;
  uint16_t op_data_ = 0;
  uint32_t op_cycles_ = 0;
  uint8_t pending_echoes_ = 0;
  RegisterResult result_;

  volatile bool trace_armed_ = false;
  volatile uint8_t trace_pos_ = 0;
  uint32_t trace_[kTraceSize] = {};

  volatile bool raw_pending_ = false;
  uint32_t raw_frame_ = 0;
};

}
