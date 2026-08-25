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

#include "fw/safe_spi_boot_entry.h"

#include "mbed.h"

#include "fw/safe_spi_frame.h"

namespace moteus {

SafeSpiBootEntryResult g_safe_spi_boot_entry;

namespace {

// aux2 on moteus-c1 (hw family 2).  This runs before the hardware
// family can be measured, so it is hardcoded for the bench board.
//
// SCK = PF1, MISO = PA10, MOSI = PA11, CS = PB7

uint32_t g_cycles_per_us = 16;

inline uint32_t Cycles() { return DWT->CYCCNT; }

inline void DelayCycles(uint32_t cycles) {
  const uint32_t start = Cycles();
  while ((Cycles() - start) < cycles) {}
}

inline void SckHigh() { GPIOF->BSRR = (1u << 1); }
inline void SckLow() { GPIOF->BSRR = (1u << (1 + 16)); }
inline void MosiWrite(uint32_t v) {
  GPIOA->BSRR = v ? (1u << 11) : (1u << (11 + 16));
}
inline uint32_t MisoRead() { return (GPIOA->IDR >> 10) & 1u; }
inline void CsHigh() { GPIOB->BSRR = (1u << 7); }
inline void CsLow() { GPIOB->BSRR = (1u << (7 + 16)); }

void SetupPins() {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN |
      RCC_AHB2ENR_GPIOFEN;
  (void)RCC->AHB2ENR;

  CsHigh();
  SckLow();
  MosiWrite(0);

  // PF1 output
  GPIOF->MODER = (GPIOF->MODER & ~(3u << (1 * 2))) | (1u << (1 * 2));
  // PA11 output, PA10 input
  GPIOA->MODER = (GPIOA->MODER & ~((3u << (11 * 2)) | (3u << (10 * 2)))) |
      (1u << (11 * 2));
  // PB7 output
  GPIOB->MODER = (GPIOB->MODER & ~(3u << (7 * 2))) | (1u << (7 * 2));
}

// Tri-state SCK/MOSI/CS so an unpowered sensor is not back-powered
// through its input protection diodes between polls.
void IdlePins() {
  GPIOF->MODER &= ~(3u << (1 * 2));
  GPIOA->MODER &= ~(3u << (11 * 2));
  GPIOB->MODER &= ~(3u << (7 * 2));
}

// Re-drive the bus for one poll or an attempt burst.
void DrivePins() {
  CsHigh();
  SckLow();
  MosiWrite(0);
  GPIOF->MODER = (GPIOF->MODER & ~(3u << (1 * 2))) | (1u << (1 * 2));
  GPIOA->MODER = (GPIOA->MODER & ~(3u << (11 * 2))) | (1u << (11 * 2));
  GPIOB->MODER = (GPIOB->MODER & ~(3u << (7 * 2))) | (1u << (7 * 2));
}

void RestorePins() {
  // Back to the reset state (analog) so that the regular aux port
  // bringup starts from a clean slate.
  GPIOF->MODER |= (3u << (1 * 2));
  GPIOA->MODER |= (3u << (11 * 2)) | (3u << (10 * 2));
  GPIOB->MODER |= (3u << (7 * 2));
}

// One 32-bit mode 0 frame at roughly 500kHz.
uint32_t Xfer(uint32_t tx) {
  const uint32_t half = g_cycles_per_us;  // ~1us half period
  uint32_t rx = 0;

  CsLow();
  DelayCycles(half);
  for (int bit = 31; bit >= 0; bit--) {
    MosiWrite((tx >> bit) & 1u);
    DelayCycles(half);
    rx = (rx << 1) | MisoRead();
    SckHigh();
    DelayCycles(half);
    SckLow();
  }
  DelayCycles(half);
  CsHigh();
  MosiWrite(0);
  // >= 700ns of CS high time required between frames.
  DelayCycles(2 * half);

  return rx;
}

void Record(uint32_t frame, uint32_t elapsed_us) {
  auto& r = g_safe_spi_boot_entry;
  if (r.rx_count > 0 && r.rx[r.rx_count - 1] == frame) { return; }
  if (r.rx_count >= SafeSpiBootEntryResult::kNumSamples) { return; }
  r.rx[r.rx_count] = frame;
  r.rx_us[r.rx_count] = elapsed_us;
  r.rx_count++;
}

void RunSafeSpiBootEntry() {
  auto& result = g_safe_spi_boot_entry;
  result.ran = 1;

  // The DWT cycle counter may not be enabled this early.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  g_cycles_per_us = SystemCoreClock / 1000000u;
  if (g_cycles_per_us == 0) { g_cycles_per_us = 16; }

  SetupPins();

  const uint32_t t0 = Cycles();
  const auto elapsed_us = [&]() { return (Cycles() - t0) / g_cycles_per_us; };

  const uint32_t entry_addr_frame =
      SafeSpiMakeRegisterFrame(SafeSpiCommand::kWriteRegister, 0x0606);
  const uint32_t entry_data_frame =
      SafeSpiMakeRegisterFrame(SafeSpiCommand::kWriteRegister, 0x00CA);
  const uint32_t confirm_read_frame =
      SafeSpiMakeRegisterFrame(SafeSpiCommand::kReadRegister, 0x0606);
  const uint32_t sensor_frame =
      SafeSpiMakeCommandFrame(SafeSpiCommand::kSensorRead);

  // Event triggered entry.  The sensor's POR happens at a higher
  // rail voltage than the STM32's, and its tstup_cmd (1.5ms) and
  // tProgEn (5ms) windows are relative to that unknown instant.
  // First watch for the sensor's earliest signs of life (works with a
  // slowly ramped supply, where the sensor wakes long after we boot),
  // then hammer the entry write pair timed from the observed wake-up.

  const auto classify =
      [&](uint32_t frame) {
        // The initialization pattern has been observed on this
        // hardware as all ones below the D bit, not necessarily with
        // a valid CRC.
        if ((frame & ~7u) == 0x0FFFFFF8u) { return 2; }  // init state
        const auto parsed = SafeSpiParseFrame(frame);
        if (!parsed.crc_ok || frame == 0 || frame == 0xffffffffu) {
          return 0;  // no life
        }
        if (parsed.s1 && parsed.s0) { return 2; }  // init state
        return 1;  // alive
      };

  // Rounds of: wait for the sensor's first signs of life (supports
  // hot-connecting the sensor's VDD while we watch), then hammer the
  // entry write pair timed from the observed wake-up.  A bouncing
  // supply contact simply starts another round.
  //
  // The overall deadline is generous to give a human time to touch
  // the sensor's VDD line into place; when the sensor is present and
  // already alive at boot the first poll succeeds immediately, so
  // normal boots are only delayed by the attempt/confirm phase.
  constexpr uint32_t kOverallDeadlineUs = 30000000u;  // 30s

  bool first_poll = true;
  while (elapsed_us() < kOverallDeadlineUs && !result.confirmed) {
    // Phase W: poll briefly at a low duty cycle, bus released in
    // between so an unpowered sensor is not back-powered.
    uint32_t wake_us = 0;
    while (elapsed_us() < kOverallDeadlineUs) {
      DrivePins();
      const uint32_t rx = Xfer(confirm_read_frame);
      const int state = classify(rx);
      if (state != 0) {
        wake_us = elapsed_us();
        result.wake_us = wake_us;
        if (first_poll) { result.alive_at_start = 1; }
        if (state == 2) {
          result.init_seen = 1;
          result.init_last_us = wake_us;
        }
        Record(rx, wake_us);
        break;
      }
      first_poll = false;
      IdlePins();
      DelayCycles(1000 * g_cycles_per_us);
    }
    if (wake_us == 0) { break; }  // overall deadline hit while waiting

    // Phase E: entry attempts from wake to wake+8ms, then confirm
    // polls until wake+80ms (tProgUL margin).  Bus stays driven.
    bool lost_contact = false;
    while (elapsed_us() < wake_us + 80000u && !result.confirmed) {
      const uint32_t now = elapsed_us();

      if (now < wake_us + 8000u) {
        const uint32_t rx1 = Xfer(entry_addr_frame);
        const uint32_t rx2 = Xfer(entry_data_frame);
        Record(rx1, now);
        Record(rx2, now);
        if (classify(rx1) == 2 || classify(rx2) == 2) {
          result.init_seen = 1;
          result.init_last_us = now;
        }
        result.write_attempts++;
      }

      const uint32_t rx3 = Xfer(confirm_read_frame);
      const uint32_t rx4 = Xfer(sensor_frame);
      Record(rx3, now);
      Record(rx4, now);
      result.read_polls++;

      // A dead bus after wake means the supply contact bounced;
      // start a new round and catch the next power-up.
      if (classify(rx3) == 0 && classify(rx4) == 0 &&
          now > wake_us + 2000u) {
        lost_contact = true;
        break;
      }

      // rx4 carries the out-of-frame response to the confirm read.
      const auto parsed = SafeSpiParseFrame(rx4);
      if (parsed.crc_ok && !parsed.sensor_data && parsed.command == 0b001) {
        result.last_readback = parsed.data;
        if (parsed.data == 0x00CA) {
          result.confirmed = 1;
          result.confirm_elapsed_us = elapsed_us();
          break;
        }
      }

      DelayCycles(200 * g_cycles_per_us);
    }

    if (!result.confirmed && !lost_contact) {
      // Sensor is alive and answering but entry did not confirm;
      // nothing more a new round can add unless contact was lost.
      break;
    }
  }

  RestorePins();
}

struct BootEntryRunner {
  BootEntryRunner() { RunSafeSpiBootEntry(); }
};

// Bench kill switch: build with MOTEUS_SAFE_SPI_BOOT_ENTRY_DISABLE
// defined to boot straight into normal operation (no programming
// mode entry attempt, no SRB writes).
#ifndef MOTEUS_SAFE_SPI_BOOT_ENTRY_DISABLE
BootEntryRunner g_boot_entry_runner __attribute__((init_priority(101)));
#endif

}
}
