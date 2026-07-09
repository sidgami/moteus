# SafeSPI Encoders #

This guide covers bringing up and testing an absolute encoder that
uses the [SafeSPI](https://safespi.org) interface standard on a
moteus controller.  This includes custom inductive encoders based on
the Renesas RAA2P3200 position sensor IC, whose datasheet
(R36DS0061ED0130) is the protocol reference used by this
implementation.

## What is included ##

* `fw/safe_spi_frame.h` - hardware independent SafeSPI frame
  construction, parsing, and CRC-3 handling
* `fw/safe_spi.h` - the ISR-rate driver, which continuously issues
  the SafeSPI "Sensor Read" command using a 4 byte DMA transfer per
  control cycle
* `fw/test/safe_spi_frame_test.cc` - host unit tests for the frame
  logic
* Integration as SPI mode 10 (`safe_spi`) in the aux port framework

The driver reports the full 16-bit DATA field from each valid SafeSPI
response into `aux[12].spi.value`.  The two SafeSPI status bits are
mirrored into `aux[12].spi.ic_pz_bits` (bit 1 = S1, bit 0 = S0) and
CRC failures increment `aux[12].spi.checksum_errors`.

## Prerequisites: sensor-side configuration ##

RAA2P3200-based sensors store their interface selection in NVM and
must be programmed *before* moteus can talk to them.  This is done
over the Renesas UART programming interface (OUT1/OUT2 pins) during a
short window after power-on, using the Renesas RAA2P-COMBOARD or an
equivalent fixture.  Configure:

1. **Interface option: SPI**.  If the part is configured for
   UART/ABI/UVW, the SPI pins are repurposed and moteus will see no
   response.
2. **Supply mode: 3.3V** (not 5V).  The moteus aux port logic is
   3.3V.
3. **SPI mode: CPOL=0, CPHA=0.**  The moteus driver uses SPI mode 0.
4. Coil gain / offset / linearization per your coil design, using the
   vendor's end-of-line calibration flow.  Linearization runs on-chip
   before the SPI output, so do it there rather than expecting
   moteus-side compensation.

Note that a "locked" sensor cannot be unlocked, so do not enable the
lock bits until the configuration is final.

## Wiring ##

Connect to a moteus aux port (typically aux2, since aux1 usually
hosts the onboard encoder).  Consult
[pinouts](../reference/pinouts.md) for which pins on your board
support SPI.  For an RAA2P3200-based sensor:

| Sensor pin    | Function       | moteus aux pin mode |
|---------------|----------------|---------------------|
| 2 (CS)        | Chip select    | `spi_cs` (mode 2)   |
| 10 (IO1)      | SCK            | `spi` (mode 1)      |
| 3 (MOSI)      | MOSI           | `spi` (mode 1)      |
| 12 (OUT2)     | MISO           | `spi` (mode 1)      |
| 14 (VDD)      | 3.3V           | 3.3V supply         |
| 15 (GND)      | GND            | GND                 |

Keep the SPI harness short or use series termination for 10 MHz
operation; the RAA2P3200's outputs are specified with 60 pF load for
local (on-board) applications.

## Running the host unit tests ##

The frame logic (CRC-3 generation/verification, frame parsing, status
bit handling) is covered by host unit tests.  Run:

```
tools/bazel test --config=host //:host
```

or just the firmware test target:

```
tools/bazel test --config=host //fw:test
```

All `SafeSpi*` test cases must pass.  These verify:

* generate-then-verify CRC round trips produce a zero remainder
* every single-bit corruption of a frame is detected
* the constant "Sensor Read" TX frame is `0x00000003`
* status bit combinations (error / programming / initialization) are
  rejected as invalid positions

## Building and flashing the firmware ##

```
tools/bazel build --config=target //:target
fw/flash.py
```

## moteus configuration ##

Using `utils/tview.py` or `moteus_tool`, with the sensor on aux2 and
(for example) pins 0-3 as CS/SCK/MOSI/MISO:

```
conf set aux2.pins.0.mode 2    # spi_cs
conf set aux2.pins.1.mode 1    # spi (SCK)
conf set aux2.pins.2.mode 1    # spi (MOSI)
conf set aux2.pins.3.mode 1    # spi (MISO)
conf set aux2.spi.mode 10      # safe_spi
conf set aux2.spi.rate_hz 10000000

conf set motor_position.sources.1.aux_number 2
conf set motor_position.sources.1.type 1       # spi
conf set motor_position.sources.1.cpr 16384    # see "Data alignment" below
conf write
d flush
```

The exact pin assignment depends upon which aux port pins on your
board have SPI capability; a `spi_pin_error` in `aux2.status.error`
means the selected pins cannot form a complete SPI bus with CS.

## Bring-up test sequence ##

Perform these steps in order.  Each one isolates a different part of
the stack.

### 1. Verify frames are being exchanged ###

In tview, watch:

* `aux2.spi.nonce` - should be continuously incrementing (one count
  per control cycle, wrapping at 256).  If it never moves, no valid
  frames are arriving: check wiring, sensor power, and that the
  sensor NVM is configured for SPI.
* `aux2.spi.checksum_errors` - should stay at 0.  See
  troubleshooting below if it climbs.
* `aux2.spi.active` - should be true once the sensor has completed
  its power-on initialization (3-5 ms for the RAA2P3200).

### 2. Verify status bits ###

`aux2.spi.ic_pz_bits` holds the SafeSPI status bits (S1 in bit 1, S0
in bit 0):

* 0 - valid sensor data (expected)
* 1 - sensor error state: check coil connections and the sensor's
  diagnostic registers
* 2 - programming mode
* 3 - initialization (normal transiently at power-on)

### 3. Verify position data and alignment ###

Rotate the target slowly by hand and watch `aux2.spi.value`:

* The value should change smoothly and monotonically with rotation,
  wrapping once per *electrical* period of your coil design.
* **Data alignment check**: the RAA2P3200 datasheet specifies a
  14-bit position in a 16-bit data field but not its alignment.
  Observe the value as you rotate:
  * If it spans 0-16383 and changes in steps of 1, the position is
    right-aligned: use `cpr 16384`.
  * If it spans 0-65535 and only changes in steps of 4 (two zero
    LSBs), it is left-aligned: use `cpr 65536`.
* For a multi-periodic coil with N periods per mechanical turn, the
  value wraps N times per turn.  The sensor is absolute only within
  one electrical period.

### 4. Verify update integrity under motion ###

Spin the target continuously (by hand or with a drill on the shaft)
and confirm:

* `checksum_errors` remains static
* `motor_position.sources.1.filtered_value` in tview tracks without
  jumps or direction reversals

The `utils/encoder_bandwidth.py` and `utils/compare_encoders.py`
scripts can quantify noise and latency against a reference encoder if
one is fitted on the same shaft.

### 5. Calibrate and close the loop ###

If the SafeSPI encoder is the commutation sensor:

```
python3 -m moteus.moteus_tool --target 1 --calibrate
```

Then verify basic closed loop operation at low torque:

```
d pos nan 0.5 1 p0.5 d0.01 f0.05
```

and confirm smooth motion in both directions.  Standard calibration
troubleshooting applies, see [calibration](calibration.md).

## Troubleshooting ##

| Symptom | Likely cause |
|---|---|
| `nonce` never increments, `checksum_errors` 0 | MISO stuck low/high: sensor unpowered, NVM not set to SPI mode, or wiring fault.  An all-zero or all-one frame fails CRC *and* the D-bit check silently; scope MISO during the 32-clock burst. |
| `checksum_errors` increments continuously | 1) SPI mode mismatch: confirm the sensor NVM CPOL/CPHA is 0/0.  2) CRC seed convention mismatch, see below.  3) Signal integrity: lower `aux2.spi.rate_hz` (e.g. 2500000) and retest. |
| `checksum_errors` increments occasionally | Signal integrity or grounding: shorten harness, lower clock rate, add series resistors. |
| `active` false, `ic_pz_bits` == 1 | Sensor error state: coil open/short, target out of range, supply out of range.  Read the sensor IRQ/diagnostic registers with a vendor tool. |
| Value moves but motor calibration fails | Wrong `cpr` (alignment, see step 3) or coil period count not matching expectations. |

### CRC seed verification ###

The driver implements the CRC-3 as stated in the RAA2P3200 datasheet
(polynomial x^3 + x + 1, augmented algorithm, seed 0b101) and its
self-consistency is unit tested.  However, the match against real
silicon has not yet been validated on hardware.  If every frame fails
CRC at low clock rates with confirmed-good signals, capture one MISO
frame with a logic analyzer and check it against
`SafeSpiComputeCrc3()` in `fw/safe_spi_frame.h`; if the silicon uses
a different seed convention, only the single `crc = 0b101;`
initialization line needs to change (try `0b100` or `0b000`).

## Design notes and limitations ##

* The driver issues one 32-bit frame per control loop cycle (about
  3.3 us on the wire at 10 MHz).  Because SafeSPI sensors latch
  position at the falling edge of CS and we always send the same
  Sensor Read command, each frame delivers the position sampled at
  its own start - there is no extra pipeline latency from the
  SafeSPI out-of-frame protocol.
* The 700 ns minimum CS-high time between frames is guaranteed by the
  control loop period.
* Register read/write access (for example reading the on-chip
  temperature or turns counter) is not implemented; the sensor should
  be fully configured through the vendor programming flow before use.
* The sensor's own diagnostics (S1/S0) gate `aux2.spi.active`, so a
  sensor-reported fault will cause the position source to drop out
  rather than feed bad data to the control loop.
