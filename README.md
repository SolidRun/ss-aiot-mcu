# STM32U031C8 I2C Sensor Slave Project

## Overview

This project implements an I2C slave interface on the STM32U031C8 MCU to control and read multiple sensors. The MCU acts as a slave device, receiving commands from a master over I2C and performing actions such as turning sensors on/off, reading sensor data, and configuring thresholds.

## Supported sensors and modules:

-LED

-Infrared (IR) sensor

-Accelerometer

-GPS

-Battery charger status

-Real-time clock (RTC)

-Interrupts (MCU, IR, ACC)

## Table of Contents:

1. I2C Interface

2. Command Definitions

    2.1 Command & Response Format

    2.2 Examples

    2.3 Response Lengths

    2.4 GPS NMEA Passthrough

    2.5 Timekeeping

3. Sensor Thresholds & Configuration

    3.1 Accelerometer — events, sample capture, temperature

    3.2 Infrared Sensor (IR)

    3.3 Battery Charging

4. Known Limitations




### 1. I2C Interface:

The MCU is a **slave** on I2C2. All commands in this document are issued by the
master (the SOM).

| Property | Value |
|----------|-------|
| Slave address (7-bit) | **0x18** |
| MCU peripheral | I2C2 (`PA9` = SCL, `PA10` = SDA) |
| Bus clock | Driven by the master. The slave's `TIMINGR` is configured for standard-mode timing (~100 kHz). |
| Clock stretching | Enabled (`NoStretchMode = DISABLE`) |
| Addressing | 7-bit |

The address is set by `hi2c2.Init.OwnAddress1 = 48` in `Core/Src/i2c.c`.
`OwnAddress1` is the 8-bit form, so the 7-bit address on the wire is `48 >> 1 = 0x18`.
In STM32CubeMX the field `I2C2.OwnAddress` holds the **7-bit** value (24 = 0x18) and
CubeMX doubles it when generating the code — do not "fix" one to match the other.

From Linux on the SOM, the MCU should appear at `0x18`:

```
i2cdetect -y -r <bus>
```

#### Interrupt line

Besides I2C the MCU drives one interrupt line towards the SOM, `MCU_INT` (`PA15`).

| Property | Value |
|----------|-------|
| Drive | **Open drain**, `GPIO_MODE_OUTPUT_OD` |
| Polarity | **Active low** — pulled low to signal, released to deassert |
| Idle level | Held high by a **pull-up on the SOM**, which the master must enable |

The master must therefore configure the line as level triggered, active low. Under
Linux that is `IRQ_TYPE_LEVEL_LOW` in the device tree.

The line is asserted whenever a source records an interrupt, and released by the
`Read interrupt status` command, which snapshots and clears every source in one
operation. It stays asserted until that command is issued, so a master that never
reads the interrupt status will see the line held low indefinitely.

#### What the interrupt status register contains

`Read interrupt status` (`0x12 0x07`) returns five bytes: a source bitfield,
followed by one detail byte per source. `DATA_LEN` is `5`, so read 7.

| Byte | Contents |
|------|----------|
| `data[0]` | **Sources.** `0x01` the MCU itself, `0x02` IR, `0x04` accelerometer, `0x08` RTC. |
| `data[1]` | **MCU detail** — `0x01` the firmware started or restarted. |
| `data[2]` | **IR detail** — `0x01` motion, `0x02` presence. Thermal shock is routed off the pin and not reported. |
| `data[3]` | **Accelerometer detail** — `0x01` motion, `0x02` tilt, `0x04` free-fall. |
| `data[4]` | **RTC detail** — `0x01` alarm A fired. Alarm B, the wakeup timer and tamper are not used and have no bit yet. |

The MCU raises `0x01` in both `data[0]` and `data[1]` once during init, so the
first read after a reset reports the source and the restart as its reason.

`data[0]` is a bitfield, so `0x01` means the MCU itself rather than a generic
"an interrupt happened".

#### Every byte answers "what fired", never "what is true now"

All five bytes are accumulated latches, and the read clears them. Two sources
firing between reads produce both bits rather than the last one: presence does not
mask motion, and an accelerometer wake-up does not overwrite an earlier one.

The reason the firmware cannot report present state is physical. Both sensors
drive their INT pin as a **level**, not a pulse — `ALGO_CONFIG.INT_PULSED` is `0`
on the STHS34PF80 — and the firmware watches only the rising edge. There is no
falling-edge handler anywhere in the project. So a condition that stays asserted
produces exactly one event, and once the master has consumed it there is no second
edge and no way to ask whether it is still true. For present state, read the
sensor's own value command instead: `0x12 0x02` returns live presence and motion
counts.

The snapshot is taken with interrupts masked, so `data[0]` is exactly the OR of
the sources at a single instant, and an edge that arrives during the read is
reported on the next one rather than lost.

#### Why open drain, and why active low

**Open drain, because the SOM's I/O voltage is not ours to choose.** The line
crosses into the SOM's domain, which is 1.8 V on some modules and 3.3 V on others.
Open drain means the MCU only ever pulls the line down and never sources into it,
so the high level is set by the SOM's own pull-up at the SOM's own rail. Driving
the line push-pull from the MCU's supply would put that supply on the CPU input
and can damage a 1.8 V one.

**Active low, because the MCU's pin starts high impedance.** `PA15` has no
alternate function on this part — it is not a debug pin, so it comes out of reset
as a floating input and stays that way until `MX_GPIO_Init()` runs. Against the
SOM's pull-up that window reads as deasserted. Were the line active high, the same
window would look like an interrupt, and the SOM would see a spurious one on every
MCU reset.

#### Transaction sequence

A command is one **write** transaction followed by a separate **read** transaction:

1. Master writes the 3-byte header, then `DATA_LEN` payload bytes (if any).
2. The MCU decodes and executes the command.
3. Master reads the response.

The master must allow time between the write and the read: the command is executed
when the write completes, and some commands take a while (see
[Response Lengths](#23-response-lengths) and [Known Limitations](#4-known-limitations)).

#### How many bytes to read — read this before writing master code

The slave arms **every** response at the full size of its transmit buffer — 257
bytes, `STATUS`, `DATA_LEN` and 255 of payload — regardless of which command
produced it. Any read length from 1 to 257 bytes therefore completes normally,
and a master is free to use one fixed-size read for everything and let
`DATA_LEN` say how much of the reply is real. The longest reply any command
actually produces is 86 bytes.

**Bytes past `2 + DATA_LEN` are meaningless and must be discarded.** They are
not padding and not zeros: the buffer is never cleared between commands, so the
tail holds whatever the previous response left there — NMEA text after a GPS
read, consumed accelerometer samples after a motion read. Only before the first
command following a reset is it all zeros, because the buffer lives in `.bss`.

**Reading more than 257 bytes hangs the bus.** Past that the slave runs out of
armed data mid-transfer and keeps stretching SCL; the master sees
`SCL is stuck low` / `Connection timed out`, and it stays that way until the
stuck-bus watchdog fires about ten seconds later (see
[Known Limitations](#4-known-limitations)). 257 is the only limit that matters —
there is no per-command one.

Reading *fewer* bytes than `2 + DATA_LEN` is legitimate rather than merely
tolerated, and a **short read followed by a longer one works for every command**.
A master may read 2 bytes to learn `STATUS` and `DATA_LEN`, then issue a second
read transaction of `2 + DATA_LEN` for the whole reply.

What makes that safe is that only a master **write** runs a handler. The read
branch of the address-match callback re-arms the transmit from the start of the
buffer and changes nothing in it, so a bare read is idempotent: repeat it and the
same bytes come back, indefinitely, until the next command replaces them. The
response behaves like a re-readable snapshot register.

Two constraints come with it. The re-read always restarts at byte 0 — there is no
way to fetch the payload alone, so the second transaction re-reads `STATUS` and
`DATA_LEN` as well. And **the master must not send any command between the two
reads**: a command overwrites the buffer, and for the three commands that consume
what they report (`Read ACC motion data`, `Read ACC temperature`, `Read GPS
data`) the measured data is then gone from both the buffer and the queue it came
from.

Per-command lengths are listed in [Response Lengths](#23-response-lengths).
Where the amount of real data varies the two commands differ: `Read GPS data`
pads its payload to a constant 32 bytes, while `Read ACC motion data` reports
the valid byte count in `DATA_LEN` and leaves the rest stale.

### 2. Command Definitions:
[Protocol Header](Core/Inc/protocol.h)

### 2.1 Command & Response Format:

Command structure:
| CMD (1B) | SENSOR_ID (1B) | DATA_LEN (1B) | DATA (N Bytes) |

Response structure:
| STATUS (1B) | DATA_LEN (1B) | DATA (N Bytes) |

`STATUS`: `0x00` = OK, `0x01` = error (unknown command, unknown sensor ID, or a
command-specific failure documented below).

`DATA_LEN` is one byte in both directions and the buffers cover its whole range,
so a payload of up to 255 is always accepted on a write and always readable on a
read. There is no length a master can put in the field that the slave rejects.

Multi-byte values are little-endian unless stated otherwise.

### 2.2 Examples:

| Command Description           | Command                  | Expected Response                           |
|-------------------------------|--------------------------|--------------------------------------------|
| Turn ON LED                    | {0x10,0x01,0x00,{}}     | {0x00,0x00,{}}                             |
| Turn OFF LED                   | {0x11,0x01,0x00,{}}     | {0x00,0x00,{}}                             |
| Read LED status                | {0x12,0x01,0x00,{}}     | {0x00,1,{0x01}} (0x01=ON, 0x00=OFF)        |
| Read IR data                   | {0x12,0x02,0x00,{}}     | {0x00,4+N×10,{uint32 now, N × (uint32 timestamp, int16 presence, int16 motion, int16 tAmb)}}, N = 0..5 |
| Read IR config                 | {0x13,0x02,0x00,{}}     | {0x00,0x00,{}} — placeholder, no configuration |
| Read ACC motion data           | {0x12,0x03,0x00,{}}     | {0x00,4+N×10,{uint32 now, N × (uint32 timestamp, int16 x, int16 y, int16 z)}}, N = 0..8 |
| Read ACC temperature           | {0x12,0x0A,0x00,{}}     | {0x00,4+6×N,{uint32 now, N × (uint32 timestamp, int16 temp)}}, N = 0..1 |
| Read ACC config                | {0x13,0x03,0x00,{}}     | {0x00,0x00,{}} — placeholder, no configuration |
| Read GPS data                  | {0x12,0x04,0x00,{}}     | {0x00,32,{32 raw NMEA bytes}} / {0x01,32,{padding}} if nothing queued |
| Read GPS config                | {0x13,0x04,0x00,{}}     | {0x00,0x00,{}} — placeholder, no configuration |
| Read battery status            | {0x12,0x05,0x00,{}}     | {0x00,7,{flags, int16 ibat, uint16 vbat, uint16 vbus}} |
| Read current time              | {0x12,0x06,0x00,{}}     | {0x00,6,{YY,MM,DD,HH,MM,SS}}              |
| Sync RTC from GPS              | {0x13,0x06,0x00,{}}     | {0x00,0x00,{}}                             |
| Read interrupt status          | {0x12,0x07,0x00,{}}     | {0x00,5,{SOURCES, MCU, IR, ACC, RTC}}       |
| Read interrupt config          | {0x13,0x07,0x00,{}}     | {0x00,6,{EN_SOURCES, PWR_SOURCES, EN_MCU, EN_IR, EN_ACC, EN_RTC}} |
| Set interrupt config           | {0x13,0x07,0x06,{EN_SOURCES, PWR_SOURCES, EN_MCU, EN_IR, EN_ACC, EN_RTC}} | {0x00,6,{EN_SOURCES, PWR_SOURCES, EN_MCU, EN_IR, EN_ACC, EN_RTC}} |
| Set daily alarm                | {0x13,0x08,0x03,{HH,MM,SS}} | {0x00,0x00,{}}                         |
| Cancel alarm                   | {0x11,0x08,0x00,{}}     | {0x00,0x00,{}}                             |
| Read armed alarm               | {0x12,0x08,0x00,{}}     | {0x00,3,{HH,MM,SS}}                        |
| Turn OFF SoM                   | {0x11,0x09,0x00,{}}     | {0x00,0x00,{}}                             |

Notes on individual commands:

- **Read IR config** is a placeholder. There is no configuration to report, and
  a payload sent to set one is discarded.
- **Read IR data** opens with a snapshot of the MCU timebase and then hands out
  captured samples, oldest first, as fixed 10-byte records with no padding:

  | Byte | Field | Encoding |
  |------|-------|----------|
  | 0-3  | now | uint32, 25 µs per LSB, MCU timebase at this read |
  | 4-7  | timestamp | uint32, same units and timebase, when sample 0 was read out |
  | 8-9  | presence | int16, raw algorithm output |
  | 10-11 | motion | int16, raw algorithm output |
  | 12-13 | tAmb | int16, ambient temperature, hundredths of °C |
  | 14.. | | further samples, 10 bytes each |

  `DATA_LEN` is `4 + N*10`, from `4` to `54` — so `(DATA_LEN - 4) / 10` gives
  the number of samples. Timestamps follow the same rule as the accelerometer's, and `now` is present even when no samples are waiting.

  **The read consumes what it returns**, and the buffer holds 30 samples. It
  carries no interrupt information and clears none — that belongs to
  `Read interrupt status`.

  The MCU reads one sample every second from the main loop, matching the
  sensor's 1 Hz output data rate, so 30 samples is 30 s; older samples are
  overwritten. See [Infrared Sensor](#32-infrared-sensor-ir).
- **Read ACC motion data** opens with a snapshot of the MCU timebase and then
  hands out captured samples, oldest first, as fixed 10-byte records with no
  padding:

  | Byte | Field | Encoding |
  |------|-------|----------|
  | 0-3  | now | uint32, 25 µs per LSB, MCU timebase at this read — see below |
  | 4-7  | timestamp | uint32, same units and timebase, when sample 0 was captured |
  | 8-9  | x | int16, raw, 0.061 mg/LSB at ±2g |
  | 10-11 | y | int16, raw |
  | 12-13 | z | int16, raw |
  | 14.. | | further samples, 10 bytes each |

  `DATA_LEN` is `4 + N*10`, from `4` to `84` — so `(DATA_LEN - 4) / 10` gives
  the number of samples. A record is never split across two reads.

  **`now` is always present**, including when no samples are waiting, in which
  case `DATA_LEN` is `4` and the four bytes stand alone. It is sampled once the
  command has been decoded, so it marks when the master asked.

  **`timestamp` is when the sample was captured, on the MCU's own timebase**, in
  25 µs units. It is absolute, not an age: every read reports the same value for
  the same sample, and samples from different reads share one scale, so the
  master can order and space them against each other without tracking which read
  they arrived in.

  **Age is `now - timestamp`**, in the same 25 µs units, and `now` comes from the
  same counter in the same response, so no clock of the master's is involved.

  The counter starts when the MCU starts and **wraps every 29.8 hours**. Take
  differences modulo 2³² and read the result signed — the shorter of the two
  intervals a difference could stand for is the correct one for anything under
  14.9 hours old. A sample captured after `now` was sampled reports a small
  negative age, which is not an error.

  The MCU timebase is not the RTC and carries no calendar meaning, and it does
  not survive a reset, so timestamps from either side of one are not comparable.
  A reset is visible as `0x01` in the MCU detail byte of `Read interrupt status`.

  **Consecutive records are not guaranteed one sample period apart.** Samples
  are dropped when the sensor's FIFO fills before the MCU drains it, which
  leaves a gap between two records that are then adjacent in the response. Use
  each record's own `timestamp`; do not reconstruct times by counting records.

  **The read consumes what it returns.** Poll it repeatedly to walk the capture
  and stop when `DATA_LEN` comes back `4`; each read yields at most 8 samples, so
  draining a full buffer takes 13 reads. Sending another command before the data
  has been read loses it — see
  [How many bytes to read](#how-many-bytes-to-read--read-this-before-writing-master-code).

  `STATUS` is always `0x00`; an empty buffer is reported as `DATA_LEN = 4`, the
  snapshot alone, not as an error.

  The MCU drains the sensor's FIFO every 100 ms from the main loop, so samples
  accumulate continuously. The buffer holds 104 samples, 250 ms at the current
  416 Hz output data rate; older samples are overwritten. See
  [Accelerometer](#31-accelerometer).
- **Read ACC temperature** returns the accelerometer's die temperature and the
  time it was captured:

  | Byte | Field | Encoding |
  |------|-------|----------|
  | 0-3  | now | uint32, 25 µs per LSB, MCU timebase at this read |
  | 4-7  | timestamp | uint32, same units and timebase, when the reading was captured |
  | 8-9  | temp | int16, 256 LSB/°C, `0` = 25 °C, so °C = 25 + temp/256 |

  `DATA_LEN` is `10` when a reading is waiting, `4` when none is — none captured
  since reset, or the last one already read. `now` is present either way.
  `STATUS` is always `0x00`.

  `timestamp` is on the same MCU timebase as the motion samples, so a
  temperature can be placed directly against a capture without either read
  having to happen at any particular time, and its age is `now - timestamp`
  exactly as for a motion sample.

  **The read consumes the reading**, so a second read returns `DATA_LEN = 4`
  until the next one arrives. Temperature is batched at 1.6 Hz, so a fresh
  reading turns up about every 0.6 s.

  The offset is untrimmed at **±15 °C** — see [Temperature](#temperature).
- **Read ACC config** is a placeholder. There is no configuration to report, and
  a payload sent to set one is discarded.
- **Set daily alarm** takes three binary bytes, 24-hour: hour, minute, second.
  The alarm has **no date** — it matches that time of day, every day, because the
  hardware alarm can compare a day-of-month or a weekday and a time, and nothing
  wider. It stays armed after it fires, so it comes round again the next day.

  `STATUS = 0x01` means the payload was not three bytes, or a field was out of
  range (hour > 23, minute or second > 59). Nothing is changed in that case: an
  alarm that was already armed stays armed.

  The alarm and its interrupt enable live in the RTC's backup domain, so **an
  armed alarm survives an MCU reset** for as long as VBAT holds. That is
  deliberate — it is what lets the alarm wake a SOM that was off when the MCU
  restarted — but it means the master cannot assume a fresh MCU has no alarm. Use
  `Read armed alarm` to find out.
- **Cancel alarm** is `Turn OFF` on the alarm sensor ID. There is no magic time
  value that means "cancel", so `00:00:00` is settable like any other time.
- **Read armed alarm** reports the time of an armed alarm, if any.
  `STATUS = 0x01` means no alarm is currently armed, and the time bytes that follow
  are without meaning.
- **Read interrupt status** is described in full under
  [What the interrupt status register contains](#what-the-interrupt-status-register-contains).
  It is the only command that clears interrupt state, and the only one that releases
  the `MCU_INT` line.
- **Interrupt config** is one command for both directions. An empty payload
  reads the configuration, a six-byte payload replaces it. The response carries
  the configuration in effect after the command:

  | Byte | Field | Meaning |
  |------|-------|---------|
  | 0 | `EN_SOURCES` | which sources may be reported at all, same bits as `SOURCES` |
  | 1 | `PWR_SOURCES` | which of the reported sources also power the SOM on, same bits as `SOURCES` |
  | 2 | `EN_MCU` | which MCU events may be reported, same bits as the MCU detail byte |
  | 3 | `EN_IR` | which IR events may be reported, same bits as the IR detail byte |
  | 4 | `EN_ACC` | which accelerometer events may be reported, same bits as the ACC detail byte |
  | 5 | `EN_RTC` | which RTC events may be reported, same bits as the RTC detail byte |

  An event is reported when it passes its detail mask and its source bit in
  `EN_SOURCES`. One that does not is neither latched nor reported. `PWR_SOURCES`
  applies to events that passed both masks.

  Defaults after reset are `0xFF, 0x0E, 0xFF, 0xFF, 0xFF, 0xFF`: every event
  reported, and IR, accelerometer and RTC alarm power the SOM up.

  `STATUS = 0x01` means the payload was neither empty nor six bytes; the
  configuration is unchanged and `DATA_LEN` is `0`. The configuration lives in
  RAM and does not survive an MCU reset.
- **Read GPS data** returns **raw NMEA bytes**, not a parsed position. It is a
  passthrough: the MCU does not decode coordinates at all. The payload is **always
  32 bytes** — packed with as many queued sentences as fit and padded with newlines
  if there were not enough. `DATA_LEN` indicates the real length before padding.
  `STATUS = 0x01` means the queue was empty and the 32 bytes are all
  padding. Either read 34 bytes, or split the read into two transactions:
  1. 2 bytes for header, 2. `2 + DATA_LEN` for header with data.
  Sending a command after short read discards the `DATA_LEN` bytes of data.
  See [GPS NMEA Passthrough](#24-gps-nmea-passthrough) — reading this command
  correctly requires more than the table row above.
- **Read GPS config** is a placeholder. There is no configuration to report, and
  a payload sent to set one is discarded.
- **Read current time** returns **binary** values (not BCD). Hour 23 is `0x17`.
  `STATUS = 0x01` means the calendar has never been set from GNSS — the six data
  bytes are still returned, but they are the RTC's power-on default and mean
  nothing.

  `STATUS` survives a reset. It is backed by a backup register (`BKP_DR1`), not a
  RAM flag, so the MCU does not claim the time is unverified just because it
  rebooted. The marker is lost only when VBAT is lost — which is also when the
  calendar itself stops, so the two always agree. See
  [Timekeeping](#25-timekeeping).
- **Sync RTC from GPS** requests a re-sync and returns immediately. It clears the
  synchronised marker; the next valid RMC re-sets the calendar, which with a fix
  is within about a second. Poll `Read current time` and watch `STATUS`.

  This command is **not** required for the MCU to know the time. The MCU syncs
  itself from GNSS with no involvement from the master; the command exists to
  force it early.
- **Turn ON** is implemented for `SENSOR_LED` only.
- **Turn OFF** is implemented for both `SENSOR_LED` and `SoM` (CPU).
- **Turn OFF SoM** must cut power unless fatal internal error occured, since it is too late for host to reconsider.
  Cutting power must be delayed by 1s after i2c response, giving sufficient time for host to process final interrupts.

  Initial power-state follows pull-up/pull-down assembly options: R111 assembled = default on, R13 assembled = default off.

### 2.3 Response Lengths:

Total bytes the master should read (`2 + DATA_LEN`):

| Command | Bytes to read | Typical latency |
|---------|---------------|-----------------|
| `0x10` / `0x11` (LED on/off) | 2 | immediate |
| `0x12,0x01` (LED status) | 3 | immediate |
| `0x12,0x02` (IR data) | 56 — read all, use `DATA_LEN` | immediate — served from RAM, no bus access |
| `0x12,0x03` (ACC motion data) | 86 — read all, use `DATA_LEN` | immediate — served from RAM, no bus access |
| `0x12,0x0A` (ACC temperature) | 12 | immediate — served from RAM, no bus access |
| `0x12,0x04` (GPS data) | 34 — always | immediate — a copy out of RAM, no bus access |
| `0x12,0x05` (battery) | 9 | five I2C1 register reads |
| `0x12,0x06` (time) | 8 | immediate |
| `0x12,0x07` (interrupt status) | 7 | immediate — snapshots RAM latches, no bus access |
| `0x13,0x07` (interrupt config) | 8 | immediate |
| `0x12,0x08` (armed alarm) | 5 | immediate |
| `0x11,0x08` (cancel alarm) | 2 | immediate |
| `0x11,0x09` (power-off som) | 2 | response immediate, power-off after 1s |
| `0x13,*` (config) | 2 | immediate; alarm writes the RTC |

### 2.4 GPS NMEA Passthrough:

The GNSS receiver (u-blox MIA-M10) is on **I2C3** and is reachable only from the
MCU — the SOM has no direct connection to it, and there is no UART. The MCU does
not parse position. It reads the module's NMEA stream and hands whole sentences
to the master unchanged, so gpsd on the SOM does the framing and filtering it is
already better at than we would be.

#### How the MCU fills the queue

- The main loop reads a 64-byte block from the module's DDC (I2C) interface every
  20 ms and discards `0xFF` idle bytes. NMEA is 7-bit ASCII, so `0xFF` can never
  be real data and needs no length register to disambiguate it.
- Framing resynchronises on `$`, so a block that starts mid-sentence loses only
  that sentence.
- Only sentences whose checksum verifies are queued, and they are queued **whole**.
  Malformed, truncated and over-long sentences never reach the master.
- No sentence-type filtering. Everything the module emits is passed on.

#### What the master must do

1. **Always read exactly 34 bytes** (`2 + 32`). The payload is a fixed 32 bytes,
   packed from as many queued sentences as fit and padded with newlines when the
   queue runs dry mid-payload. Reading a variable amount is not possible — the
   master cannot learn `DATA_LEN` before it reads, and over-reading hangs the bus.

   ```
   i2ctransfer -y $bus "w3@$addr" 0x12 0x04 0x00 r34@$addr
   ```
2. **Concatenate and split on `\n`.** Sentences are stored complete, including the
   leading `$` and the trailing `*hh\r\n`, so the master frames exactly as it would
   on a serial port. A payload may hold several sentences, or the middle of one —
   a sentence with a fix is 60–80 bytes and spans two or three reads. The newline
   padding shows up as empty lines, which any NMEA framer discards. Padding never
   appears in the middle of a sentence.
3. **Poll until `STATUS = 0x01`, then back off.** `0x01` means the queue was empty
   and the payload is all padding. Sleeping about 100 ms at that point is a good
   default — see the polling budget below.
4. **Come back at least once a second.** The module emits its whole batch in a
   once-per-second burst of about ten sentences, and the queue holds 11. Leave a
   gap longer than a second and sentences are lost.

#### Polling budget

Measured on the SOM: a plain shell loop of `i2ctransfer` manages about **128
reads/s**, and at that rate **87% of reads return `STATUS = 0x01`** — the queue is
empty almost every time.

That is far more than needed, and free-running polling is expensive on the bus. One
transaction is 3 bytes written plus 34 read plus overhead, roughly 40 bytes, which
at 100 kHz is about 4 ms. 128 reads/s therefore occupies **about half the bus**,
almost all of it returning padding.

| Polling strategy | Reads/s | Bus occupancy | Headroom vs. a fix |
|---|---|---|---|
| Free-running loop | ~128 | ~50% | 4× |
| Drain, then sleep 100 ms | ~20–30 | ~10% | 1–2× |
| Once per second only | ~1 | negligible | loses data |

Drain-then-sleep is the recommended shape: read repeatedly until `STATUS = 0x01`,
sleep ~100 ms, repeat. That keeps up with a fix and leaves the bus alone.

#### Freshness, and what gets lost

When the queue is full the **oldest** sentence is discarded to make room for the
newest, so a master that polls occasionally always gets the most recent ~1 s of
traffic rather than a stale snapshot from whenever the buffer first filled.
Everything older than that is gone.

A master that polls once a minute therefore sees a stream with 59-second gaps.
That is by design — every NMEA sentence carries its own UTC, so the master can
tell how old a fix is without help from this protocol.

Reclaiming a slot while the master is part-way through reading it costs one
truncated line, which fails its own NMEA checksum on the master side and is
discarded there. This only happens if the master stops mid-sentence for longer
than it takes the queue to wrap.

#### Debugging trap: identical consecutive reads

If the master is idle between reads, **two reads seconds apart can return
byte-identical payloads, and that is not a fault.** The queue holds 11 sentences,
which is about the size of one of the module's once-per-second bursts, so once it
is saturated it always holds the same slice of the same repeating batch — the phase
is locked. With no fix every field is empty, so the bytes really are the same.

To see the queue actually advance, issue several reads **back to back** rather than
one every few seconds. Consecutive reads walk through the queue and reassemble into
a continuous stream. `nmea_bytes_out` in the debugger settles it from the firmware
side: if it advances by 32 per command, the command is being processed.

#### Measured rates

| Condition | Byte rate | Sentences/s | Reads/s to keep up |
|---|---|---|---|
| No fix (empty fields) | ~370 B/s | ~10 | ~12 |
| With a fix | ~2–3× that | ~10 | ~25–35 |

The sentence *rate* does not change with a fix — the sentences get longer because
the fields fill in. Because the payload is packed rather than one-sentence-per-read,
the read count tracks the byte rate divided by 32.

#### Firmware-side diagnostics

Globals in `Core/Src/nmea.c` and `Core/Src/ublox.c`, useful in a debugger:

| Symbol | Meaning |
|---|---|
| `nmea_accepted` | valid sentences queued |
| `nmea_dropped_old` | oldest sentences discarded for room. **Climbs whenever the master is not polling, which is normal** — it is not an error counter. Judge it against `nmea_pop_empty`. |
| `nmea_pop_empty` | times the queue was drained dry. Climbing means the master is keeping up; static while `nmea_dropped_old` climbs means it is not. |
| `nmea_bytes_out` | bytes handed to the master; compare with `ublox_bytes_total` |
| `nmea_bad_csum` | checksum or framing failures; should stay 0 |
| `nmea_overlong` | sentence exceeded `NMEA_MAX_SENTENCE`; should stay 0 |
| `nmea_torn` | a master read cut short by a reclaim |
| `nmea_time_updates` | RMC times accepted into the RTC |
| `nmea_time_nofix` | of those, how many arrived with no position fix. Stops rising once a fix is held |
| `ublox_pump_calls` | blocks read; `× 64` must equal `bytes_total + filler_total` |
| `ublox_bytes_total` / `ublox_filler_total` | real bytes / `0xFF` idle bytes |
| `ublox_err_count` | failed I2C3 transfers |

### 2.5 Timekeeping:

The MCU keeps its own time and **does not depend on the SOM** to tell it what the
time is. The only path into the calendar is GNSS.

- The RTC is clocked from the **LSE crystal** (Y3, 32.768 kHz), not the internal
  LSI. LSI is an RC oscillator at roughly ±5%; the crystal is 20 ppm — about
  1.7 s of drift per day instead of 72 minutes.
- LSE and the calendar both sit in the **backup domain**, powered from VBAT, so
  the clock is *designed* to keep running while the main rail is off.
  **Verified:** an MCU reset preserves the calendar. **Not yet verified:** that it
  survives with the main rail down and the cell still connected — that needs SW1
  opened for a measured interval, not the supply unplugged. Removing the supply
  altogether takes VBAT with it and clears the calendar, which is expected.
- **The calendar holds UTC**, not local time. No timezone or DST is applied
  anywhere in the firmware.
- UTC is taken from the `RMC` sentence as it passes through the passthrough
  framer, so timekeeping does not depend on the master draining the queue.
- The calendar is set on the first acceptable `RMC`, and re-synced **once an hour**
  after that. An hour of LSE drift at 20 ppm is 72 ms.
- `MX_RTC_Init` checks `BKP_DR0` and leaves an already-running calendar alone, so
  a reset does not reset the clock.
- `BKP_DR1` records that the calendar came from GNSS. This is what `STATUS` on
  `Read current time` reports, which is why that status survives a reset.

#### What counts as an acceptable RMC — and why `status` is not part of it

An `RMC` is accepted when its **time and date fields are both populated and
well formed**. `RMC.status` is deliberately *not* tested.

`status` is a **position** flag: u-blox sets it to `A` only for a valid position
fix, and `V` otherwise — including when the receiver knows the time perfectly well
but is tracking too few satellites to place itself. Indoors that is every epoch, so
testing `status` would mean never setting the clock.

The validity signal is the field itself. `CFG-NMEA-OUT_INVTIME` and
`CFG-NMEA-OUT_INVDATE` default to false on the M10, which means an invalid time or
date is emitted as an **empty field** rather than a guess. u-blox document this
directly:

```
$GPGLL,,,,,124924.00,V,N*42     <- invalid position, VALID time
$GPGLL,,,,,,V,N*64              <- time unknown (cold start)
```

So "time and date both present" is equivalent to `validDate && validTime` in
UBX-NAV-PVT, which is exactly what the firmware's earlier UBX-based path tested.
Note that `validDate` can lag `validTime`, so an `RMC` with a populated time and an
empty date is a legitimate state — both are required here.

> ⚠️ **Do not enable `CFG-NMEA-OUT_INVTIME` or `CFG-NMEA-OUT_INVDATE` on the
> module.** They permit "the receiver's best knowledge of time to be output, even
> though it might be wrong", which would make a populated field stop meaning
> anything and silently break this scheme.

`nmea_time_nofix` counts how many of the accepted times arrived without a position
fix. Once a fix is held, `nmea_time_updates` keeps rising while `nmea_time_nofix`
stops — which is a cheaper fix indicator than parsing NMEA on the master side.

#### After a power cycle, expect `STATUS = 0x01` for a while

Removing power cold-starts the receiver: it loses its ephemeris, almanac and
last-known time, so it must decode the navigation message from a satellite again.
That takes tens of seconds of continuous tracking with a usable signal, and indoors
it may not happen at all. Until it does, the time field in NMEA stays **empty**, the
firmware correctly refuses to set the clock, and `Read current time` reports
`STATUS = 0x01`.

This is the most common reason for an apparently dead clock, and it is not a
firmware fault — check the GGA satellite count and whether the time field is
populated before looking anywhere else. Note also that if the module's own backup
supply (`V_BCKP`) is not maintained on the board, every power cycle is a full cold
start rather than a warm one.

#### Two things the firmware cannot tell you

**Sub-second accuracy before a fix.** NMEA carries no equivalent of NAV-PVT's
`fullyResolved` bit or its `tAcc` estimate. Until the receiver has resolved the
leap second, u-blox warn that "plausible times are nearly always generated, but
they may be wrong by a few seconds". Closing that gap means enabling
`CFG-MSGOUT-UBX_NAV_PVT_I2C` and parsing UBX alongside NMEA on the same DDC stream
— possible, since both protocols interleave in the 0xFF stream, but not done.

**Age.** `STATUS = 0x00` means "set from GNSS, and VBAT has not been lost since" —
it does not say when. With normal sky view the hourly re-sync keeps the error within
72 ms; after days with no reception it could be seconds. If the master needs the age
of the last sync, that would be a protocol addition.

### 3. Sensor Thresholds & Configuration:

### 3.1 Accelerometer:

Sensor: ISM330DHCX on I2C1. Wake-up, free-fall and tilt detection all
routed to INT1.

All three are latched in the device, so an event holds until the MCU reads it
and is reported to the master by `Read interrupt status`.

Tilt answers to a **sustained** change of orientation, not a transient one:
moving the board and returning it leaves the net orientation unchanged and
reports nothing. Measured: taken from flat to its edge and left there, it
reports the tilt bit on its own, with no motion bit.

**Output data rate: 416 Hz**, set from `ACC_ODR_HZ`. Full scale ±2g.

#### Sample capture

The sensor's own 3 kB FIFO batches accelerometer samples at the output data
rate and the die temperature at 1.6 Hz, with a timestamp word every 8th
sample. The MCU drains it into a 104-sample ring buffer **from the main loop**,
every 100 ms, so samples accumulate continuously whether or not anything is
moving; 104 samples is **250 ms** at 416 Hz. `Read ACC motion data` hands that
ring to the master and touches no bus.

Samples are stamped on the MCU timebase as they leave the FIFO. The drain reads
that timebase and the sensor's own timestamp counter back to back and adds the
offset between them to every stamp it reconstructs.

Inside the device only every 8th sample carries a real timestamp; the seven in
between are interpolated as `anchor + k/ODR`, 96 LSB apart at 416 Hz. Measured
on hardware over an undisturbed 52-sample capture: every interval exactly 96
LSB, contiguous across successive reads with no repeats. The sensor's FIFO
discards its oldest entries once full, so a late drain loses samples and the
gap falls between two records the master receives back to back.

Events in the accelerometer detail byte of `Read interrupt status`:

| Bit  | Source | Meaning |
|------|--------|---------|
| 0x01 | `WAKE_UP_SRC` | motion — any of X, Y, Z over the threshold |
| 0x02 | `EMB_FUNC_STATUS_MAINPAGE` | tilt |
| 0x04 | `WAKE_UP_SRC` | free-fall |

Which axis moved is not reported, and bit 3 upwards is unused: the device also
reports the wake-up axes individually, a wake-up summary and activity-state
changes, and none of those is configured well enough to act on.

Two registers are read, not one: tilt is an embedded function and reports
in its own status register. Neither is read through `ALL_INT_SRC`, because
reading that register clears `WU_IA` before `WAKE_UP_SRC` can be read.

#### Tilt

The wake-up detector runs on the high-passed signal, so it responds to
*change* and not to position: a slow tilt changes the orientation completely
without crossing the wake-up threshold. Tilt detection covers that case.

Enabling it takes two writes: `tilt_en` in `EMB_FUNC_EN_A` and `tilt_init` in
`EMB_FUNC_INIT_A` (`0x66`). The algorithm does not start on the enable alone,
and ST's driver exposes no setter for the init register, so the firmware writes
it directly.

The interrupt is latched with the base functions
(`ISM330DHCX_ALL_INT_LATCHED`), so the status holds until the MCU reads it.
That mode is set through the register layer rather than
`ISM330DHCX_Set_Interrupt_Latch()`, whose `uint8_t` argument rejects anything
above 1 and so cannot express it.

Measured: a slow change to about 90°, held there, reports the tilt bit and no
motion bit; a tilt that returns the board to where it started reports nothing;
reading the status returns the bit once and `0x00` after, and later tilts still
report.

#### Free-fall

`ISM330DHCX_ACC_Enable_Free_Fall_Detection()` runs with the threshold at ST's
312 mg and the duration at 15 samples, 36 ms at 416 Hz, from
`ACC_FF_DURATION_MS`. ST's own default is 6 samples.

`ff_ia` is reported as `0x04` and, like motion, triggers a FIFO drain.
Measured: it sets when the board is lifted sharply, alongside the wake-up bits
from the same movement. The threshold is untuned against a real fall.

#### No sample read touches the bus

`Read ACC motion data`, `Read ACC temperature` and `Read IR data` all copy out
of RAM, including the timestamps: the sensor has already been read by the time
the SOM asks, and the snapshot each read returns comes from the MCU timebase
alone.

The I2C2 slave callback and the two sensor EXTI handlers all sit at NVIC
priority 1, so a sensor read in the event handler cannot collide with a slave
callback already in progress.

#### Temperature

The die temperature comes from the same FIFO as the motion samples, so it
costs no extra bus transaction. It is reported **raw**, as
[`Read ACC temperature`](#22-examples) describes: 256 LSB/°C with `0` meaning
25 °C. The MCU does no conversion.

It is the sensor's own die, not the air. Measured 49.0 °C against the
STHS34PF80's ambient channel at 50.2 °C, which cross-checks both conversion
formulas.

`Toff` is untrimmed at **±15 °C** part to part, so this channel tracks
*change* well and absolute temperature poorly. Use `Read IR data`'s ambient
channel where the absolute number matters.

Wake-up threshold: `ACC_THS_DEFAULT = 0x04`, set at build time.

Trigger values: 0–63 (1 LSB = fraction of ±2g full scale)

Threshold (mg) = FS(g) × (threshold / 64) × 1000

| Threshold | FS(g) = ±2g (mg) |
|-----------|-----------------|
| 1         | 31 mg           |
| 2         | 62 mg           |
| 4         | 125 mg          |
| 8         | 250 mg          |
| 16        | 500 mg          |
| 32        | 1 g             |
| 63        | 1.97 g          |
      

##### Recommended values:

1–3: Very sensitive (tiny motion)

4–8: Medium motion (walking, light shake)

10–20: Strong motion (hit, fall)


### 3.2 Infrared Sensor (IR):

Sensor: STHS34PF80 on I2C1, continuous mode at 1 Hz, presence and motion routed
to a single interrupt line (`INT_OR`).

The sensor has no FIFO, so the MCU reads one {presence, motion, tAmb} sample
every second from the main loop into a 30-entry ring buffer, stamped on the MCU
timebase as it is read. `Read IR data` hands that ring to the master and touches
no bus.

Interrupt code:

| Bit  | Source | Meaning |
|------|--------|---------|
| 0x01 | `FUNC_STATUS.mot_flag` | motion |
| 0x02 | `FUNC_STATUS.pres_flag` | presence |

`Read IR data` also returns the sensor's own ambient channel, in hundredths
of a degree Celsius. Sensitivity is 100 LSB/°C, so the raw register value
already is hundredths and is passed straight through. Like the
accelerometer's, this reads the sensor's own package rather than the air —
the two are a cross-check on each other's scaling, not two measurements of
room temperature. `IR_SENSOR_ReadTObject()` and `IR_SENSOR_ReadTAmbShock()`
exist in the firmware and still have no command.

Frequency [Hz]= 1Hz = 1000ms

Hysteresis default: **HYST = 50** (the datasheet writes it as `32h`)

Sensor sensitivity: 2000 LSB/°C, so 1 LSB of threshold ≈ 0.0005 °C.

Detection:

Compares two internally filtered signals

Event flag set if difference > threshold

Flag cleared when signal < (threshold − hysteresis)

| Threshold | Hysteresis | Approx. Signal Change | Use Case                           |
|-----------|------------|---------------------|-----------------------------------|
| 100       | 50         | ≥0.05°C             | Very sensitive, long range        |
| 150       | 50         | ≥0.075°C            | Sensitive, moderate range         |
| 200       | 50         | ≥0.1°C              | Balanced sensitivity/stability    |
| 250       | 50         | ≥0.125°C            | Less sensitive, shorter range     |
| 300       | 50         | ≥0.15°C             | Low sensitivity, short range      |
| 400       | 50         | ≥0.2°C              | Minimal sensitivity, very stable  |

> **The firmware uses `IR_THS_DEFAULT = 1000` (≈0.5 °C), set at build time and
> outside the table above.** 1000 is the value the board has been running. The
> 100–400 range in the table comes from the sensor sensitivity figure and has not
> been validated on this hardware. Treat the table as a starting point for
> tuning, not as tested settings.
>
> The firmware never writes the hysteresis registers, so hysteresis stays at the
> sensor default of 50. Note that ST pairs 50 with a threshold of 200 — a ratio of
> 25%. Against the firmware's 1000 the ratio is 5%, which leaves only a narrow band
> (flag sets at 1000, clears at 950) and can make the flag chatter when the signal
> settles near the threshold. If 1000 is kept, a hysteresis around 250 restores
> ST's ratio.

Both `PRESENCE_THS` and `MOTION_THS` are written with the same value.


### 3.3 Battery Charging:

Charger: BQ25638 on I2C1. The response is seven bytes:

| Byte | Field | Encoding |
|---|---|---|
| 0 | `flags` | Bitmask, see below |
| 1-2 | `ibat` | Battery current in **mA**, `int16` (range -10000 - +5025). **Signed** — negative is discharging |
| 3-4 | `vbat` | Battery voltage in **mV**, `uint16` (range 0 - 5000) |
| 5-6 | `vbus` | Supply (J1 DC Jack or J3 USB) voltage in **mV**, `uint16` (range 0 - 20000) |

The three measurements are reported in their natural units, at the resolution the
charger's ADC provides. All three
are 16-bit **little-endian**, low byte first, per [Command & Response Format](#21-command--response-format).

`ibat` is signed (two's complement).

FLags are as follows:

| Bit | Encoding |
| --- | -------- |
|   0 | Power Supply connected (J1 DC Jack or J3 USB)
|   1 | Battery is charging |
|   2 | Power supply has fault (over-voltage) |
|   3 | Battery has fault (dead or over-voltage) |

### 4. Known Limitations:

Current firmware behaviour the master side should be aware of.

- **Commands are executed inside the I2C2 interrupt handler**, and most of them
  perform blocking reads on I2C1. A command therefore holds the I2C2 interrupt for
  as long as the sensor access takes. Size the master's I2C timeout accordingly.
  The exceptions are `Read GPS data` and the three sample reads, which copy out
  of RAM and touch no bus.
- **The MCU timebase is not tied to the RTC and does not survive a reset**, so a
  timestamp from before a reset is not comparable with one from after. A reset is
  visible as `0x01` in the source and MCU detail bytes of `Read interrupt status`.
- **A read before any command has been sent** returns zeros rather than an error:
  `STATUS = 0x00`, `DATA_LEN = 0x00`, and a zeroed tail because the transmit
  buffer starts in `.bss`. It is indistinguishable from a successful command that
  produced no payload.
- **`Turn ON` / `Turn OFF` with a sensor ID other than `SENSOR_LED`** return
  `STATUS = 0x00` without doing anything.
- **Only `Read interrupt status` clears interrupt state.** Neither `Read IR data`
  nor the accelerometer reads carry or clear it; the accelerometer's event bits
  are reported solely by `Read interrupt status`. The accelerometer notifies the
  SOM once per new event, not on every interrupt while an unread bit sits in the
  latch.
- **The stuck-bus watchdog fires after ~10 s**, not 10 ms: the counter is
  driven by TIM6 at 1 Hz while the timeout constant is named as milliseconds.
  If the master dies mid-transaction, expect the MCU to be unresponsive for about
  ten seconds before it recovers.

  Recovery itself works: `resetI2C2()` calls `HAL_I2C_DeInit()` / `MX_I2C2_Init()`,
  and both of those clear the `PE` bit in `CR1`, which is this peripheral's
  documented software reset — it returns the internal state machine and status bits
  to their reset values and releases SCL and SDA. Note that this only covers the
  case where the MCU's own peripheral is holding SCL (clock stretching while waiting
  for a master that went away). If an external device on the bus is holding the lines
  down, no reset of the MCU's peripheral can clear that.
- **The GNSS queue holds about one second** — 11 sentences, which is one of the
  module's once-per-second bursts with no margin. A master that polls infrequently
  sees an intermittent stream rather than a continuous one, and `nmea_dropped_old`
  climbs. That is the design, not a fault: the newest second is kept and the rest
  is dropped. See [GPS NMEA Passthrough](#24-gps-nmea-passthrough).
- **`Read GPS data` returns partial sentences.** The 32-byte payload is smaller
  than an NMEA sentence with a fix. This is not an error condition and there is no
  flag for it — the master frames on `\n`, as it would on a UART.
- **The slave hangs the bus if the master reads past 257 bytes.** There is no
  "no more data" response; the slave simply stops having bytes and holds SCL. It
  arms the full 257-byte buffer for every command, so any read up to that is
  harmless, including over-reading a short reply — but the 257-byte ceiling is
  absolute, and the bytes past `2 + DATA_LEN` are stale rather than padded. See
  [How many bytes to read](#how-many-bytes-to-read--read-this-before-writing-master-code).
- **No UBX is parsed.** The firmware reads NMEA only, which means the receiver's
  `fullyResolved` / `confirmedTime` / `tAcc` indications are not available — see
  [Timekeeping](#25-timekeeping).
- **The interrupt bytes report events, not present state**, and there is no
  falling-edge handler, so the master is never told when a condition ends. See
  [Every byte answers "what fired", never "what is true now"](#every-byte-answers-what-fired-never-what-is-true-now).

  `FUNC_STATUS` on the STHS34PF80 is not clear-on-read: `tshock`, `mot` and
  `pres` are level flags re-evaluated every ODR cycle, and only the `DRDY` bit
  in `STATUS` (`0x23`) is cleared by being read.
- **Presence and motion currently return the same value.** The STHS34PF80's filter
  bandwidths (`LPF_M`, `LPF_P`, `LPF_P_M`, `LPF_A_T`) are never configured, so they
  stay at their reset divider and the two algorithm outputs are the same signal.
  Motion therefore responds to slow changes that a motion detector should ignore.
- **The IR baseline walks for about twenty seconds after a reset.** Measured:
  presence climbed monotonically from −856 to +235 with nothing moving in front
  of the sensor. With a low threshold that settling alone raises a presence
  event.

  The walk starts with the algorithm reset, which zeroes the internal filters
  and leaves them to charge up to the real signal level.
  `sths34pf80_odr_set()` performs that reset itself — `odr_safe_set()` calls
  `reset_algo_bit_set()` on every transition to an operative ODR — and
  `IR_SENSOR_StartContinuous()` sets the ODR last, after every threshold write.

  What sets the duration is the ODR. Every filter cutoff is a fraction of it,
  `ODR/9` at reset, and the sensor runs at **1 Hz** — a 0.11 Hz cutoff, so
  seconds per time constant. At 30 Hz the same settling would take under a
  second. The same 1 Hz also means a person walking past is one or two samples,
  which is why only something held in front of the sensor moves the numbers.
- **Expect a spurious motion event about a second after every MCU reset**, from the
  same filter transient. Both runs of the measurement above showed it.
- **The IR hysteresis registers are never written**, so hysteresis stays at the
  sensor default of 50 against a threshold of 1000 — a 5% band where ST pairs
  50 with 200, a 25% one. The `INT` line follows the level, so a signal sitting
  near the threshold makes the flag chatter and the line with it.
- **`IR_SENSOR_StartContinuous()` ignores the return of `sths34pf80_odr_set()`,
  which can fail silently.** That function clamps the ODR against the averaging
  setting in `AVG_TRIM` — 1024 averages allows at most 1 Hz, 32 allows 30 Hz —
  and returns −1 without writing anything when the request is too high. Raising
  the ODR without checking can leave the sensor in power-down with no
  indication.
- **There is no `HAL_I2C_ErrorCallback`.** An I2C2 error is left to the stuck-bus
  watchdog above rather than being handled where it happens.
- **`0x13 0x02` (configure IR) does nothing.** It returns `STATUS = 0x00` and
  discards its payload. The presence and motion thresholds are fixed at build
  time.
- **`0x13 0x03` (configure accelerometer) does nothing.** It returns
  `STATUS = 0x00` and discards its payload. The wake-up threshold is fixed at
  build time.
- **`0x13 0x04` (configure GPS) does nothing.** It returns `STATUS = 0x00` and
  discards its payload.
- **Free-fall fires, but the threshold is untuned against a real fall.** It has
  been seen to set on a sharp lift by hand, which is not the same thing. See
  [Free-fall](#free-fall).
- **Some events the device reports are read and dropped.** The wake-up summary
  `wu_ia`, the individual `x_wu`/`y_wu`/`z_wu` axes, `sleep_change_ia` and the
  embedded functions other than tilt are all discarded rather than surfaced, so
  an event in one of those passes unnoticed.
- **The sample buffer holds 250 ms at 416 Hz.** 104 samples is a fraction of a
  captured event, so a master that polls slower than that loses motion. See
  [Sample capture](#sample-capture).
- **A motion event is reported before its samples are in RAM.** The interrupt
  notifies the SOM immediately, but the FIFO is drained on the 100 ms main-loop
  cadence, so a read that arrives first returns what was captured up to the last
  drain.
- **An IR event is reported before its sample is in RAM.** The same holds for
  the infrared sensor on its 1 s main-loop cadence, so a `Read IR data` that
  arrives first returns samples up to a second older than the event.
