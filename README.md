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

    3.1 Accelerometer

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

`Read interrupt status` (`0x12 0x07`) returns three bytes: a source bitfield,
followed by one detail byte per source that has one.

| Byte | Contents |
|------|----------|
| `data[0]` | **Sources.** `0x01` the MCU itself, `0x02` IR, `0x04` accelerometer, `0x08` RTC. `0x10` charger is allocated and is never set yet. |
| `data[1]` | **IR detail** — masked `FUNC_STATUS`: `0x02` motion, `0x04` presence. `0x01` thermal shock is routed off the pin and masked in firmware. |
| `data[2]` | **Accelerometer detail** — `WAKE_UP_SRC` as the device reports it: `0x01` Z, `0x02` Y, `0x04` X, `0x08` the wake-up flag, `0x20` free-fall, `0x40` sleep change. |
| `data[3]` | **RTC detail** — `0x01` alarm A fired. Alarm B, the wakeup timer and tamper are not used and have no bit yet. |

A source occupies its bit whether or not it has a detail byte. `0x01` is raised
once during init, so the first read after a reset reports it — every threshold the
master configured is back at its default and needs setting again. Earlier firmware
gave the master no way to tell a reboot from an idle MCU.

`data[0]` was a plain `0` / `1` in earlier firmware. A master testing `!= 0` is
unaffected; one testing `== 1` now reads that as "the MCU itself".

#### Every byte answers "what fired", never "what is true now"

All three bytes are accumulated latches, and the read clears them. Two sources
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

The slave arms its response with an **exact** length and has nothing to send past
it. **Reading more bytes than the response contains hangs the bus**: the slave runs
out of data mid-transfer and keeps stretching SCL, and the master sees
`SCL is stuck low` / `Connection timed out`. It stays that way until the stuck-bus
watchdog fires, which takes about ten seconds (see
[Known Limitations](#4-known-limitations)).

Every response therefore has a **fixed** length, listed in
[Response Lengths](#23-response-lengths). Read exactly that many bytes. Where the
amount of real data varies — only `Read GPS data` — the payload is padded to a
constant size rather than shortened, so the number of bytes to read never depends
on what is in the reply.

Reading *fewer* bytes than the response contains is tolerated but pointless.

### 2. Command Definitions:
[Protocol Header](Core/Inc/protocol.h)

### 2.1 Command & Response Format:

Command structure:
| CMD (1B) | SENSOR_ID (1B) | DATA_LEN (1B) | DATA (N Bytes) |

Response structure:
| STATUS (1B) | DATA_LEN (1B) | DATA (N Bytes) |

`STATUS`: `0x00` = OK, `0x01` = error (unknown command, unknown sensor ID,
`DATA_LEN` too large, or a command-specific failure documented below).

`DATA_LEN` in a command is capped at 32 (the payload size of `I2C_Command_t`).
A larger value is rejected with `STATUS = 0x01` and no payload is read.

Multi-byte values are little-endian unless stated otherwise.

### 2.2 Examples:

| Command Description           | Command                  | Expected Response                           |
|-------------------------------|--------------------------|--------------------------------------------|
| Turn ON LED                    | {0x10,0x01,0x00,{}}     | {0x00,0x00,{}}                             |
| Turn OFF LED                   | {0x11,0x01,0x00,{}}     | {0x00,0x00,{}}                             |
| Read LED status                | {0x12,0x01,0x00,{}}     | {0x00,1,{0x01}} (0x01=ON, 0x00=OFF)        |
| Read IR data                   | {0x12,0x02,0x00,{}}     | {0x00,4,{int16 presenceVal, int16 motionVal}}            |
| Set IR threshold               | {0x13,0x02,0x02,{THS_H,THS_L}}  | {0x00,0x00,{}}                     |
| Read ACC interrupt             | {0x12,0x03,0x00,{}}     | {0x00,1,{WAKE_UP_SRC mask}}                |
| Set ACC threshold              | {0x13,0x03,0x01,{THS}}  | {0x00,0x00,{}}                             |
| Read GPS data                  | {0x12,0x04,0x00,{}}     | {0x00,32,{32 raw NMEA bytes}} / {0x01,32,{padding}} if nothing queued |
| Configure GPS power/reset      | {0x13,0x04,0x02,{RSTN,EN}} | {0x00,0x00,{}}                          |
| Read battery status            | {0x12,0x05,0x00,{}}     | {0x00,7,{flags, int16 ibat, uint16 vbat, uint16 vbus}} |
| Read current time              | {0x12,0x06,0x00,{}}     | {0x00,6,{YY,MM,DD,HH,MM,SS}}              |
| Sync RTC from GPS              | {0x13,0x06,0x00,{}}     | {0x00,0x00,{}}                             |
| Read interrupt status          | {0x12,0x07,0x00,{}}     | {0x00,4,{SOURCES, IR, ACC, RTC}}            |
| Set daily alarm                | {0x13,0x08,0x03,{HH,MM,SS}} | {0x00,0x00,{}}                         |
| Cancel alarm                   | {0x11,0x08,0x00,{}}     | {0x00,0x00,{}}                             |
| Read armed alarm               | {0x12,0x08,0x00,{}}     | {0x00,4,{armed,HH,MM,SS}}                  |

Notes on individual commands:

- **Set IR threshold** takes **two** bytes, big-endian (`THS_H` first). The IR
  threshold range is 0–32767, so one byte cannot express the useful values.
- **Read IR data** returns presence and motion counts only. It carries no interrupt
  information and clears none — that belongs to `Read interrupt status`. The payload
  is 4 bytes, so read 6.
- **Read ACC interrupt** returns the accelerometer's own `WAKE_UP_SRC` bits rather
  than an acceleration value: `0x01` Z, `0x02` Y, `0x04` X, `0x08` the wake-up flag,
  `0x20` free-fall, `0x40` sleep change. They accumulate until the interrupt status
  is read; this command does **not** clear them. Raw axes are not exposed over this
  protocol (`ACC_ReadAxes()` exists in the firmware but has no command).
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
- **Read armed alarm** returns `1` in the first byte when an alarm is armed and
  `0` when it is not. The three time bytes are meaningful only in the first case
  and read zero in the second.
- **Read interrupt status** is described in full under
  [What the interrupt status register contains](#what-the-interrupt-status-register-contains).
  It is the only command that clears interrupt state, and the only one that releases
  the `MCU_INT` line.
- **Read GPS data** returns **raw NMEA bytes**, not a parsed position. It is a
  passthrough: the MCU does not decode coordinates at all. The payload is **always
  32 bytes** — packed with as many queued sentences as fit and padded with newlines
  if there were not enough, so `DATA_LEN` is always `0x20` and carries no
  information. `STATUS = 0x01` means the queue was empty and the 32 bytes are all
  padding. Always read 34 bytes.
  See [GPS NMEA Passthrough](#24-gps-nmea-passthrough) — reading this command
  correctly requires more than the table row above.
- **Configure GPS power/reset** drives `GPS_RSTN` and `GNSS_PWR_EN`.
  Use `0` or `1` per byte.
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
- **Turn ON / Turn OFF** are implemented for `SENSOR_LED` only.

### 2.3 Response Lengths:

Total bytes the master should read (`2 + DATA_LEN`):

| Command | Bytes to read | Typical latency |
|---------|---------------|-----------------|
| `0x10` / `0x11` (LED on/off) | 2 | immediate |
| `0x12,0x01` (LED status) | 3 | immediate |
| `0x12,0x02` (IR data) | 6 | two I2C1 sensor reads |
| `0x12,0x03` (ACC interrupt) | 3 | immediate |
| `0x12,0x04` (GPS data) | 34 — always | immediate — a copy out of RAM, no bus access |
| `0x12,0x05` (battery) | 9 | five I2C1 register reads |
| `0x12,0x06` (time) | 8 | immediate |
| `0x12,0x07` (interrupt status) | 6 | immediate — snapshots RAM latches, no bus access |
| `0x12,0x08` (armed alarm) | 6 | immediate |
| `0x11,0x08` (cancel alarm) | 2 | immediate |
| `0x13,*` (config) | 2 | IR/ACC re-init: several I2C1 writes; alarm: immediate |

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

Sensor: ISM330DHCX on I2C1. Wake-up detection on INT1, latched.

Interrupt code:

| Code | Meaning             |
|------|-------------------|
| 0x00 | No motion detected |
| 0x01 | Motion detected    |


Default threshold: ACC_THS_DEFAULT = 0x04

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

Interrupt code:

| Code | Meaning             |
|------|-------------------|
| 0x00 | No motion detected |
| 0x02 | Motion detected    |
| 0x04 | Presence detected    |

The firmware reports one code at a time and gives presence priority: if presence
and motion are flagged together, `0x04` is returned and the motion flag is not
reported.

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

> **The firmware default is `IR_THS_DEFAULT = 1000` (≈0.5 °C), which is outside
> the table above.** 1000 is the value the board has actually been running and it
> behaves well — a high threshold means few false triggers. The 100–400 range in
> the table comes from the sensor sensitivity figure and has not been validated on
> this hardware. Treat the table as a starting point for tuning, not as tested
> settings.
>
> The firmware never writes the hysteresis registers, so hysteresis stays at the
> sensor default of 50. Note that ST pairs 50 with a threshold of 200 — a ratio of
> 25%. Against the firmware's 1000 the ratio is 5%, which leaves only a narrow band
> (flag sets at 1000, clears at 950) and can make the flag chatter when the signal
> settles near the threshold. If 1000 is kept, a hysteresis around 250 restores
> ST's ratio.

Both `PRESENCE_THS` and `MOTION_THS` are written with the same value; the protocol
has no way to set them independently.


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
  `Read GPS data` is the exception — it copies out of RAM and touches no bus.
- **A read before any command has been sent** returns 4 bytes of zeros rather
  than an error, because the response length is initialised to 2.
- **`Turn ON` / `Turn OFF` with a sensor ID other than `SENSOR_LED`** return
  `STATUS = 0x00` without doing anything.
- **Only `Read interrupt status` clears interrupt state.** `Read IR data` no longer
  carries or clears it, and `Read ACC interrupt` reports the accelerometer's bits
  without clearing them. Earlier firmware cleared from the value reads too, so
  whichever command ran first consumed the event.
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
- **The slave hangs the bus if the master over-reads.** There is no
  "no more data" response; the slave simply stops having bytes and holds SCL.
  This is why every response has a fixed length, and why the GPS payload is padded
  instead of shortened. A master that reads `2 + max DATA_LEN` on a variable-length
  reply will hang the bus intermittently, depending on what happened to be queued.
  Hardening the slave to arm a full-length buffer for every command, so that
  over-reads are harmless, is an open item.
- **No UBX is parsed.** The firmware reads NMEA only, which means the receiver's
  `fullyResolved` / `confirmedTime` / `tAcc` indications are not available — see
  [Timekeeping](#25-timekeeping).
- **The interrupt bytes report events, not present state**, and there is no
  falling-edge handler, so the master is never told when a condition ends. See
  [Every byte answers "what fired", never "what is true now"](#every-byte-answers-what-fired-never-what-is-true-now).

  An earlier revision of this file claimed `FUNC_STATUS` on the STHS34PF80 is
  clear-on-read. It is not: `tshock`, `mot` and `pres` are level flags re-evaluated
  every ODR cycle, and only the `DRDY` bit in `STATUS` (`0x23`) is cleared by being
  read.
- **Presence and motion currently return the same value.** The STHS34PF80's filter
  bandwidths (`LPF_M`, `LPF_P`, `LPF_P_M`, `LPF_A_T`) are never configured, so they
  stay at their reset divider and the two algorithm outputs are the same signal.
  Motion therefore responds to slow changes that a motion detector should ignore.
- **The IR baseline walks after a reset**, because `sths34pf80_algo_reset()` is
  never called. Measured: presence climbed monotonically from −856 to +235 over
  twenty seconds with nothing moving in front of the sensor. With a low threshold
  that settling alone raises a presence event.
- **Expect a spurious motion event about a second after every MCU reset**, from the
  same filter transient. Both runs of the measurement above showed it.
- **There is no `HAL_I2C_ErrorCallback`.** An I2C2 error is left to the stuck-bus
  watchdog above rather than being handled where it happens.
