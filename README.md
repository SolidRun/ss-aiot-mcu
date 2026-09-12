# SolidRun SolidSense AIOT System Controller Driver

This project implements drivers for the embedded System Controller on SolidRun SolidSense AIOT board.

## Drivers

The core `ssaiot-sc` driver owns the I2C link to the controller and splits it
into one sub-device per logical function. It is built from [`core.c`](core.c)
(I2C client probe and private data), [`transport.c`](transport.c) (command and
response framing), [`irq.c`](irq.c) (interrupt demultiplexer) and
[`mfd.c`](mfd.c) (sub-device table).

The controller speaks a command/response protocol rather than exposing a
register map, so the core provides no regmap. Sub-devices instead issue whole
commands through `ssaiot_sc_xfer()`, which frames the request, performs the
transfer and hands back the response payload along with its status byte.

The controller also multiplexes every event onto a single interrupt line, and
the read that reports a pending event clears all of them at once. Only
[`irq.c`](irq.c) may therefore sample it: it performs that read and dispatches
the result as nested per-source interrupts, so a sub-device never has to poll
for events belonging to another.

Sub-devices are plain platform drivers, registered as the following cells:

| Cell | Function | Named IRQs |
|------|----------|------------|
| `ssaiot-sc-led` | LED | - |
| `ssaiot-sc-ir` | Infrared presence and motion sensor | `activity`, `presence` |
| `ssaiot-sc-acc` | Accelerometer | `motion`, `tilt`, `freefall` |
| `ssaiot-sc-gnss` | GNSS NMEA passthrough | - |
| `ssaiot-sc-charger` | Battery charger status | - |
| `ssaiot-sc-rtc` | Real-time clock | - |

A sub-device reaches the core with `dev_get_drvdata(pdev->dev.parent)`,
addresses its own function through the matching `SSAIOT_SC_SENSOR_*` id, and
claims any interrupt it needs with `platform_get_irq_byname()`. Adding a
function means adding a cell to [`mfd.c`](mfd.c) and writing the driver; no
change to the transport is required.

At probe the core asks the controller to identify itself, so which firmware is
running can be read back from the kernel log:

```sh
dmesg | grep solids
```
```
[    7.337873] solidsense-aiot-system-controller 1-0018: controller api 0, firmware 5977093c.
[    7.361491] solidsense-aiot-system-controller 1-0018: SolidSense AIOT System Controller probed.
```

`api` is the protocol version the firmware implements, and `firmware` the
abbreviated commit it was built from, as in `git show 5977093c` in the firmware
repository. A `-dirty` suffix means the tree carried uncommitted changes at
build time, and `00000000` that the build had no git to ask.

The below drivers are already implemented:

### GNSS

The GNSS device driver supplies raw NMEA records via generic `/dev/gnss0` device-node,
for use with standard linux gps software such as [gpsd](https://gpsd.gitlab.io/gpsd/).

For basic testing use `gpsmon`, `cgps`, or `cat /dev/gnss0`.

### Charger

The charger driver can monitor external dc supply and a range of standard batteries.
The system controller reference firmware is configured for single cell type 18650 (LiCoO2).

Charger and battery status can be queried using `upower -d` command.

### Accelerometer

The accelerometer driver registers two IIO devices: `ssaiot-sc-accel` for the
three axes, and `ssaiot-sc-accel-temp` for the sensor's die temperature. Both
are buffer capable and timestamp every sample. Requires `CONFIG_IIO_BUFFER` and
`CONFIG_IIO_KFIFO_BUF`.

The sensor samples continuously at 416 Hz by default, configurable in controller
firmware.

Neither device offers a `raw` attribute. The controller serves samples from a
queue that is consumed by being read, so there is no current value to hand out
one at a time - read from the buffer instead, one sample if that is all that is
wanted:

```sh
iio_readdev -b 8 -s 64 ssaiot-sc-accel > motion.bin
iio_readdev -b 1 -s 1 ssaiot-sc-accel > sample.bin
```

Apply `scale`, and for temperature `offset` as well, to interpret the result:

```sh
iio_attr -c ssaiot-sc-accel accel_z scale
iio_attr -c ssaiot-sc-accel-temp temp offset
```

Temperature arrives far more slowly than motion, slowly enough that the libiio
tools time out waiting for it. Ask for very little at a time, and expect to wait:

```sh
iio_readdev -b 1 -s 2 ssaiot-sc-accel-temp > temp.bin
```

The die temperature is not guaranteed accurate, use only as indicator.

#### Events

The controller runs its own motion, tilt and free-fall detectors and reports each
as an IIO event on `ssaiot-sc-accel`. They sit on event-only channels, so they
never appear in the buffer:

| Detector | Enable attribute |
|----------|------------------|
| Motion | <code>n_accel_x\|y\|z_mag_adaptive_rising_en</code> |
| Tilt | `in_incli_change_either_en` |
| Free-fall | `in_accel_x&y&z_mag_falling_en` |

Enabling an attribute only gates delivery to userspace. The detectors are
configured by controller firmware and run either way.

##### Detection, with no tooling

The interrupt counters advance on every detection whether or not userspace
enabled the event, so this alone shows the controller firing and the
demultiplexer routing each detector to its own source:

```sh
grep -E 'motion|tilt|freefall' /proc/interrupts
```

Move, tilt or drop the board, run it again, and the matching counter has
stepped.

##### Detection with libIIO

The last released version of [libIIO](analogdevicesinc.github.io/libiio/main/) (v0.26) does not support iio event channels.
Once v1.0 will be released, this section shall be updated.

### Infrared

The infrared driver registers one IIO device, `ssaiot-sc-ir`, carrying the
STHS34PF80's two detector signals and its ambient temperature. It is buffer
capable and timestamps every sample. Requires `CONFIG_IIO_BUFFER` and
`CONFIG_IIO_KFIFO_BUF`.

The sensor samples continuously at 30 Hz by default, configurable in controller
firmware.

The two detector channels are told apart by their labels:

| Channel | Label | Reading |
|---------|-------|---------|
| `proximity0` | `presence` | stationary body in the field of view |
| `proximity1` | `motion` | movement in the field of view |
| `temp` | `ambient` | the sensor's own package temperature |

```sh
iio_attr -c ssaiot-sc-ir proximity0 label
```

There is no `raw` attribute. The controller serves readings from a queue that is
consumed by being read, so there is no current value to hand out one at a time.
All three arrive together in one record, so they always scan together and the
sample layout is fixed:

```sh
iio_readdev -b 1 -s 4 ssaiot-sc-ir > ir.bin
```

At a low output data rate the libiio tools give up before the buffer fills.
Reading the character device directly avoids their timeout:

```sh
cd /sys/bus/iio/devices/iio:device5
echo 1 > scan_elements/in_proximity0_en
echo 1 > scan_elements/in_proximity1_en
echo 1 > scan_elements/in_temp_en
echo 1 > scan_elements/in_timestamp_en
echo 16 > buffer/length
echo 1 > buffer/enable
dd if=/dev/iio:device5 bs=16 count=4 2>/dev/null | hexdump -C
echo 0 > buffer/enable
```

Find the device number first, it is not stable across boots:

```sh
grep -H . /sys/bus/iio/devices/iio:device*/name
```

Each sample is 16 bytes:

| Byte | Field | Encoding |
|-------|-------|----------|
| 0-1 | presence | int16, unitless |
| 2-3 | motion | int16, unitless |
| 4-5 | ambient | int16, apply `scale` for millidegrees |
| 6-7 | padding | |
| 8-15 | timestamp | int64, nanoseconds on `current_timestamp_clock` |

Presence and motion carry no scale. They are differences between two of the
sensor's internal low-pass filters, so they mean something only against the
detector thresholds, which are set in controller firmware. Ambient is 100 LSB
per degree, reported as a `scale` of 10:

```sh
iio_attr -c ssaiot-sc-ir temp scale
```

The package temperature is not guaranteed accurate, use only as indicator.

#### Events

The controller runs the sensor's own presence and motion detectors and reports
each as an IIO event. A detector compares one of the buffered signals against a
threshold, so its events belong to that same channel:

| Detector | Channel | Enable attribute |
|----------|---------|------------------|
| Presence | `proximity0` | `in_proximity0_thresh_rising_en` |
| Motion | `proximity1` | `in_proximity1_thresh_rising_en` |

Enabling an attribute only gates delivery to userspace. The detectors run either
way, and their thresholds are set in controller firmware - there is no command
to change them, so no `_value` attribute is offered.

##### Detection, with no tooling

The interrupt counters advance on every detection whether or not userspace
enabled the event:

```sh
grep -E 'presence|activity' /proc/interrupts
```

The motion detector's interrupt is named `activity`, not `motion`.

Walk into the field of view, run it again, and the matching counter has stepped.

##### Detection with libIIO

The last released version of [libIIO](analogdevicesinc.github.io/libiio/main/) (v0.26) does not support iio event channels.
Once v1.0 will be released, this section shall be updated.

### RTC

The RTC supports read-only time based on GNSS, and wake on alarm.

Read the current time using `hwclock -f /dev/rtc0 -r`, this succeeds as soon as the system controller has synced time with GNSS at least once.

For wake on alarm, use ` rtcwake` command.
For example to shutdown and restart after 5 minutes: `rtcwake -m off -s 300`

## Device-Tree Binding

System Controller must be described as a sub-node below an i2c bus exactly as shown below, all properties are mandatory:

```c
sc: system-controller@18 {
	compatible = "solidrun,solidsense-aiot-system-controller";
	reg = <0x18>;
	interrupts-extended = <&pinctrl RZG2L_GPIO(5, 6) IRQ_TYPE_LEVEL_LOW>;
};
```

Note IRQ pin must enable integrated pullup in pinconfig!

Optional properties:

- `monitored-battery` phandle to support capacity reporting from an ocv table.
  There is no temperature sensor, use fixed 20°C.

  Generic example for single cell type 18650 3.7V nominal and 2500mAh (LiCoO2):
  ```c
  / {
  	bat: battery {
  		compatible = "simple-battery";
  		factory-internal-resistance-micro-ohms = <100000>;
  		ocv-capacity-celsius = <20>;
  		ocv-capacity-table-0 = <4200000 100>, <4150000 95>, <4110000 90>,
  				       <4080000  85>, <4020000 80>, <3980000 75>,
  				       <3950000  70>, <3910000 65>, <3870000 60>,
  				       <3850000  55>, <3840000 50>, <3820000 45>,
  				       <3800000  40>, <3790000 35>, <3770000 30>,
  				       <3750000  25>, <3730000 20>, <3710000 15>,
  				       <3690000  10>, <3610000  5>, <3000000  0>;
  	};
  };

  &sc {
  	monitored-battery = <&bat>;
  };
  ```

  All battery properties shown in example must be specified to enable capacity reporting.

- `system-power-controller` marks the controller as the board's power-off method, cutting power on shutdown.

  There is no power button, only the controller can re-enable power
  based on sensor configuration, rtc or on controller reset.
