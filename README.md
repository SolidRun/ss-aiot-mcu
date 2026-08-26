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
| `ssaiot-sc-ir` | Infrared presence and motion sensor | `motion`, `presence` |
| `ssaiot-sc-acc` | Accelerometer wake-up | `motion` |
| `ssaiot-sc-gnss` | GNSS NMEA passthrough | - |
| `ssaiot-sc-charger` | Battery charger status | - |
| `ssaiot-sc-rtc` | Real-time clock | - |

A sub-device reaches the core with `dev_get_drvdata(pdev->dev.parent)`,
addresses its own function through the matching `SSAIOT_SC_SENSOR_*` id, and
claims any interrupt it needs with `platform_get_irq_byname()`. Adding a
function means adding a cell to [`mfd.c`](mfd.c) and writing the driver; no
change to the transport is required.

The below drivers are already implemented:

TBD.

## Device-Tree Binding

System Controller must be described as a sub-node below an i2c bus exactly as shown below, all properties are mandatory:

```c
system-controller@18 {
	compatible = "solidrun,solidsense-aiot-system-controller";
	reg = <0x18>;
	interrupts-extended = <&pinctrl RZG2L_GPIO(5, 6) IRQ_TYPE_LEVEL_LOW>;
};

Note IRQ pin must enable integrated pullup in pinconfig!
