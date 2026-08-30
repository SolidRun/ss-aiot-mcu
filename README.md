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
