<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: Firmware for the L3X-Z radiation sensor
=====================================================

[![General Formatting Checks](https://github.com/107-systems/l3xz-fw_radiation_sensor/workflows/General%20Formatting%20Checks/badge.svg)](https://github.com/107-systems/l3xz-fw_radiation_sensor/actions?workflow=General+Formatting+Checks)
[![Spell Check](https://github.com/107-systems/l3xz-fw_radiation_sensor/workflows/Spell%20Check/badge.svg)](https://github.com/107-systems/l3xz-fw_radiation_sensor/actions?workflow=Spell+Check)
[![Compile Examples](https://github.com/107-systems/l3xz-fw_radiation_sensor/workflows/Compile/badge.svg)](https://github.com/107-systems/l3xz-fw_radiation_sensor/actions?workflow=Compile)

Firmware for the radiation sensor board providing radiation data via OpenCyphal.

## How-to-build/upload
```bash
arduino-cli compile -b arduino:samd:nano_33_iot -v .
arduino-cli upload -b arduino:samd:nano_33_iot -v . -p /dev/ttyACM0
```

## OpenCyphal Settings

Specific seetings for the L3X-Z Hexapod can be found here: https://github.com/107-systems/l3xz-hw#node-ids

### Node-ID

Every leg controller needs to have its own Node-ID. The Node-ID is stored in the eeprom. If no eeprom is found, the Node-ID is 101.

### Subject-ID

All controllers can use the same Subject-IDs. The host can differentiate between them by their Node-IDs.

| **Subject-ID** | **direction** | **name**          | **type**    |
|:--------------:|:-------------:|:-----------------:|:-----------:|
| heartbeat      | pub           | heartbeat         | heartbeat   |
| 1001           | pub           | input-voltage     | Real32      |
| 1005           | sub           | LED1              | Bit         |
| 3000           | pub           | radiation_value   | Integer16   |

## Related Repositories
* [107-Arduino-MCP2515](https://github.com/107-systems/107-Arduino-MCP2515)
* [107-Arduino-UAVCAN](https://github.com/107-systems/107-Arduino-UAVCAN)
