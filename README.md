# Fujitsu AirStage-H component for ESPHome

An ESPHome component to control Fujitsu AirStage-H (product line previously known as Halcyon) units via the three wire (RWB) bus.

## Basic configuration

```yaml
substitutions:
  device_name: halcyon-controller
  friendly_name: Halcyon Controller
  device_description: Atom Lite + FOSV
  esp_board: m5stack-atom

external_components:
#  - source: github://Omniflux/esphome-tzsp
  - source: github://Omniflux/esphome-fujitsu-halcyon

packages:
  wifi: !include common/wifi.yaml

esphome:
  name: ${device_name}
  friendly_name: ${friendly_name}
  comment: ${device_description}

esp32:
  board: ${esp_board}
  framework:
    type: esp-idf

api:

ota:
  - platform: esphome
    password: !secret ota_password

#logger:
#  level: DEBUG

button:
  - platform: restart
    name: Restart
  - platform: safe_mode
    name: Restart (Safe Mode)

sensor:
- platform: uptime
  name: "Uptime"

uart:
  tx_pin: GPIO22  # Device dependent
  rx_pin: GPIO19  # Device dependent
  baud_rate: 500
  parity: EVEN

climate:
- platform: fujitsu-halcyon
  name: None  # Use device friendly_name

  # Fujitsu devices use 0 and 1, but 2-15 should also work. Must not skip addresses
  controller_address: 1  # 0=Primary, 1=Secondary

  #temperature_controller_address: 0  # Fujitsu controller address to read temperature from
  #temperature_sensor_id: my_temperature_sensor  # ESPHome sensor to read temperature from
  #humidity_sensor: my_humidity_sensor  # ESPHome sensor to read humidity from
  #ignore_lock: true  # Ignore child/part/feature lock set on unit or primary/central remote control

  # To capture communications for debugging / analysis
  # Use Wireshark with https://github.com/Omniflux/fujitsu-airstage-h-dissector
  #tzsp:
  #  ip: 192.168.1.20
  #  protocol: 255
```

## External temperature and humidity sensors

You can use ESPHome (or Home Assistant) sensors to report the current temperature and humidity to the Home Assistant climate component.

```yaml
sensor:
  - platform: homeassistant # https://esphome.io/components/sensor/homeassistant.html
    id: my_temperature_sensor
    entity_id: sensor.my_temperature_sensor  # Home Assistant entity_id
    unit_of_measurement: "°F"  # unit_of_measurement is lost on import, defaults to °C
  - platform: homeassistant
    id: my_humidity_sensor
    entity_id: sensor.my_humidity_sensor

climate:
  - platform: fujitsu-halcyon
    name: None
    controller_address: 1
    temperature_sensor_id: my_temperature_sensor
    humidity_sensor: my_humidity_sensor
```

If your unit supports sensor switching and has had the function settings set appropriately (see your installation manual, usually settings `42` and `48`), your unit can also be set to use this sensor instead of the sensor in its air intake. Enable the `Use Sensor` switch in the Home Assistant device page to use this feature.

## Exposed entities

The component exposes the following entities. Some feature-dependent entities are **disabled by default** in Home Assistant — check the **Supported Features** diagnostic sensor to see which features your unit reports, then enable the relevant entities in the HA device page.

| Entity | Type | Category | Default | Description |
|---|---|---|---|---|
| Climate | Climate | — | Enabled | Main climate control (mode, temperature, fan, swing, preset) |
| Standby Mode | Binary sensor | Diagnostic | Enabled | Indicates defrosting, oil recovery, or waiting for other units |
| Error | Binary sensor | Diagnostic | Enabled | Error state |
| Error Code | Text sensor | Diagnostic | Enabled | Error code (hex format: `address error_code`, e.g. `00 A3`) |
| Connected | Binary sensor | Diagnostic | Enabled | UART initialization with the indoor unit completed |
| Supported Features | Text sensor | Diagnostic | Enabled | Comma-separated list of features reported by the indoor unit |
| Initialization Stage | Text sensor | Diagnostic | Enabled | Initialization progress (e.g. `(4/4)` when complete) |
| Reinitialize | Button | Config | Enabled | Restart the UART initialization sequence |
| Remote Temperature Sensor | Sensor | Diagnostic | Disabled | Temperature reported by another controller on the bus |
| Use Sensor | Switch | Config | Disabled | Use the controller's temperature sensor instead of the unit's intake sensor |
| Advance Vertical Louver | Button | — | Disabled | Advance the vertical louver position |
| Advance Horizontal Louver | Button | — | Disabled | Advance the horizontal louver position |
| Filter Timer Expired | Binary sensor | Diagnostic | Disabled | Filter timer has expired |
| Reset Filter Timer | Button | Config | Disabled | Reset the filter timer |
| Function / Function Value / Function Unit | Number | Config | Enabled | Raw function read/write interface for advanced configuration |
| Function_Read | Button | Config | Enabled | Execute raw function read |
| Function_Write | Button | Config | Disabled | Execute raw function write |

Disabled entities are registered in Home Assistant but hidden until manually enabled. This avoids cluttering the device page with features your unit may not support. To enable an entity, go to the device page in Home Assistant, click on the entity, and toggle it on.

## Debugging

Configure TZSP and use Wireshark with [fujitsu-airstage-h-dissector](https://github.com/Omniflux/fujitsu-airstage-h-dissector) to debug / decode the Fujitsu serial protocol.

## Related projects

- FOSV's [Fuji-Atom-Interface](https://github.com/FOSV/Fuji-Atom-Interface) - Open hardware interface compatible with this component

<!-- -->

- My [esphome-fujitsu-dmmum](https://github.com/Omniflux/esphome-fujitsu-dmmum) - Fujitsu AirStage-H 3-wire Central Controller component for ESPHome

<!-- -->

- Aaron Zhang's [esphome-fujitsu](https://github.com/FujiHeatPump/esphome-fujitsu)
- Jaroslaw Przybylowicz's [fuji-iot](https://github.com/jaroslawprzybylowicz/fuji-iot)
- Raal Goff's [FujiHeatPump](https://github.com/unreality/FujiHeatPump)
- Raal Goff's [FujiHK](https://github.com/unreality/FujiHK)

<!-- -->

- Myles Eftos's [Reverse engineering](https://hackaday.io/project/19473-reverse-engineering-a-fujitsu-air-conditioner-unit)
