# MQTT Endpoints and Payloads

## Broker
- Host: `broker.emqx.io`
- Port: `1883`
- Username: `emqx`
- Password: `public`

## Topics
- Telemetry: `hardandsoft/esp32/data`
- Raw telemetry: `hardandsoft/esp32/gpio_raw`
- Module state: `hardandsoft/esp32/state`
- Commands to device: `hardandsoft/esp32/cmd`

Publish cadence: about once per second for telemetry topics.

---

## 1) Telemetry Topic
Topic: `hardandsoft/esp32/data`

Example payload:

```json
{
  "ts": "2026-03-31 21:10:05",
  "up": 1234,

  "temp": 22.4,
  "temperature": 22.4,
  "ntc_c": 22.4,
  "temp_raw": 1890,
  "ntc_raw": 1890,

  "gx": 0.0,
  "gy": -0.1,
  "gz": 0.0,
  "gabs": 0.1,
  "ax": 0.012,
  "ay": -0.004,
  "az": 0.998,
  "roll": -0.23,
  "pitch": 1.18,
  "yaw": 274.66,
  "heading": 271.91,
  "mx": 18.4,
  "my": -6.2,
  "mz": 31.7,
  "mag_present": true,
  "imu_addr": 105,

  "rssi": -57,
  "cpu": 18,

  "v": 4.01,
  "ma": 92.3,
  "mw": 370,
  "mah": 145.26,
  "batt": 91.0,
  "batt_min": 1890,

  "ssid": "your-wifi",
  "ip": "192.168.1.44",
  "mac": "34:85:18:AA:BB:CC",
  "channel": 6
}
```

Notes:
- `temp`, `temperature`, and `ntc_c` are aliases (same value).
- `imu_addr` is decimal (`0x69` is `105`).
- `roll`, `pitch`, and `yaw` come from gyro+accelerometer Kalman fusion.
- `heading` uses magnetometer correction when available.
- `mx`, `my`, `mz` are magnetometer values (null when no magnetometer is present).
- `mag_present` is `true` only when AK8963 magnetometer is detected (MPU9250 path).
- If a module is disabled, related values can be `null`.

---

## 2) Raw Telemetry Topic
Topic: `hardandsoft/esp32/gpio_raw`

Example payload:

```json
{
  "ts": "2026-03-31 21:10:05",
  "up": 1234,
  "gpio": {
    "gpio0": 1,
    "gpio1": 0,
    "gpio2": 1,
    "gpio3": 1,
    "gpio4": 1,
    "gpio5": 0,
    "gpio6": 0,
    "gpio7": 0,
    "gpio8": 1,
    "gpio9": 1,
    "gpio10": 0,
    "gpio20": 0,
    "gpio21": 0
  },
  "therm_raw": 1890,
  "gyro_raw": {
    "x": 12,
    "y": -8,
    "z": 3,
    "addr": 105
  },
  "accel_raw": {
    "x": 210,
    "y": -65,
    "z": 16302
  },
  "orientation": {
    "roll": -0.23,
    "pitch": 1.18,
    "yaw": 274.66,
    "heading": 271.91
  },
  "mag_raw": {
    "x": 121,
    "y": -40,
    "z": 209,
    "present": true
  },
  "ina219_raw": {
    "bus": 32256,
    "current": 2350,
    "power": 740
  }
}
```

Notes:
- If `gyro` or `current` module is disabled, related nested values may become `null`.
- `orientation` mirrors fused values used for 3D object control.
- `mag_raw.present` indicates if the magnetometer exists on your IMU module.

---

## 3) Module State Topic
Topic: `hardandsoft/esp32/state`

Example payload:

```json
{
  "modules": {
    "temperature": true,
    "gyro": true,
    "cpu": true,
    "current": true,
    "cpu_stress": false
  }
}
```

---

## 4) Online Status Event
Topic: `hardandsoft/esp32/data`

Sent on MQTT reconnect:

```json
{"status":"online"}
```

---

## 5) Command Topic
Topic: `hardandsoft/esp32/cmd`

### Enable CPU stress
```json
{"module":"cpu_stress","enabled":true}
```

### Disable CPU stress
```json
{"module":"cpu_stress","enabled":false}
```

Supported `module` values:
- `temperature`
- `gyro`
- `cpu`
- `current`
- `cpu_stress`

Behavior notes:
- If `enabled` is missing, firmware treats it as `true`.
- Enabling `cpu_stress` also enables CPU monitoring.

---

## Useful CLI Commands

Subscribe all ESP topics:

```bash
mosquitto_sub -h broker.emqx.io -p 1883 -u emqx -P public -t hardandsoft/esp32/# -v
```

Send command:

```bash
mosquitto_pub -h broker.emqx.io -p 1883 -u emqx -P public -t hardandsoft/esp32/cmd -m '{"module":"cpu_stress","enabled":true}'
```
