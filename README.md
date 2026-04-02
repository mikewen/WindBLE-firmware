# WindBLE AC6328 Firmware

Firmware for **AC6328** MCU to read analog wind sensor signals (ultrasonic type), perform calibration and filtering, and transmit wind data over BLE.

---

## Overview

This firmware:

* Reads **wind speed** and **wind direction** from ADC inputs (0–5V sensors)
* Applies **voltage scaling**, **offset calibration**, and **filtering**
* Outputs **Apparent Wind Speed (AWS)** and **Apparent Wind Angle (AWA)**
* Sends data via **BLE notification** (Service `ae30`, Characteristic `ae02`)

Designed to work with:

* Android app: *WindBLE*
* Ultrasonic wind sensors with analog outputs

---

## Hardware Assumptions

### Sensor Inputs

* Wind Speed: 0–5V → 0–60 m/s
* Wind Direction: 0–5V → 0–360°

### Signal Conditioning

* Voltage divider: **5V → 2.5V**
* ADC input range: **0–3.3V**
* ADC resolution: **10-bit (0–1023)**

---

## Data Processing Pipeline

```
ADC → Voltage Scaling → Physical Units → Offset Calibration → Filtering → BLE
```

### 1. Voltage Conversion

```
V_in = ADC / 1023 * 3.3 * (5 / 2.5)
```

### 2. Wind Speed

```
speed (m/s) = V_in * 12
```

### 3. Wind Direction

```
angle (deg) = V_in * 72
```

---

## Calibration

### Wind Speed Offset (Zero Wind)

1. Ensure **no wind condition**
2. Average multiple ADC samples
3. Compute offset:

```
speed_offset = measured_speed
```

Runtime:

```
speed = raw_speed - speed_offset
if (speed < 0) speed = 0
```

---

### Wind Direction Offset (Alignment)

1. Point boat **into the wind**
2. Record measured angle:

```
dir_offset = measured_angle
```

Runtime:

```
angle = raw_angle - dir_offset
wrap to 0–360°
```

---

## Filtering

### Speed Filtering

* First-order low-pass filter:

```
y += α * (x - y)
```

### Direction Filtering (Circular)

To avoid wrap issues at 0/360°:

```
x = cos(angle)
y = sin(angle)

x_f = LPF(x)
y_f = LPF(y)

angle = atan2(y_f, x_f)
```

---

## BLE Data Format

### Characteristic

* Service UUID: `ae30`
* Characteristic UUID: `ae02`
* Property: **NOTIFY**

---

### Packet Structure (7 bytes)

| Byte | Description            |
| ---- | ---------------------- |
| 0    | Frame ID `'W'` (0x57)  |
| 1    | Flags                  |
| 2-3  | AWS (uint16, m/s ×100) |
| 4-5  | AWA (uint16, deg ×100) |
| 6    | Checksum (XOR)         |

---

### Example

```
AWS = 5.23 m/s → 523
AWA = 123.45° → 12345

[57][01][02][0B][30][39][CS]
```

---

### Checksum

```
cs = buf[0] ^ buf[1] ^ ... ^ buf[5]
```

---

## Units

| Parameter | Unit | Encoding      |
| --------- | ---- | ------------- |
| AWS       | m/s  | ×100 (uint16) |
| AWA       | deg  | ×100 (uint16) |

---

## Update Rate

Recommended:

* **10 Hz BLE notify rate**

Reason:

* Smooth UI response
* Low noise
* Efficient BLE usage

---

## Flags Byte

| Bit | Meaning      |
| --- | ------------ |
| 0   | Data valid   |
| 1   | Calibrated   |
| 2   | Sensor error |
| 3–7 | Reserved     |

---

## File Structure (Suggested)

```
/src
  adc.c
  wind.c
  filter.c
  ble.c
  main.c
```

---

## Key Functions

### Wind Computation

* `compute_wind()`
* Converts ADC → physical → filtered values

### Packet Builder

* `build_packet()`
* Encodes data into BLE format

### BLE Notify

* `ble_notify()`
* Sends packet over ae02 characteristic

---

## Design Rationale

### Why compute on MCU?

* Reduces BLE bandwidth
* Ensures deterministic output
* Simplifies Android app
* Keeps calibration close to hardware

---

### Why circular filtering?

* Prevents discontinuity at 0°/360°
* Essential for stable wind direction display

---

## Future Improvements

* Temperature compensation (ultrasonic sensors)
* Gust detection (high-pass filter)
* NMEA 0183 output (MWV)
* BLE configuration characteristic (set offsets remotely)
* Auto-calibration routines

---

