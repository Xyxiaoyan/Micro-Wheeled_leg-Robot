# STM32F407 Migration Guide

This directory contains a ported version of the Micro-Wheeled-Leg Robot firmware
targeting the **STM32F407** microcontroller (e.g. STM32F407VGT6 on the Discovery
board or a custom PCB).

---

## Migration overview

| Item | ESP32 (original) | STM32F407 (ported) |
|---|---|---|
| **Build system** | Arduino IDE + esp32 board package | Arduino IDE + STM32duino board package |
| **Motor control** | SimpleFOC v2 | SimpleFOC v2 (STM32 supported natively) |
| **IMU** | MPU6050_tockn (I2C) | MPU6050_tockn (I2C, unchanged) |
| **Encoders** | AS5600 via I2C | AS5600 via I2C (unchanged) |
| **Servo UART** | Serial2 @ 1 Mbaud | HardwareSerial (UART4, PC10/PC11) @ 1 Mbaud |
| **WiFi control** | Built-in WiFi AP + WebSocket/JSON | **Not available** – replaced with UART binary protocol |
| **ADC (battery)** | `esp_adc_cal` (ESP32-specific) | Standard `analogRead()` |
| **Commander** | SimpleFOC Commander over USB Serial | SimpleFOC Commander over USB Virtual COM (same) |

---

## Hardware – STM32F407 pin mapping

```
Peripheral          STM32F407 pins          Notes
─────────────────── ─────────────────────── ─────────────────────────────────
Motor 1 PWM         PA8 / PA9 / PA10        TIM1 CH1/CH2/CH3
Motor 1 Enable      PA11                    GPIO output
Motor 2 PWM         PC6 / PC7 / PC8         TIM8 CH1/CH2/CH3
Motor 2 Enable      PC9                     GPIO output
Encoder 1 (I2C1)    PB8 (SCL)  PB9 (SDA)   AS5600 #1
Encoder 2 (I2C2)    PB10 (SCL) PB11 (SDA)  AS5600 #2
MPU6050 (I2C2)      PB10 (SCL) PB11 (SDA)  Shared with Encoder 2
Servo UART (UART4)  PC10 (TX)  PC11 (RX)   STS3032 @ 1 Mbaud
Debug / Commander   USB Virtual COM (CDC)   115200 baud (mapped to Serial)
Battery ADC         PA0                     ADC1_IN0, 3.3 V reference
LED (battery)       PD12                    GPIO output (Discovery green LED)
```

> **Note:** PA9/PA10 are TIM1 CH2/CH3. Do **not** assign USART1 to these pins at
> the same time.  USART3 (PB10/PB11) must not be enabled because those pins are
> used for I2C2.

---

## Software dependencies

Install the following libraries in the Arduino IDE (Library Manager) **before**
compiling the ported firmware:

| Library | Version tested | Notes |
|---|---|---|
| **STM32duino** board package | ≥ 2.6.0 | Boards Manager → "STM32 MCU based boards" |
| **SimpleFOC** | 2.3.x | Motor / sensor / PID / LPF / Commander |
| **MPU6050_tockn** | 1.x | IMU driver (unchanged) |

The following ESP32-only libraries are **removed** and do not need to be installed:

- `WiFi`, `WebServer`, `WebSocketsServer`
- `ArduinoJson`
- `esp_adc_cal`

---

## WiFi / remote-control replacement

Because the STM32F407 does not have built-in WiFi, the original WiFi AP +
WebSocket JSON protocol is replaced with a **20-byte binary UART protocol** sent
over the USB Virtual COM port (or any UART).

### Packet format (host → robot, 20 bytes)

```
Offset  Field          Type    Notes
──────  ─────────────  ──────  ─────────────────────────────────────
 0      Header 0       uint8   0xAA
 1      Header 1       uint8   0x55
 2      Mode           uint8   0 = BASIC
 3      Direction      uint8   0=FORWARD 1=BACK 2=RIGHT 3=LEFT 4=STOP 5=JUMP
 4      Height         uint8   0–100 (default 38)
 5      Roll sign      uint8   0 = positive, 1 = negative
 6      Roll magnitude uint8   absolute value
 7      Linear sign    uint8   0 = positive, 1 = negative
 8      Linear mag     uint8   absolute value
 9      Angular sign   uint8   0 = positive, 1 = negative
10      Angular mag    uint8   absolute value
11      Stable (go)    uint8   0 = off, 1 = on (enable balancing)
12      Joy X sign     uint8   0 = positive, 1 = negative
13      Joy X mag      uint8   absolute value
14      Joy Y sign     uint8   0 = positive, 1 = negative
15      Joy Y mag      uint8   absolute value
16–19   Reserved       uint8   set to 0
```

A host application (PC script, mobile app, or an external ESP-01/ESP8266 module
acting as a WiFi-to-UART bridge) sends this packet at whatever rate is needed
(typically 20–50 Hz).

The SimpleFOC **Commander** interface continues to work in parallel on the same
Serial (USB CDC), allowing real-time PID tuning exactly as before.

### Optional: add an ESP8266 WiFi bridge

To restore WiFi control without redesigning the PCB:

1. Connect an ESP-01S module to the STM32's spare UART.
2. Flash the ESP-01S with a simple AT-to-binary bridge firmware that receives the
   existing JSON WebSocket messages from the web UI and converts them to the
   20-byte binary format before forwarding to the STM32 over UART.

---

## How to build

1. Install the STM32duino board package:  
   `File → Preferences → Additional Boards Manager URLs`  
   → add `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`

2. `Tools → Board → STM32 MCU based boards → Generic STM32F4 series`

3. `Tools → Board part number → Generic F407VGTx` (or your exact variant)

4. `Tools → USB support → CDC (generic Serial supersede U(S)ART)`  
   This maps `Serial` to USB Virtual COM so the Commander works over USB.

5. `Tools → Upload method → STM32CubeProgrammer (DFU)` (or SWD/JTAG)

6. Install all required libraries (see table above).

7. Open `firmware/wl_robot_stm32/wl_robot_stm32.ino` and click **Upload**.

---

## Key code differences from the ESP32 version

### Pin definitions
```cpp
// ESP32 (original)
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 12);
I2Cone.begin(19, 18, 400000UL);
I2Ctwo.begin(23,  5, 400000UL);

// STM32F407 (ported)
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA8, PA9, PA10, PA11);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PC6, PC7, PC8,  PC9);
I2Cone.begin();  // pins set in TwoWire constructor
I2Ctwo.begin();
```

### I2C instantiation
```cpp
// ESP32 (original) – pins passed to begin()
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
I2Cone.begin(19, 18, 400000UL);
I2Ctwo.begin(23,  5, 400000UL);

// STM32F407 (ported) – pins passed to constructor
TwoWire I2Cone(PB9, PB8);   // SDA=PB9, SCL=PB8  (I2C1)
TwoWire I2Ctwo(PB11, PB10); // SDA=PB11, SCL=PB10 (I2C2)
I2Cone.setClock(400000);
I2Ctwo.setClock(400000);
I2Cone.begin();
I2Ctwo.begin();
```

### Servo UART
```cpp
// ESP32 (original)
Serial2.begin(1000000);
sms_sts.pSerial = &Serial2;

// STM32F407 (ported)
HardwareSerial ServoSerial(PC11, PC10); // RX=PC11, TX=PC10 (UART4)
ServoSerial.begin(1000000);
sms_sts.pSerial = &ServoSerial;
```

### Battery ADC
```cpp
// ESP32 (original) – uses esp_adc_cal
uint32_t raw = analogRead(BAT_PIN);
uint32_t voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
double battery = (voltage * 3.97) / 1000.0;

// STM32F407 (ported) – standard analogRead, 12-bit, 3.3 V reference
// External voltage divider ratio must match the PCB (original used ~3.97×)
float raw = analogRead(BAT_PIN);
float battery = raw * (3.3f / 4095.0f) * VBAT_DIVIDER_RATIO;
```

### Communication (control protocol)
```cpp
// ESP32 (original)
void web_loop() {
    webserver.handleClient();
    websocket.loop();
    rp.spinOnce();
}

// STM32F407 (ported)
void comm_loop() {
    rp.spinOnce();  // reads binary packet from Serial (USB CDC)
}
```

---

## Known limitations / future work

- **No web UI:** The HTML/CSS/JS web interface (`basic_web.h`) is not used. A
  separate host-side application must implement the binary control protocol.  
  Alternatively, an ESP8266 bridge can restore the original web interface.
- **ADC voltage divider ratio:** The `VBAT_DIVIDER_RATIO` constant in
  `wl_robot_stm32.ino` must be calibrated for the actual resistor values on the
  new PCB.
- **SimpleFOC timer allocation:** SimpleFOC automatically selects STM32 timers
  for PWM.  If you change the PWM pins, verify that TIM1 and TIM8 are available
  and not claimed by other peripherals.
