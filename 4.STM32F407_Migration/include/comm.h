// -----------------------------------------------------------------------------
// Copyright (c) 2024 Mu Shibo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// -----------------------------------------------------------------------------

// comm.h – UART binary communication module for STM32F407.
// Replaces wifi.h / WebSocket / ArduinoJson from the ESP32 version.
//
// The host (PC tool, mobile app, or an ESP8266 WiFi-to-UART bridge) sends
// 20-byte binary packets at 20–50 Hz over the USB Virtual COM port (Serial).
// The SimpleFOC Commander also runs on the same Serial interface.
// See README.md for the full binary packet format.

#pragma once

#include <Arduino.h>

// Call once in setup() after Serial.begin().
void Comm_Init(void);

// Process pending incoming data and update the wrobot control struct.
// Call every loop iteration (replaces web_loop() from the ESP32 version).
void comm_loop(void);
