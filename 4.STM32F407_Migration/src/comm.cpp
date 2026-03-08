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

// comm.cpp – UART binary communication module for STM32F407.
// Replaces wifi.cpp / WebSocketsServer / WebServer from the ESP32 version.
//
// The control interface is the USB Virtual COM port (Serial, mapped to CDC by
// -DUSBD_USE_CDC in platformio.ini).  The same Serial is used by the SimpleFOC
// Commander for real-time PID tuning, so both can coexist because:
//   • Binary control packets start with the magic bytes 0xAA 0x55.
//   • Commander messages are plain ASCII starting with an uppercase letter.
//   • spinOnce() only consumes bytes that begin with 0xAA; everything else is
//     left in the buffer for the Commander to process.

#include "comm.h"
#include "robot.h"

extern RobotProtocol rp;

void Comm_Init(void)
{
    // Limit readBytes() blocking to 10 ms in case a partial packet arrives.
    // The main loop must not stall; incomplete packets are silently discarded.
    Serial.setTimeout(10);
}

// comm_loop – replaces web_loop() from the ESP32 version.
// Must be called every main loop iteration.
void comm_loop(void)
{
    rp.spinOnce();
}
