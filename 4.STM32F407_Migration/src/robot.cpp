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

// STM32F407 port: replaces JSON/WebSocket parsing with a 20-byte binary UART
// protocol.  The packet layout intentionally mirrors the original _now_buf so
// the rest of the application (wrobot struct, direction logic, etc.) remains
// unchanged.
//
// Binary packet format (ROBOT_PACKET_LEN bytes, host → robot over Serial):
//   [0]  0xAA  header byte 0
//   [1]  0x55  header byte 1
//   [2]  mode  (0 = BASIC)
//   [3]  dir   (0=FORWARD 1=BACK 2=RIGHT 3=LEFT 4=STOP 5=JUMP)
//   [4]  height (0–100)
//   [5]  roll sign      (0 = positive, 1 = negative)
//   [6]  roll magnitude
//   [7]  linear sign
//   [8]  linear magnitude
//   [9]  angular sign
//  [10]  angular magnitude
//  [11]  stable / go    (0 = off, 1 = balancing enabled)
//  [12]  joy_x sign
//  [13]  joy_x magnitude
//  [14]  joy_y sign
//  [15]  joy_y magnitude
//  [16–19]  reserved (0)

#include "robot.h"

Wrobot wrobot;

// ─────────────────────────────────────────────────────────────────────────────
RobotProtocol::RobotProtocol(uint8_t len)
{
    _len     = len;
    _now_buf = new uint8_t[_len];
    _old_buf = new uint8_t[_len];

    for (int i = 0; i < _len; i++) {
        _now_buf[i] = 0;
        _old_buf[i] = 0;
    }
    _now_buf[0] = 0xAA;
    _now_buf[1] = 0x55;
}

RobotProtocol::~RobotProtocol()
{
    delete[] _now_buf;
    delete[] _old_buf;
}

// ─────────────────────────────────────────────────────────────────────────────
// spinOnce – call every loop iteration.
// Looks for the 0xAA 0x55 header in the Serial receive buffer and, once a full
// ROBOT_PACKET_LEN-byte packet is available, decodes it into wrobot.
// ─────────────────────────────────────────────────────────────────────────────
void RobotProtocol::spinOnce(void)
{
    while (Serial.available() >= (int)_len) {
        uint8_t b = (uint8_t)Serial.read();
        if (b != 0xAA) continue;
        if (Serial.peek() != 0x55) continue;
        Serial.read();  // consume 0x55

        uint8_t tmp[ROBOT_PACKET_LEN];
        tmp[0] = 0xAA;
        tmp[1] = 0x55;
        int got = Serial.readBytes((char *)(tmp + 2), _len - 2);
        if (got != (int)(_len - 2)) return;  // incomplete packet – discard

        for (int i = 0; i < _len; i++) _now_buf[i] = tmp[i];
        parsePacket();
        checkBufRefresh();
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// parsePacket – decode _now_buf into the wrobot struct.
// ─────────────────────────────────────────────────────────────────────────────
void RobotProtocol::parsePacket(void)
{
    uint8_t dir_raw = _now_buf[3];
    switch (dir_raw) {
        case FORWARD: wrobot.dir = FORWARD; break;
        case BACK:    wrobot.dir = BACK;    break;
        case RIGHT:   wrobot.dir = RIGHT;   break;
        case LEFT:    wrobot.dir = LEFT;    break;
        case JUMP:    wrobot.dir = JUMP;    break;
        default:      wrobot.dir = STOP;    break;
    }

    wrobot.height  = _now_buf[4];

    wrobot.roll    = (_now_buf[5]  == 0 ? 1 : -1) * (int)_now_buf[6];
    wrobot.linear  = (_now_buf[7]  == 0 ? 1 : -1) * (int)_now_buf[8];
    wrobot.angular = (_now_buf[9]  == 0 ? 1 : -1) * (int)_now_buf[10];
    wrobot.go      = (_now_buf[11] != 0);
    wrobot.joyx    = (_now_buf[12] == 0 ? 1 : -1) * (int)_now_buf[13];
    wrobot.joyy    = (_now_buf[14] == 0 ? 1 : -1) * (int)_now_buf[15];
}

// ─────────────────────────────────────────────────────────────────────────────
// UART_WriteBuf – echo _now_buf back over Serial (optional debug use).
// ─────────────────────────────────────────────────────────────────────────────
void RobotProtocol::UART_WriteBuf(void)
{
    for (int i = 0; i < _len; i++) {
        Serial.write(_now_buf[i]);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// checkBufRefresh – returns 1 if the buffer changed since the last call.
// ─────────────────────────────────────────────────────────────────────────────
int RobotProtocol::checkBufRefresh(void)
{
    int ret = 0;
    for (int i = 0; i < _len; i++) {
        if (_now_buf[i] != _old_buf[i]) {
            ret = 1;
            break;
        }
    }
    for (int i = 0; i < _len; i++) {
        _old_buf[i] = _now_buf[i];
    }
    return ret;
}
