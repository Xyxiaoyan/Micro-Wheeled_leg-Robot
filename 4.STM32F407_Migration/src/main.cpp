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

// main.cpp – STM32F407 port of wl_pro_robot.ino
//
// Key differences from the ESP32 version:
//  1. PlatformIO project (no .ino); Arduino.h included explicitly.
//  2. WiFi / WebServer / WebSocketsServer / ArduinoJson removed.
//     Control is received via 20-byte binary UART packets on Serial (USB CDC).
//  3. ESP32-specific esp_adc_cal replaced with standard analogRead().
//  4. Pin numbers updated for STM32F407 (see PIN MAP section below).
//  5. I2C buses use STM32duino TwoWire constructor with explicit SDA/SCL pins.
//  6. Servo UART uses a dedicated HardwareSerial on UART4 (PC10/PC11).
//  7. Function prototypes declared explicitly (required outside .ino context).
//
// ─── PIN MAP ─────────────────────────────────────────────────────────────────
//  Motor 1 PWM          PA8 / PA9 / PA10       TIM1 CH1/CH2/CH3
//  Motor 1 Enable       PA11
//  Motor 2 PWM          PC6 / PC7 / PC8        TIM8 CH1/CH2/CH3
//  Motor 2 Enable       PC9
//  Encoder 1  (I2C1)    PB8 (SCL)  PB9 (SDA)  AS5600 #1
//  Encoder 2  (I2C2)    PB10 (SCL) PB11 (SDA) AS5600 #2
//  MPU6050    (I2C2)    PB10 (SCL) PB11 (SDA) shared with Encoder 2
//  Servo UART (UART4)   PC10 (TX)  PC11 (RX)  STS3032 @ 1 Mbaud
//  Serial / Commander   USB Virtual COM (CDC)  mapped by -DUSBD_USE_CDC
//  Battery ADC          PA0                    ADC1_IN0, 3.3 V ref
//  LED (battery)        PD12                   GPIO output (Discovery green LED)
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <SimpleFOC.h>
#include "Servo_STS3032.h"
#include "robot.h"
#include "comm.h"

// ─── Function prototypes ─────────────────────────────────────────────────────
void lqr_balance_loop(void);
void leg_loop(void);
void jump_loop(void);
void yaw_loop(void);
void yaw_angle_addup(void);
void bat_check(void);

// Commander callback prototypes
void StabAngle(char *cmd);
void StabGyro(char *cmd);
void StabDistance(char *cmd);
void StabSpeed(char *cmd);
void StabYawAngle(char *cmd);
void StabYawGyro(char *cmd);
void lpfJoyy(char *cmd);
void StabLqrU(char *cmd);
void StabZeropoint(char *cmd);
void lpfZeropoint(char *cmd);
void StabRollAngle(char *cmd);
void lpfRoll(char *cmd);

// ─── Motor instances ─────────────────────────────────────────────────────────
BLDCMotor        motor1  = BLDCMotor(7);
BLDCMotor        motor2  = BLDCMotor(7);
// STM32F407 pin assignments (replace ESP32 GPIO 32/33/25/22 and 26/27/14/12)
BLDCDriver3PWM   driver1 = BLDCDriver3PWM(PA8, PA9, PA10, PA11);
BLDCDriver3PWM   driver2 = BLDCDriver3PWM(PC6, PC7, PC8,  PC9);

// ─── Encoder instances (I2C) ─────────────────────────────────────────────────
// STM32duino TwoWire constructor: TwoWire(SDA, SCL)
// ESP32 used TwoWire(bus_num) then begin(sda, scl, freq).
// STM32duino uses pin-based constructor then begin() + setClock().
TwoWire I2Cone(PB9,  PB8);   // SDA=PB9,  SCL=PB8  (I2C1) – Encoder 1
TwoWire I2Ctwo(PB11, PB10);  // SDA=PB11, SCL=PB10 (I2C2) – Encoder 2 + IMU
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// ─── PID controllers ─────────────────────────────────────────────────────────
PIDController pid_angle     {.P = 1,     .I = 0,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_gyro      {.P = 0.06f, .I = 0,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_distance  {.P = 0.5f,  .I = 0,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_speed     {.P = 0.7f,  .I = 0,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_yaw_angle {.P = 1.0f,  .I = 0,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_yaw_gyro  {.P = 0.04f, .I = 0,  .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_lqr_u     {.P = 1,     .I = 15, .D = 0, .ramp = 100000, .limit = 8};
PIDController pid_zeropoint {.P = 0.002f,.I = 0,  .D = 0, .ramp = 100000, .limit = 4};
PIDController pid_roll_angle{.P = 8,     .I = 0,  .D = 0, .ramp = 100000, .limit = 450};

// ─── Low-pass filters ────────────────────────────────────────────────────────
LowPassFilter lpf_joyy      {.Tf = 0.2f};
LowPassFilter lpf_zeropoint {.Tf = 0.1f};
LowPassFilter lpf_roll      {.Tf = 0.3f};

// ─── Commander (real-time PID tuning over Serial / USB CDC) ──────────────────
Commander command = Commander(Serial);

void StabAngle(char *cmd)     { command.pid(&pid_angle,      cmd); }
void StabGyro(char *cmd)      { command.pid(&pid_gyro,       cmd); }
void StabDistance(char *cmd)  { command.pid(&pid_distance,   cmd); }
void StabSpeed(char *cmd)     { command.pid(&pid_speed,      cmd); }
void StabYawAngle(char *cmd)  { command.pid(&pid_yaw_angle,  cmd); }
void StabYawGyro(char *cmd)   { command.pid(&pid_yaw_gyro,   cmd); }
void lpfJoyy(char *cmd)       { command.lpf(&lpf_joyy,       cmd); }
void StabLqrU(char *cmd)      { command.pid(&pid_lqr_u,      cmd); }
void StabZeropoint(char *cmd) { command.pid(&pid_zeropoint,  cmd); }
void lpfZeropoint(char *cmd)  { command.lpf(&lpf_zeropoint,  cmd); }
void StabRollAngle(char *cmd) { command.pid(&pid_roll_angle, cmd); }
void lpfRoll(char *cmd)       { command.lpf(&lpf_roll,       cmd); }

// ─── Control protocol instance ───────────────────────────────────────────────
RobotProtocol rp(ROBOT_PACKET_LEN);

// ─── Servo (STS3032) ─────────────────────────────────────────────────────────
// UART4: PC10=TX, PC11=RX @ 1 Mbaud (replaces ESP32 Serial2 @ GPIO 16/17)
HardwareSerial ServoSerial(PC11, PC10);  // RX, TX
SMS_STS        sms_sts;

// ─── IMU (MPU6050 on I2C2) ───────────────────────────────────────────────────
MPU6050 mpu6050(I2Ctwo);

// ─── Constants and parameters ────────────────────────────────────────────────
// Note: PI (uppercase) is provided by Arduino.h; no local define needed.

// LQR self-balancing controller
float LQR_angle      = 0;
float LQR_gyro       = 0;
float LQR_speed      = 0;
float LQR_distance   = 0;
float angle_control  = 0;
float gyro_control   = 0;
float speed_control  = 0;
float distance_control = 0;
float LQR_u          = 0;
float angle_zeropoint    = -2.25f;
float distance_zeropoint = -256.0f;  // sentinel: impossible value → not yet set

// YAW axis
float YAW_gyro          = 0;
float YAW_angle         = 0;
float YAW_angle_last    = 0;
float YAW_angle_total   = 0;
float YAW_angle_zero_point = -10;
float YAW_output        = 0;

// Leg servo control
byte ID[2];
s16  Position[2];
u16  Speed[2];
byte ACC[2];

// Logic flags
float robot_speed          = 0;
float robot_speed_last     = 0;
int   wrobot_move_stop_flag = 0;
int   jump_flag             = 0;
float leg_position_add      = 0;
int   uncontrolable         = 0;

// Battery monitoring
// STM32F407: analogRead() returns 0–4095 (12-bit), VREF = 3.3 V
// Adjust VBAT_DIVIDER_RATIO to match the resistor divider on your PCB.
// The original ESP32 board used a ratio of ~3.97 (8 V full → 2.015 V at ADC).
#define BAT_PIN            PA0
#define VBAT_DIVIDER_RATIO 3.97f
#define VBAT_FULL          7.8f   // V – LED on above this voltage

uint16_t bat_check_num = 0;

// Battery / status LED
// PD12 = green LED on STM32F407 Discovery; change to match your PCB.
#define LED_BAT PD12

// ─── setup() ─────────────────────────────────────────────────────────────────
void setup()
{
    // Serial (USB CDC via -DUSBD_USE_CDC in platformio.ini)
    // Used for Commander (PID tuning) and debug output.
    Serial.begin(115200);

    // Servo UART: UART4, 1 Mbaud
    ServoSerial.begin(1000000);

    // Servo initialisation
    sms_sts.pSerial = &ServoSerial;
    ID[0]      = 1;
    ID[1]      = 2;
    ACC[0]     = 30;
    ACC[1]     = 30;
    Speed[0]   = 300;
    Speed[1]   = 300;
    Position[0] = 2148;
    Position[1] = 1948;
    sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);

    // Battery LED
    pinMode(LED_BAT, OUTPUT);
    analogReadResolution(12);  // 12-bit ADC (STM32F4 default, explicit for clarity)

    // I2C buses (STM32duino: pins set in constructor, configure clock then begin)
    I2Cone.setClock(400000);
    I2Ctwo.setClock(400000);
    I2Cone.begin();
    I2Ctwo.begin();
    sensor1.init(&I2Cone);
    sensor2.init(&I2Ctwo);

    // IMU
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    // Link sensors to motors
    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);

    // Speed-loop PID
    motor1.PID_velocity.P = 0.05f;
    motor1.PID_velocity.I = 1;
    motor1.PID_velocity.D = 0;
    motor2.PID_velocity.P = 0.05f;
    motor2.PID_velocity.I = 1;
    motor2.PID_velocity.D = 0;

    // Driver setup
    motor1.voltage_sensor_align = 6;
    motor2.voltage_sensor_align = 6;
    driver1.voltage_power_supply = 8;
    driver2.voltage_power_supply = 8;
    driver1.init();
    driver2.init();

    motor1.linkDriver(&driver1);
    motor2.linkDriver(&driver2);

    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    motor1.controller        = MotionControlType::torque;
    motor2.controller        = MotionControlType::torque;

    motor1.useMonitoring(Serial);
    motor2.useMonitoring(Serial);
    motor1.init();
    motor1.initFOC();
    motor2.init();
    motor2.initFOC();

    // Map Commander characters to PID/LPF objects
    command.add('A', StabAngle,     "pid angle");
    command.add('B', StabGyro,      "pid gyro");
    command.add('C', StabDistance,  "pid distance");
    command.add('D', StabSpeed,     "pid speed");
    command.add('E', StabYawAngle,  "pid yaw angle");
    command.add('F', StabYawGyro,   "pid yaw gyro");
    command.add('G', lpfJoyy,       "lpf joyy");
    command.add('H', StabLqrU,      "pid lqr u");
    command.add('I', StabZeropoint, "pid zeropoint");
    command.add('J', lpfZeropoint,  "lpf zeropoint");
    command.add('K', StabRollAngle, "pid roll angle");
    command.add('L', lpfRoll,       "lpf roll");

    // Communication init (UART binary protocol, replaces WiFi AP)
    Comm_Init();

    delay(500);
}

// ─── loop() ──────────────────────────────────────────────────────────────────
void loop()
{
    bat_check();          // 电压检测
    comm_loop();          // UART数据更新 (replaces web_loop)
    mpu6050.update();     // IMU数据更新
    lqr_balance_loop();   // LQR自平衡控制
    yaw_loop();           // YAW轴转向控制
    leg_loop();           // 腿部动作控制

    // 将自平衡计算输出转矩赋给电机
    motor1.target = (-0.5f) * (LQR_u + YAW_output);
    motor2.target = (-0.5f) * (LQR_u - YAW_output);

    // 倒地失控后关闭输出
    if (abs(LQR_angle) > 25.0f) {
        uncontrolable = 1;
    }
    if (uncontrolable != 0) {  // 扶起后延时恢复
        if (abs(LQR_angle) < 10.0f) {
            uncontrolable++;
        }
        if (uncontrolable > 200) {
            uncontrolable = 0;
        }
    }

    // 关停输出（遥控停止或角度过大失控）
    if (wrobot.go == 0 || uncontrolable != 0) {
        motor1.target  = 0;
        motor2.target  = 0;
        leg_position_add = 0;
    }

    // 记录上一次的遥控数据
    wrobot.dir_last  = wrobot.dir;
    wrobot.joyx_last = wrobot.joyx;
    wrobot.joyy_last = wrobot.joyy;

    // 迭代计算FOC相电压
    motor1.loopFOC();
    motor2.loopFOC();

    // 设置轮部电机输出
    motor1.move();
    motor2.move();

    command.run();
}

// ─── LQR self-balancing controller ──────────────────────────────────────────
void lqr_balance_loop()
{
    LQR_distance = (-0.5f) * (motor1.shaft_angle    + motor2.shaft_angle);
    LQR_speed    = (-0.5f) * (motor1.shaft_velocity + motor2.shaft_velocity);
    LQR_angle    = (float)mpu6050.getAngleY();
    LQR_gyro     = (float)mpu6050.getGyroY();

    angle_control = pid_angle(LQR_angle - angle_zeropoint);
    gyro_control  = pid_gyro(LQR_gyro);

    if (wrobot.joyy != 0) {
        distance_zeropoint    = LQR_distance;
        pid_lqr_u.error_prev  = 0;
    }

    if ((wrobot.joyx_last != 0 && wrobot.joyx == 0) ||
        (wrobot.joyy_last != 0 && wrobot.joyy == 0)) {
        wrobot_move_stop_flag = 1;
    }
    if (wrobot_move_stop_flag == 1 && abs(LQR_speed) < 0.5f) {
        distance_zeropoint    = LQR_distance;
        wrobot_move_stop_flag = 0;
    }

    if (abs(LQR_speed) > 15) {
        distance_zeropoint = LQR_distance;
    }

    distance_control = pid_distance(LQR_distance - distance_zeropoint);
    speed_control    = pid_speed(LQR_speed - 0.1f * lpf_joyy(wrobot.joyy));

    robot_speed_last = robot_speed;
    robot_speed      = LQR_speed;
    if (abs(robot_speed - robot_speed_last) > 10 ||
        abs(robot_speed) > 50 ||
        jump_flag != 0) {
        distance_zeropoint   = LQR_distance;
        LQR_u                = angle_control + gyro_control;
        pid_lqr_u.error_prev = 0;
    } else {
        LQR_u = angle_control + gyro_control + distance_control + speed_control;
    }

    if (abs(LQR_u) < 5 && wrobot.joyy == 0 &&
        abs(distance_control) < 4 && jump_flag == 0) {
        LQR_u          = pid_lqr_u(LQR_u);
        angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control));
    } else {
        pid_lqr_u.error_prev = 0;
    }

    // 平衡控制参数自适应（与腿部高度相关）
    if (wrobot.height < 50) {
        pid_speed.P = 0.7f;
    } else if (wrobot.height < 64) {
        pid_speed.P = 0.6f;
    } else {
        pid_speed.P = 0.5f;
    }
}

// ─── Leg motion control ───────────────────────────────────────────────────────
void leg_loop()
{
    jump_loop();
    if (jump_flag == 0) {
        ACC[0]   = 8;
        ACC[1]   = 8;
        Speed[0] = 200;
        Speed[1] = 200;
        float roll_angle = (float)mpu6050.getAngleX() + 2.0f;
        leg_position_add = pid_roll_angle(lpf_roll(roll_angle));
        Position[0] = 2048 + 12 + (s16)(8.4f * (wrobot.height - 32)) - (s16)leg_position_add;
        Position[1] = 2048 - 12 - (s16)(8.4f * (wrobot.height - 32)) - (s16)leg_position_add;
        if (Position[0] < 2110) Position[0] = 2110;
        else if (Position[0] > 2510) Position[0] = 2510;
        if (Position[1] < 1586) Position[1] = 1586;
        else if (Position[1] > 1986) Position[1] = 1986;
        sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
    }
}

// ─── Jump control ────────────────────────────────────────────────────────────
void jump_loop()
{
    if (wrobot.dir_last == 5 && wrobot.dir == 4 && jump_flag == 0) {
        ACC[0]      = 0;
        ACC[1]      = 0;
        Speed[0]    = 0;
        Speed[1]    = 0;
        Position[0] = (s16)(2048 + 12 + 8.4f * (80 - 32));
        Position[1] = (s16)(2048 - 12 - 8.4f * (80 - 32));
        sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
        jump_flag = 1;
    }
    if (jump_flag > 0) {
        jump_flag++;
        if (jump_flag > 30 && jump_flag < 35) {
            ACC[0]      = 0;
            ACC[1]      = 0;
            Speed[0]    = 0;
            Speed[1]    = 0;
            Position[0] = (s16)(2048 + 12 + 8.4f * (40 - 32));
            Position[1] = (s16)(2048 - 12 - 8.4f * (40 - 32));
            sms_sts.SyncWritePosEx(ID, 2, Position, Speed, ACC);
            jump_flag = 40;
        }
        if (jump_flag > 200) {
            jump_flag = 0;
        }
    }
}

// ─── YAW axis steering control ───────────────────────────────────────────────
void yaw_loop()
{
    yaw_angle_addup();
    YAW_angle_total += wrobot.joyx * 0.002f;
    float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
    float yaw_gyro_control  = pid_yaw_gyro(YAW_gyro);
    YAW_output = yaw_angle_control + yaw_gyro_control;
}

// ─── YAW angle accumulator ───────────────────────────────────────────────────
void yaw_angle_addup()
{
    YAW_angle = (float)mpu6050.getAngleZ();
    YAW_gyro  = (float)mpu6050.getGyroZ();

    if (YAW_angle_zero_point == -10) {
        YAW_angle_zero_point = YAW_angle;
    }

    float yaw_angle_1, yaw_angle_2, yaw_addup_angle;
    if (YAW_angle > YAW_angle_last) {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last - 2 * PI;
    } else {
        yaw_angle_1 = YAW_angle - YAW_angle_last;
        yaw_angle_2 = YAW_angle - YAW_angle_last + 2 * PI;
    }
    yaw_addup_angle = (abs(yaw_angle_1) > abs(yaw_angle_2))
                          ? yaw_angle_2
                          : yaw_angle_1;

    YAW_angle_total += yaw_addup_angle;
    YAW_angle_last   = YAW_angle;
}

// ─── Battery voltage check ───────────────────────────────────────────────────
// ESP32 used esp_adc_cal_raw_to_voltage().
// STM32F407 uses standard 12-bit analogRead() with 3.3 V reference.
// Calibrate VBAT_DIVIDER_RATIO to match the actual voltage-divider resistors
// on your PCB (original board: ~3.97×).
void bat_check()
{
    if (bat_check_num > 1000) {
        float raw     = (float)analogRead(BAT_PIN);
        float battery = raw * (3.3f / 4095.0f) * VBAT_DIVIDER_RATIO;

        Serial.println(battery);
        digitalWrite(LED_BAT, (battery > VBAT_FULL) ? HIGH : LOW);

        bat_check_num = 0;
    } else {
        bat_check_num++;
    }
}
