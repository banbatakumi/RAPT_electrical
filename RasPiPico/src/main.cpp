#include <Arduino.h>

#include "MPU6050_6Axis_MotionApps20.h"

// mpu関連
MPU6050 mpu;

// MPU control/status vars
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [roll, pitch, yaw]   roll/pitch/yaw container and gravity

int16_t yaw, pitch, roll;

void setup() {
      Serial.begin(9600);
      delay(1000);
      Serial.println("system start");

      Serial1.begin(9600);  // UART0初期化 TX:GP0 / RX:GP1

      // mpu関連
      Wire.begin();
      Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
      mpu.initialize();
      devStatus = mpu.dmpInitialize();

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXAccelOffset(-813);
      mpu.setYAccelOffset(1992);
      mpu.setZAccelOffset(1712);
      mpu.setXGyroOffset(-64);
      mpu.setYGyroOffset(36);
      mpu.setZGyroOffset(43);

      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
            // Calibration Time: generate offsets and calibrate our MPU6050
            // mpu.CalibrateAccel(6);
            // mpu.CalibrateGyro(6);
            // mpu.PrintActiveOffsets();
            mpu.setDMPEnabled(true);
      } else {
            Serial.print("DMP Initialization failed.");
      }
}

void loop() {
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = ypr[0] * 180 / M_PI;
            pitch = ypr[1] * 180 / M_PI;
            roll = ypr[2] * 180 / M_PI;

            // M5にデータ送信
            Serial1.write(0xFF);
            Serial1.write((uint8_t)((yaw & 0xFF00) >> 8));
            Serial1.write((uint8_t)(yaw & 0x00FF));
            Serial1.write((uint8_t)((pitch & 0xFF00) >> 8));
            Serial1.write((uint8_t)(pitch & 0x00FF));
            Serial1.write((uint8_t)((roll & 0xFF00) >> 8));
            Serial1.write((uint8_t)(roll & 0x00FF));
            Serial1.write(0xAA);
            // Serial.print("yaw = ");
            // Serial.print(yaw);
            // Serial.print(", pitch = ");
            // Serial.print(pitch);
            // Serial.print(",roll = ");
            // Serial.println(roll);
      }
}