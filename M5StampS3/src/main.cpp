#include <Arduino.h>

int16_t yaw, pitch, roll;

void setup() {
      // シリアル通信の初期化
      USBSerial.begin(9600);  // pc
      Serial.begin(9600);     // raspico
      delay(1000);
      USBSerial.println("system start");
}

void loop() {
      static const uint8_t HEADER = 0xFF;   // ヘッダ
      static const uint8_t FOOTER = 0xAA;   // ヘッダ
      static const uint8_t data_size = 6;   // データのサイズ
      static uint8_t index = 0;             // 受信したデータのインデックスカウンター
      static uint8_t recv_data[data_size];  // 受信したデータ
      static uint8_t recv_byte;
      if (Serial.available()) {
            recv_byte = Serial.read();
            if (index == 0) {
                  if (recv_byte == HEADER) {
                        index++;
                  } else {
                        index = 0;
                  }
            } else if (index == (data_size + 1)) {
                  if (recv_byte == FOOTER) {
                        yaw = ((((uint16_t)recv_data[0] << 8) & 0xFF00) | ((int16_t)recv_data[1] & 0x00FF)) - 32768;
                        pitch = ((((uint16_t)recv_data[2] << 8) & 0xFF00) | ((int16_t)recv_data[3] & 0x00FF)) - 32768;
                        roll = ((((uint16_t)recv_data[4] << 8) & 0xFF00) | ((int16_t)recv_data[5] & 0x00FF)) - 32768;
                  }
                  index = 0;
            } else {
                  recv_data[index - 1] = recv_byte;
                  index++;
            }
            USBSerial.print("yaw = ");
            USBSerial.print(yaw);
            USBSerial.print(", pitch = ");
            USBSerial.print(pitch);
            USBSerial.print(", roll = ");
            USBSerial.println(roll);
      }
      USBSerial.println("system start");
}
