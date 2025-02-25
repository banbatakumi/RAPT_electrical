#include <Arduino.h>

void setup() {
      // シリアル通信の初期化
      USBSerial.begin(9600);
      Serial.begin(9600);
}

void loop() {
      Serial.write(0xFF);
}
