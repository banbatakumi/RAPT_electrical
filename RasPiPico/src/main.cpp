#include <Arduino.h>

void setup() {
      Serial.begin(9600);
      Serial1.begin(9600);  // UART0初期化 TX:GP0 / RX:GP1
}

void loop() {
      if (Serial1.available()) {
            Serial.println("received data");
            Serial.print(Serial1.read());
      }
      sleep_ms(100);
}