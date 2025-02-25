#include <Arduino.h>

void setup() {
      USBSerial.begin(9600);  // シリアル通信の初期化
}

void loop() {
      USBSerial.println("Hello World");  // Hello Worldを送信
}
