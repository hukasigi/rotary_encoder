#include <Arduino.h>

const int8_t  PIM_ROTARY_A   = 26;
const int8_t  PIN_ROTARY_B   = 27;
volatile long rolls          = 0;
volatile bool value_rotary_b = 0;
static int    cnt            = 0;

// 一秒周期のタイマー割り込み カウントリセット
// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    value_rotary_b = digitalRead(PIN_ROTARY_B);
    value_rotary_b ? rolls++ : rolls--;
}

void setup() {
    Serial.begin(115200);
    pinMode(PIM_ROTARY_A, INPUT_PULLUP);
    pinMode(PIN_ROTARY_B, INPUT_PULLUP);
    attachInterrupt(PIM_ROTARY_A, detect_turn_a, FALLING);
}

void loop() {
    Serial.print("rolls");
    Serial.println(rolls);
    delay(50);
}