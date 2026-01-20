#include <Arduino.h>

const int8_t  PIM_ROTARY_A = 26;
const int8_t  PIN_ROTARY_B = 27;
const int8_t  PIN_PWM      = 25;
const int8_t  PIN_DIR      = 33;
const int16_t target       = 1000;
const float   KP           = 0.5;

volatile long pos            = 0;
volatile bool value_rotary_b = 0;
static int    cnt            = 0;
int           error          = 0;

hw_timer_t* timer = NULL;

// 一秒周期のタイマー割り込み カウントリセット
// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    value_rotary_b = digitalRead(PIN_ROTARY_B);
    value_rotary_b ? pos-- : pos++;
}

void setup() {
    Serial.begin(115200);
    pinMode(PIM_ROTARY_A, INPUT_PULLUP);
    pinMode(PIN_ROTARY_B, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);

    attachInterrupt(PIM_ROTARY_A, detect_turn_a, FALLING);

    ledcAttachPin(PIN_PWM, 0);
    ledcSetup(0, 20000, 8);
}

void loop() {
    int now = 0;

    noInterrupts();
    now = pos;
    interrupts();

    int error = target - now;

    bool dir = (error > 0);
    digitalWrite(PIN_DIR, dir);

    int pwm = abs(error * KP);
    pwm     = constrain(pwm, 0, 255);

    if (abs(error) < 5) pwm = 0;

    ledcWrite(0, pwm);

    Serial.print("pos  ");
    Serial.println(now);
    delay(10);
}