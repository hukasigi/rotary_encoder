#include <Arduino.h>

const int8_t  PIM_ROTARY_A = 26;
const int8_t  PIN_ROTARY_B = 27;
const int8_t  PIN_PWM      = 25;
const int8_t  PIN_DIR      = 33;
const double  TARGET_RPM   = 90.;
const int16_t RESOLUTION   = 2048;

const double  KP             = 0.5;
int           pwm            = 0;
volatile long rolls          = 0;
volatile long rolls_is       = 0;
volatile bool value_rotary_b = 0;
volatile long pos            = 0;
volatile long speed          = 0;
// volatile long lastPos        = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
// 一秒周期のタイマー割り込み カウントリセット
// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    value_rotary_b = digitalRead(PIN_ROTARY_B);
    value_rotary_b ? rolls++ : rolls--;
}

void IRAM_ATTR onTimer() {
    // 読み込みを安定させるための呪文
    portENTER_CRITICAL_ISR(&timerMux);
    rolls_is = rolls;
    rolls    = 0;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    Serial.begin(115200);
    pinMode(PIM_ROTARY_A, INPUT_PULLUP);
    pinMode(PIN_ROTARY_B, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);

    ledcSetup(0, 20000, 8);
    ledcAttachPin(PIN_PWM, 0);

    attachInterrupt(PIM_ROTARY_A, detect_turn_a, FALLING);

    // timer0使用して、分周80 1カウント1us
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);

    // 1,000,000us=1秒
    timerAlarmWrite(timer, 100000, true);

    // timer開始
    timerAlarmEnable(timer);
}

void loop() {
    long r;
    portENTER_CRITICAL(&timerMux);
    r = rolls_is;
    portEXIT_CRITICAL(&timerMux);

    double rpm = (double)r / RESOLUTION * 60. * 10.;

    double error = TARGET_RPM - rpm;

    pwm += KP * error;

    if (pwm > 255) pwm = 255;
    if (pwm < 0) pwm = 0;

    ledcWrite(0, pwm);

    Serial.printf("RPM: %.1f  PWM: %d\n ERROR : %d\n", rpm, pwm, error);
}