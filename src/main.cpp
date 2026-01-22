#include <Arduino.h>

const int8_t  PIM_ROTARY_A = 26;
const int8_t  PIN_ROTARY_B = 27;
const int8_t  PIN_PWM      = 25;
const int8_t  PIN_DIR      = 33;
const double  TARGET_RPM   = 50.;
const int16_t RESOLUTION   = 2048;

const double KP = 0.2;
const double KI = 0.005;

int    pwm      = 0;
double rpm_f    = 0;
double integral = 0;

volatile long rolls          = 0;
volatile long rolls_is       = 0;
volatile bool value_rotary_b = 0;
volatile long pos            = 0;
volatile long speed          = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
// 一秒周期のタイマー割り込み カウントリセット
// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    portENTER_CRITICAL_ISR(&timerMux);
    value_rotary_b = digitalRead(PIN_ROTARY_B);
    value_rotary_b ? rolls++ : rolls--;
    portEXIT_CRITICAL_ISR(&timerMux);
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

    // 10,000 10ms
    timerAlarmWrite(timer, 10000, true);

    // timer開始
    timerAlarmEnable(timer);
}

void loop() {
    long r;
    portENTER_CRITICAL(&timerMux);
    r = rolls_is;
    portEXIT_CRITICAL(&timerMux);

    double rpm = (double)r / RESOLUTION * 60. * 100.;

    rpm_f = rpm_f * 0.7 + rpm * 0.3;

    double error = TARGET_RPM - rpm_f;

    integral += error; // 誤差をためる
    double control = KP * error + KI * integral;
    pwm += control; // PWMを少しずつ動かす

    pwm = constrain(pwm, 0, 255);

    ledcWrite(0, pwm);

    Serial.printf("RPM: %.1f  PWM: %d ERROR : %.1f\n", rpm, pwm, error);
}