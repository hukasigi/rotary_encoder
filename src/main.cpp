#include <Arduino.h>

const int8_t PIN_ROTARY_A = 26;
const int8_t PIN_ROTARY_B = 27;
const int8_t PIN_PWM      = 25;
const int8_t PIN_DIR      = 33;

// 目的角/2048
const int16_t target = 1000;
// デルタ秒
const double DT = 0.01;

const double KP = 0.3;
const double KI = 0.002;
const double KD = 0.005;

volatile long pos            = 0;
volatile bool value_rotary_b = 0;
static long   lastPos        = 0;
double        integral       = 0;

hw_timer_t* timer = NULL;

// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    value_rotary_b = digitalRead(PIN_ROTARY_B);
    value_rotary_b ? pos-- : pos++;
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ROTARY_A, INPUT_PULLUP);
    pinMode(PIN_ROTARY_B, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);

    attachInterrupt(PIN_ROTARY_A, detect_turn_a, FALLING);

    ledcAttachPin(PIN_PWM, 0);
    ledcSetup(0, 20000, 8);
}

void loop() {
    int now = 0;

    noInterrupts();
    now = pos;
    interrupts();

    int error = target - now;

    double velocity   = (now - lastPos) / DT;
    double derivative = -velocity;
    lastPos           = now;

    double control = KP * error + KI * integral + KD * derivative;

    // 出力が飽和してないときだけ積分
    if ((abs(control) < 255) && (abs(error) < 100)) {
        integral += error * DT;
    }

    integral = constrain(integral, -1000, 1000);

    bool dir = (control > 0);
    digitalWrite(PIN_DIR, dir);

    int pwm = abs(control);
    pwm     = constrain(pwm, 0, 255);

    if (abs(error) < 3) pwm = 0;

    ledcWrite(0, pwm);

    Serial.print("pos  ");
    Serial.println(now);

    delay(10);
}