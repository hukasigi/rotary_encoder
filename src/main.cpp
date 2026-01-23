#include <Arduino.h>

const int8_t PIN_ROTARY_A = 26;
const int8_t PIN_ROTARY_B = 27;
const int8_t PIN_PWM      = 25;
const int8_t PIN_DIR      = 33;

// 目的角/2048
const int16_t target = 1000;

const double KP = 0.3;
const double KI = 0.002;
const double KD = 0.005;

volatile long pos            = 0;
volatile bool value_rotary_b = 0;
static long   lastPos        = 0;
double        integral       = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    bool b = gpio_get_level((gpio_num_t)PIN_ROTARY_B);
    portENTER_CRITICAL_ISR(&timerMux);
    b ? pos-- : pos++;
    portEXIT_CRITICAL_ISR(&timerMux);
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
    static unsigned long last  = micros();
    unsigned long        now_t = micros();
    if (now_t - last >= 10000) {
        double DT = (now_t - last) / 1e6;
        last      = now_t;

        long now;
        portENTER_CRITICAL(&timerMux);
        now = pos;
        portEXIT_CRITICAL(&timerMux);

        int error = target - now;

        static double vel_f = 0;

        double vel        = (now - lastPos) / DT;
        vel_f             = 0.7 * vel_f + 0.3 * vel;
        double derivative = -vel_f;

        lastPos = now;

        double control = KP * error + KI * integral + KD * derivative;

        if (abs(control) < 255) {
            integral += error * DT;
        }

        integral = constrain(integral, -500, 500);

        bool dir = (control > 0);
        digitalWrite(PIN_DIR, dir);

        int pwm = abs(control);
        pwm     = constrain(pwm, 0, 255);

        if (abs(error) < 3) pwm = 0;

        ledcWrite(0, pwm);

        Serial.print("pos  ");
        Serial.println(now);
    }
}