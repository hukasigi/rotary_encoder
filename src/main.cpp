#include <Arduino.h>

const uint8_t PIN_ROTARY_A = 26;
const uint8_t PIN_ROTARY_B = 27;
const uint8_t PIN_PWM      = 25;
const uint8_t PIN_DIR      = 33;
const uint8_t PWM_CHANNEL  = 0;

const double INTEGRAL_MAX = 500.;
const double INTEGRAL_MIN = -500.;

const uint16_t FREQUENCY       = 20000;
const uint8_t  RESOLUTION_BITS = 8;

// 制御周期
const uint16_t CONTROL_CYCLE = 100000;

// 目標位置
const int16_t angle  = 200;
int           target = map(angle, 0, 360, 0, 2024);

const double KP = 0.3;
const double KI = 0.002;
const double KD = 0.005;

volatile long pos            = 0;
volatile bool value_rotary_b = 0;
double        integral       = 0;
double        prev_error     = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    bool b_in = gpio_get_level((gpio_num_t)PIN_ROTARY_B);
    portENTER_CRITICAL_ISR(&timerMux);
    b_in ? pos-- : pos++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_ROTARY_A, INPUT_PULLUP);
    pinMode(PIN_ROTARY_B, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);

    attachInterrupt(PIN_ROTARY_A, detect_turn_a, FALLING);

    ledcAttachPin(PIN_PWM, PWM_CHANNEL);
    ledcSetup(PWM_CHANNEL, FREQUENCY, RESOLUTION_BITS);
}

void loop() {
    static unsigned long prev_time = micros();
    unsigned long        now_time  = micros();
    if (now_time - prev_time >= CONTROL_CYCLE) {
        double dt = (now_time - prev_time) / 1e6;
        prev_time = now_time;

        /* NOTE: 割り込みが発生したときにposの値は違う値になってしまうので、
        別の変数にコピーして使用する*/
        long pos_now;
        portENTER_CRITICAL(&timerMux);
        pos_now = pos;
        portEXIT_CRITICAL(&timerMux);

        int error = target - pos_now;

        double derivative = (error - prev_error) / dt;
        prev_error        = error;

        integral += error * dt;
        // I項によるオーバーフローを防ぐため
        integral = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);

        double control = KP * error + KI * integral + KD * derivative;

        uint8_t pwm = constrain(abs(control), 0, 255);

        digitalWrite(PIN_DIR, control > 0 ? HIGH : LOW);
        ledcWrite(PWM_CHANNEL, pwm);

        Serial.print("pos  ");
        Serial.println(pos_now);
    }
}