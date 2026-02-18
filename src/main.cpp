#include <Arduino.h>

const uint8_t PIN_ROTARY_A = 26;
const uint8_t PIN_ROTARY_B = 27;
const uint8_t PIN_ROTARY_Z = 32;
const uint8_t PIN_PWM      = 25;
const uint8_t PIN_DIR      = 33;
const uint8_t PWM_CHANNEL  = 0;

const double INTEGRAL_MAX = 1000.;
const double INTEGRAL_MIN = -1000.;

const uint16_t FREQUENCY       = 20000;
const uint8_t  RESOLUTION_BITS = 8;

// 制御周期
const uint16_t CONTROL_CYCLE = 2000;

// 目標位置
const int16_t angle  = 200;
int           target = map(angle, 0, 360, 0, 2048);

const double KP = 3.;
const double KI = 0.5;
const double KD = 0.;

volatile long pos            = 0;
volatile bool value_rotary_b = 0;
double        integral       = 0;
double        prev_error     = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    portENTER_CRITICAL_ISR(&timerMux);
    value_rotary_b = gpio_get_level((gpio_num_t)PIN_ROTARY_B);
    value_rotary_b ? pos-- : pos++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
    Serial.begin(115200); // put function declarations here:
    int myFunction(int, int);

    pinMode(PIN_ROTARY_A, INPUT_PULLUP);
    pinMode(PIN_ROTARY_B, INPUT_PULLUP);
    pinMode(PIN_ROTARY_Z, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);
    ledcAttachPin(PIN_PWM, PWM_CHANNEL);
    ledcSetup(PWM_CHANNEL, FREQUENCY, RESOLUTION_BITS);

    // 初期位置合わせ
    bool prevZ = gpio_get_level((gpio_num_t)PIN_ROTARY_Z);

    ledcWrite(PWM_CHANNEL, 30); // 低速が安全

    while (1) {
        bool nowZ = gpio_get_level((gpio_num_t)PIN_ROTARY_Z);

        if (!prevZ && nowZ) { // 立ち上がり検出
            break;
        }

        prevZ = nowZ;
    }
    ledcWrite(PWM_CHANNEL, 0);
    Serial.println(pos);
    pos = 0;
    Serial.println("start");

    attachInterrupt(PIN_ROTARY_A, detect_turn_a, FALLING);
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

        uint8_t pwm = constrain(abs(control), 0., 255.);
        if (abs(error) > 1) {
            if (pwm < 50) pwm = 50;
        }

        digitalWrite(PIN_DIR, control > 0 ? HIGH : LOW);
        ledcWrite(PWM_CHANNEL, pwm);

        Serial.print("pos  ");
        Serial.printf("%d %d\n", pos_now, target);
    }
}