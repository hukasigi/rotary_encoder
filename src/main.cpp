#include <Arduino.h>

const int8_t PIN_ROTARY_A = 26;
const int8_t PIN_ROTARY_B = 27;
const int8_t PIN_PWM      = 25;
const int8_t PIN_DIR      = 33;

const double TARGET_RPM = 400.;
const double RESOLUTION = 2048.;

const double KP = 1.;
const double KI = 2.;
const double KD = 0.2;

double du        = 0;
double output    = 0;
double integral  = 0;
double deriv     = 0;
double prop      = 0;
double pre_prop  = 0;
double pre_error = 0;

volatile long rolls          = 0;
volatile long rolls_is       = 0;
volatile bool value_rotary_b = 0;
volatile long speed          = 0;

static unsigned long last    = 0;
static double        lastRpm = 0;

hw_timer_t*  timer    = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
// 一秒周期のタイマー割り込み カウントリセット
// 立ち上がったときのB相のを見て回転方向を読み取る
// HIGHだったらcw(),LOWだったらccw()のように
void IRAM_ATTR detect_turn_a() {
    portENTER_CRITICAL_ISR(&timerMux);
    value_rotary_b = gpio_get_level((gpio_num_t)PIN_ROTARY_B);
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
    pinMode(PIN_ROTARY_A, INPUT_PULLUP);
    pinMode(PIN_ROTARY_B, INPUT_PULLUP);
    pinMode(PIN_DIR, OUTPUT);

    ledcSetup(0, 20000, 8);
    ledcAttachPin(PIN_PWM, 0);

    attachInterrupt(PIN_ROTARY_A, detect_turn_a, FALLING);

    // timer0使用して、分周80 1カウント1us
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);

    // 10,000 10ms
    timerAlarmWrite(timer, 10000, true);

    // timer開始
    timerAlarmEnable(timer);
}

void loop() {
    unsigned long now_t = micros();

    if (now_t - last >= 10000) { // 10ms
        double dt = (double)(now_t - last) / 1.e6;
        last      = now_t;
        double r;
        portENTER_CRITICAL(&timerMux);
        r = rolls_is;
        portEXIT_CRITICAL(&timerMux);

        double rpm = r / RESOLUTION * 60. / dt;

        // 目標値 - 実測値でエラーを出す
        double error = TARGET_RPM - rpm;

        double prop  = error - pre_error;
        double deriv = prop - pre_prop;
        double du    = KP * prop + KI * error * dt + KD * deriv;
        output += du;
        double clamped_output = constrain(abs(output), 0., 255.);
        ledcWrite(0, clamped_output);

        Serial.printf("RPM: %.1f  PWM: %.1f ERROR : %.1f\n", rpm, clamped_output, error);

        pre_error = error;
        pre_prop  = prop;
    }
}
