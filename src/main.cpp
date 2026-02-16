#include <Arduino.h>

const int8_t PIN_ROTARY_A = 26;
const int8_t PIN_ROTARY_B = 27;
const int8_t PIN_PWM      = 25;
const int8_t PIN_DIR      = 33;

const double  RPM_FIL_PER = 0.7;
const double  RPM_RAW_PER = 0.3;
const double  TARGET_RPM  = 200.;
const int16_t RESOLUTION  = 2048;

const double KP = 2.50;
const double KI = 1.00;
const double KD = 0.0005;

int    pwm_out  = 0;
double rpm_f    = 0;
double integral = 0;

volatile long rolls          = 0;
volatile long rolls_is       = 0;
volatile bool value_rotary_b = 0;
volatile long pos            = 0;
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
        double DT = (now_t - last) / 1e6;
        last      = now_t;
        long r;
        portENTER_CRITICAL(&timerMux);
        r = rolls_is;
        portEXIT_CRITICAL(&timerMux);

        double rpm = (double)r / RESOLUTION * 60. / DT;

        // 回転数にフィルタをかけてなめらかにする
        // rpm_f = 0.7 × 昔の値 + 0.3 × 今の値
        rpm_f = rpm_f * RPM_FIL_PER + rpm * RPM_RAW_PER;

        // 目標値 - 実測値でエラーを出す
        double error = TARGET_RPM - rpm_f;

        // 回転数を時間で微分
        // errorの微分だと目標rpmが変わったら急にD項が暴れる
        double derivative = -(rpm_f - lastRpm) / DT;
        lastRpm           = rpm_f;

        double control_raw = KP * error + KI * integral + KD * derivative;
        if (abs(control_raw) < 255) { // 飽和してない時だけ積分
            // ズレが長時間つづいた分を足す
            integral += error * DT;
        }

        integral       = constrain(integral, -500, 500);
        double control = KP * error + KI * integral + KD * derivative;

        bool dir = (control <= 0);
        digitalWrite(PIN_DIR, dir);
        pwm_out = constrain(abs(control), 0, 255);

        ledcWrite(0, pwm_out);

        Serial.printf("RPM: %.1f  PWM: %d ERROR : %.1f\n", rpm, pwm_out, error);
    }
}
