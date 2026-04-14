/**
 * Peltier Control System with Safety Interlock
 * * [概要]
 * 本プログラムはESP32を使用し、ペルチェ素子の温度をPID制御する。
 * [特徴]
 * 1. 二重の安全停止(Safety Stop)機能：熱電対および上位PC(Qt)との通信異常を検知
 * 2. 非ブロッキング・シリアル通信：高頻度(5ms周期想定)のデータ受信に対応
 * 3. I2C経由でのPIC16LF制御：ペルチェ駆動ボードへPWM指令を送信
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MAX31855.h>

// --- ピンアサイン ---
#define MAXCS   5       // SPI: MAX31855用チップセレクト
#define I2C_SDA 21      // I2C: データライン
#define I2C_SCL 22      // I2C: クロックライン

// --- 通信インスタンス ---
// Serial  : デバッグ用(USB経由)
// Serial2 : 上位Qtアプリ通信用(UART経由) 16:RX, 17:TX
HardwareSerial& QtSerial = Serial2; 

// --- スレーブ側(PIC16LF) I2C定義 ---
#define I2C_PELTIER_ADDR    0x09
#define I2C_SADDR_PWM_L     0x00    // PWM出力(下位8bit)
#define I2C_SADDR_PWM_H     0x01    // PWM出力(上位8bit)
#define I2C_SADDR_RUN       0x04    // 出力許可フラグ
#define I2C_SADDR_WDT_CLEAR 0x07    // PIC側のWatchdogリセットレジスタ

// --- 制御パラメータ ---
const float TEMP_HIGH = 90.0;       // 加熱フェーズの目標温度
const float TEMP_LOW  = 10.0;       // 冷却フェーズの目標温度
const float MIN_AR_CCW = 0.1;       // 状態遷移の閾値(下限)
const float MAX_AR_CCW = 4.0;       // 状態遷移の閾値(上限)
const unsigned long PHASE_DURATION = 5400000; // 各フェーズの最大継続時間(90分)

/**
 * 稼働継続の健全性を管理するHealthStatus構造体
 * センサーや外部通信が死んだままペルチェを動かすと焼損の恐れがあるため、
 * ここで一括管理し、異常時は即座に出力を0にする(インターロック)。
 */
struct HealthStatus {
    bool sensorOk = true;           // 熱電対が正しく読み取れているか
    bool qtOk = true;               // 上位PC(Qt)からの生存信号があるか
    int sensorErrorCount = 0;       // センサー読み取りエラーの累積カウンタ
    unsigned long lastQtMessageTime = 0; // 最後にPCからデータを受信した時刻

    // 全ての安全条件を満たしているか？
    bool isAllSystemsGo() {
        return sensorOk && qtOk;
    }
};

/**
 * PID制御用変数
 * 位置型PIDアルゴリズムで使用
 */
struct PID_t {
    float P = 0.1, I = 0.5, D = 0.2;
    float err_sum = 0.0;            // 積分項(I)の累積
    float pv_old = 0.0;             // 前回測定値(D項計算用)
    unsigned long t_old = 0;        // 前回計算時刻
};

// --- グローバルインスタンス ---
Adafruit_MAX31855 thermocouple(MAXCS);
PID_t pid;
HealthStatus health;

enum State { HEATING, COOLING };    // 加熱か冷却かの運転ステート
State currentState = HEATING;
unsigned long stateStartTime = 0;

float currentTemp = 0.0;            // 現在の温度
float targetTemp = TEMP_HIGH;       // 目標温度
float AR_CCW = 0.0;                 // PCから送られてくる計測指標
unsigned long lastControlTime = 0;  // 制御周期タイマー用

// --- Qtシリアル受信管理(高頻度対応) ---
char rxBuf[32];                     // 受信文字列の一時格納用
int rxIdx = 0;                      // バッファの現在位置

// --- 安全しきい値設定 ---
const int MAX_SENSOR_ERRORS = 10;      // 10回連続でNaN(読み取り不能)なら異常とみなす
const unsigned long QT_TIMEOUT_MS = 30000; // PCからの通信が30秒途絶えたら異常とみなす

// --- 関数宣言 ---
void updateHealth();
void handleQtSerial();
float calculatePID(float pv, float sv);
void setPeltierOutput(float duty_percent);
void sendCommand(uint8_t reg, uint8_t data);
void changeState(State newState);

// ==========================================
// 初期設定(Setup)
// ==========================================
void setup() {
    Serial.begin(115200);           // USBデバッグモニタ
    QtSerial.begin(9600, SERIAL_8N1, 16, 17); // 上位PC通信用
    Wire.begin(I2C_SDA, I2C_SCL);   // I2Cバス開始
    
    Serial.println("\n--- Robust Peltier Controller System ---");

    // 1. 温度計の初期化確認
    if (!thermocouple.begin()) {
        Serial.println("[CRITICAL] MAX31855 not found. Halted.");
        while (1) delay(10);        // センサーなしでの稼働は不可能なため停止
    }

    // 2. ペルチェ駆動ボード(PIC)との疎通確認
    Wire.beginTransmission(I2C_PELTIER_ADDR);
    if (Wire.endTransmission() == 0) {
        sendCommand(I2C_SADDR_RUN, 0x00); // 駆動許可を出す
        Serial.println("Action: PIC16LF Output Enabled");
    } else {
        Serial.println("[WARNING] PIC16LF Board not responding on I2C.");
    }

    health.lastQtMessageTime = millis();
    changeState(HEATING);           // 加熱フェーズからスタート
}

// ==========================================
// メインループ(Main Loop)
// ==========================================
void loop() {
    // 1. 各デバイスの「健康状態」を毎ループ更新
    // センサーから値を読み取り、PCからのデータをバッファに詰める
    updateHealth();

    // 2. システムの健全性に基づいた条件分岐(インターロック)
    if (health.isAllSystemsGo()) {
        
        // --- [A: 正常運転モード] ---
        unsigned long elapsed = millis() - stateStartTime;
        
        // 状態遷移の判定(時間または外部パラメータAR_CCWによる)
        if (currentState == HEATING) {
            targetTemp = TEMP_HIGH;
            if (elapsed >= PHASE_DURATION || AR_CCW <= MIN_AR_CCW) {
                changeState(COOLING);
            }
        } else {
            targetTemp = TEMP_LOW;
            if (elapsed >= PHASE_DURATION || AR_CCW >= MAX_AR_CCW) {
                changeState(HEATING);
            }
        }

        // 制御周期(500ms)の実行
        if (millis() - lastControlTime >= 500) {
            lastControlTime = millis();
            
            // PID計算
            float output = calculatePID(currentTemp, targetTemp);
            
            // PICへのPWM出力送信
            setPeltierOutput(output);

            // デバッグログ
            Serial.printf("[RUN] PV:%.2f SV:%.2f OUT:%.1f%% AR:%.2f\n", 
                          currentTemp, targetTemp, output, AR_CCW);
            
            // Linux Qtアプリへの温度報告
            //QtSerial.println(currentTemp); 
        }
        
    } else {
        // --- [B: 安全停止モード] ---
        // センサー断線や通信途絶時は出力を強制的に0にする
        setPeltierOutput(0);
        
        // エラー内容を2秒間隔で通知
        static unsigned long lastWarn = 0;
        if (millis() - lastWarn > 2000) {
            Serial.printf("!!! SAFETY STOP ACTIVE !!! Reason -> Sensor:%s, PC Link:%s\n", 
                          health.sensorOk ? "OK" : "ERROR", 
                          health.qtOk ? "OK" : "TIMEOUT");
            lastWarn = millis();
        }
    }
}

// ==========================================
// システム診断・通信関数
// ==========================================

/**
 * デバイスの健康診断とデータ更新
 */
void updateHealth() {
    // --- 熱電対のチェック ---
    float raw = (float)thermocouple.readCelsius();
    if (isnan(raw)) {
        health.sensorErrorCount++;
        if (health.sensorErrorCount >= MAX_SENSOR_ERRORS) {
            health.sensorOk = false; // 連続エラーでシステム異常と断定
        }
    } else {
        currentTemp = raw;          // 正常なら温度を更新
        health.sensorErrorCount = 0; // カウンタをリセット
        health.sensorOk = true; 
    }

    // --- PC(Qt)通信の受信とタイムアウトチェック ---
    handleQtSerial();
    if (millis() - health.lastQtMessageTime > QT_TIMEOUT_MS) {
        health.qtOk = false;        // 通信途絶
    } else {
        health.qtOk = true;         // 通信健全
    }
}

/**
 * PC(Qt)からのシリアル受信処理
 * Stringクラスを使わない非ブロッキング読み取りによりメモリ断片化と遅延を防止。
 * 5ms周期のような高頻度データ送信でもメインループを阻害しない。
 */
void handleQtSerial() {
    while (QtSerial.available() > 0) {
        char c = (char)QtSerial.read();
        
        if (c == '\n') { // 改行を受信したら文字列確定
            rxBuf[rxIdx] = '\0';
            if (rxIdx > 0) {
                float val = (float)atof(rxBuf);
                // 受信データのバリデーション(物理的にあり得ない値をノイズとして弾く)
                if (val >= 0.0 && val <= 20.0) { 
                    AR_CCW = val;
                    health.lastQtMessageTime = millis(); // 受信成功時刻を更新
                }else{
                  // 範囲外データを受信した場合の警告（デバッグ用）
                  //Serial.printf("[Warning] Out of range AR received: %.2f\n", val);
                }
            }
            rxIdx = 0; // 次の受信に向けてポインタをリセット
        } 
        else if (c != '\r' && rxIdx < (int)sizeof(rxBuf) - 1) {
            // ホワイトリストフィルタリング：
            // 数値、小数点、符号のみをバッファに許可。これによりノイズ文字による誤動作を防止。
            if (isDigit(c) || c == '.' || c == '-' || c == '+') {
                rxBuf[rxIdx++] = c;
            }
        }
    }
}

/**
 * フェーズ(状態)の切り替え処理
 * 各ステート開始時に積分項をリセットし、オーバーシュートを防ぐ。
 */
void changeState(State newState) {
    currentState = newState;
    stateStartTime = millis();
    pid.err_sum = 0; // 積分項のリセット(Anti-Windup)
    Serial.printf(">>> Event: Switched to %s MODE\n", newState == HEATING ? "HEATING" : "COOLING");
}

/**
 * PID計算エンジン
 * pv: 現在の温度(Process Value), sv: 目標温度(Set Value)
 */
float calculatePID(float pv, float sv) {
    unsigned long now = millis();
    if (pid.t_old == 0) {
        pid.t_old = now; pid.pv_old = pv;
        return 0;
    }
    float dt = (float)(now - pid.t_old) / 1000.0;
    if (dt <= 0) return 0;

    // 偏差の計算
    float err = sv - pv;
    
    // I項：偏差の積算
    pid.err_sum += err * dt;
    
    // D項：測定値の変化量(Derivative on Measurement)
    // 急激な目標値変更によるスパイクを防ぐため、偏差ではなくpvの微分を使用
    float d_term = (pv - pid.pv_old) / dt;
    
    // 操作量の算出
    float res = pid.P * (-pv + pid.I * pid.err_sum - pid.D * d_term);

    // 操作量のガード(100%〜-100%)
    if (res > 100.0) { 
        res = 100.0; 
        pid.err_sum -= err * dt; // アンチワインドアップ：上限到達時は積算を止める
    } else if (res < -100.0) { 
        res = -100.0; 
        pid.err_sum -= err * dt; // 下限到達時も積算を止める
    }

    pid.pv_old = pv; 
    pid.t_old = now;
    return res;
}

/**
 * I2C経由でのペルチェ電力指令送信
 * duty_percent: -100.0(最大冷却) 〜 100.0(最大加熱)
 */
void setPeltierOutput(float duty_percent) {
    // 指令値を10倍して整数化(例: 50.5% -> 505)
    int16_t s_duty = (int16_t)(duty_percent * 10.0);
    
    // PICの各レジスタへ分割送信
    sendCommand(I2C_SADDR_PWM_L, (uint8_t)(s_duty & 0xFF));
    sendCommand(I2C_SADDR_PWM_H, (uint8_t)((s_duty >> 8) & 0xFF));
    
    // PIC側のWatchdogをクリア。ESP32がフリーズしてこれが送られなくなると、
    // PIC側でも独立して出力を停止するように設計。
    sendCommand(I2C_SADDR_WDT_CLEAR, 0x00);
}

/**
 * I2C基本送信関数
 */
void sendCommand(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(I2C_PELTIER_ADDR);
    Wire.write(reg);
    Wire.write(data);
    byte error = Wire.endTransmission();
    if (error != 0) {
        // I2C通信エラー発生時のログ(必要に応じて拡張)
        // Serial.printf("[I2C ERR] Code:%d\n", error);
    }
}