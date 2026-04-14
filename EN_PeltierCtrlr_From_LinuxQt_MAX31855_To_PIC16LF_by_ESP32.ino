/**
 * Peltier Control System with Safety Interlock
 * [Overview]
 * This program uses an ESP32 to perform PID temperature control of a Peltier element.
 * [Features]
 * 1. Dual Safety Stop: Detects thermocouple failure and communication loss with the host PC (Qt).
 * 2. Non-blocking Serial Communication: Supports high-frequency data reception (approx. 5ms cycles).
 * 3. I2C PIC16LF Control: Sends PWM commands to the Peltier driver board.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MAX31855.h>

// --- Pin Assignments ---
#define MAXCS   5       // SPI: Chip Select for MAX31855
#define I2C_SDA 21      // I2C: Data Line
#define I2C_SCL 22      // I2C: Clock Line

// --- Communication Instances ---
// Serial  : For USB Debugging
// Serial2 : For Host Qt App (UART) 16:RX, 17:TX
HardwareSerial& QtSerial = Serial2; 

// --- Slave Device (PIC16LF) I2C Definitions ---
#define I2C_PELTIER_ADDR    0x09
#define I2C_SADDR_PWM_L      0x00    // PWM Output (Lower 8-bit)
#define I2C_SADDR_PWM_H      0x01    // PWM Output (Upper 8-bit)
#define I2C_SADDR_RUN        0x04    // Output Enable Flag
#define I2C_SADDR_WDT_CLEAR 0x07    // Watchdog Reset Register on PIC side

// --- Control Parameters ---
const float TEMP_HIGH = 90.0;       // Target temp for Heating Phase
const float TEMP_LOW  = 10.0;       // Target temp for Cooling Phase
const float MIN_AR_CCW = 0.1;       // State transition threshold (Lower limit)
const float MAX_AR_CCW = 4.0;       // State transition threshold (Upper limit)
const unsigned long PHASE_DURATION = 5400000; // Max phase duration (90 minutes)

/**
 * HealthStatus struct to manage operational safety.
 * Operating the Peltier element while sensors or external communication are dead 
 * risks hardware burnout. This struct centralizes health checks and triggers 
 * an immediate interlock (output to 0) if an anomaly is detected.
 */
struct HealthStatus {
    bool sensorOk = true;           // Is thermocouple reading correctly?
    bool qtOk = true;               // Is there a heartbeat/signal from the host PC (Qt)?
    int sensorErrorCount = 0;       // Cumulative counter for sensor read errors
    unsigned long lastQtMessageTime = 0; // Timestamp of the last data received from PC

    // Check if all safety conditions are met
    bool isAllSystemsGo() {
        return sensorOk && qtOk;
    }
};

/**
 * PID Control Variables
 * Used for the Positional PID Algorithm
 */
struct PID_t {
    float P = 0.1, I = 0.5, D = 0.2;
    float err_sum = 0.0;            // Integral term (I) accumulation
    float pv_old = 0.0;             // Previous Process Value (for D-term calculation)
    unsigned long t_old = 0;        // Previous calculation timestamp
};

// --- Global Instances ---
Adafruit_MAX31855 thermocouple(MAXCS);
PID_t pid;
HealthStatus health;

enum State { HEATING, COOLING };    // Operational state: Heating or Cooling
State currentState = HEATING;
unsigned long stateStartTime = 0;

float currentTemp = 0.0;            // Current measured temperature
float targetTemp = TEMP_HIGH;       // Target temperature
float AR_CCW = 0.0;                 // Measurement metric sent from PC
unsigned long lastControlTime = 0;  // Timer for control loop cycles

// --- Qt Serial Reception Management (High Frequency) ---
char rxBuf[32];                     // Buffer for received strings
int rxIdx = 0;                      // Current buffer position

// --- Safety Threshold Settings ---
const int MAX_SENSOR_ERRORS = 10;      // System failure if 10 consecutive NaN readings
const unsigned long QT_TIMEOUT_MS = 30000; // System failure if PC communication drops for 30s

// --- Function Prototypes ---
void updateHealth();
void handleQtSerial();
float calculatePID(float pv, float sv);
void setPeltierOutput(float duty_percent);
void sendCommand(uint8_t reg, uint8_t data);
void changeState(State newState);

// ==========================================
// Setup
// ==========================================
void setup() {
    Serial.begin(115200);           // USB Debug Monitor
    QtSerial.begin(9600, SERIAL_8N1, 16, 17); // Host PC Communication
    Wire.begin(I2C_SDA, I2C_SCL);   // Start I2C Bus
    
    Serial.println("\n--- Robust Peltier Controller System ---");

    // 1. Verify Thermocouple initialization
    if (!thermocouple.begin()) {
        Serial.println("[CRITICAL] MAX31855 not found. Halted.");
        while (1) delay(10);        // Halt execution as operation without sensor is impossible
    }

    // 2. Verify communication with Peltier Driver Board (PIC)
    Wire.beginTransmission(I2C_PELTIER_ADDR);
    if (Wire.endTransmission() == 0) {
        sendCommand(I2C_SADDR_RUN, 0x00); // Enable output
        Serial.println("Action: PIC16LF Output Enabled");
    } else {
        Serial.println("[WARNING] PIC16LF Board not responding on I2C.");
    }

    health.lastQtMessageTime = millis();
    changeState(HEATING);           // Start with Heating Phase
}

// ==========================================
// Main Loop
// ==========================================
void loop() {
    // 1. Update "Health Status" of devices every loop
    // Reads sensor values and processes serial buffer
    updateHealth();

    // 2. Conditional Branching based on system health (Interlock)
    if (health.isAllSystemsGo()) {
        
        // --- [A: Normal Operation Mode] ---
        unsigned long elapsed = millis() - stateStartTime;
        
        // State transition logic (based on time or external AR_CCW parameter)
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

        // Execution of Control Cycle (500ms)
        if (millis() - lastControlTime >= 500) {
            lastControlTime = millis();
            
            // PID Calculation
            float output = calculatePID(currentTemp, targetTemp);
            
            // Send PWM output to PIC
            setPeltierOutput(output);

            // Debug Log
            Serial.printf("[RUN] PV:%.2f SV:%.2f OUT:%.1f%% AR:%.2f\n", 
                          currentTemp, targetTemp, output, AR_CCW);
            
            // Report temperature back to Linux Qt App
            //QtSerial.println(currentTemp); 
        }
        
    } else {
        // --- [B: Safety Stop Mode] ---
        // Force output to 0 during sensor failure or communication loss
        setPeltierOutput(0);
        
        // Notify error status every 2 seconds
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
// System Diagnostics & Communication
// ==========================================

/**
 * Updates device health diagnostics and data
 */
void updateHealth() {
    // --- Thermocouple Check ---
    float raw = (float)thermocouple.readCelsius();
    if (isnan(raw)) {
        health.sensorErrorCount++;
        if (health.sensorErrorCount >= MAX_SENSOR_ERRORS) {
            health.sensorOk = false; // Confirm system failure after consecutive errors
        }
    } else {
        currentTemp = raw;           // Update temperature if valid
        health.sensorErrorCount = 0; // Reset counter
        health.sensorOk = true; 
    }

    // --- PC (Qt) Communication Heartbeat/Timeout Check ---
    handleQtSerial();
    if (millis() - health.lastQtMessageTime > QT_TIMEOUT_MS) {
        health.qtOk = false;        // Link lost
    } else {
        health.qtOk = true;         // Link healthy
    }
}

/**
 * Serial reception handler for PC (Qt)
 * Uses non-blocking reading without the String class to prevent memory fragmentation and lag.
 * Maintains main loop fluidity even with high-frequency (5ms) data bursts.
 */
void handleQtSerial() {
    while (QtSerial.available() > 0) {
        char c = (char)QtSerial.read();
        
        if (c == '\n') { // Message finalized upon newline
            rxBuf[rxIdx] = '\0';
            if (rxIdx > 0) {
                float val = (float)atof(rxBuf);
                // Data Validation (Reject physically impossible values as noise)
                if (val >= 0.0 && val <= 20.0) { 
                    AR_CCW = val;
                    health.lastQtMessageTime = millis(); // Update last reception timestamp
                } else {
                    // Log out-of-range data (for debugging)
                    //Serial.printf("[Warning] Out of range AR received: %.2f\n", val);
                }
            }
            rxIdx = 0; // Reset buffer pointer for next message
        } 
        else if (c != '\r' && rxIdx < (int)sizeof(rxBuf) - 1) {
            // Whitelist filtering:
            // Allow only digits, decimal points, and signs to prevent noise-induced bugs.
            if (isDigit(c) || c == '.' || c == '-' || c == '+') {
                rxBuf[rxIdx++] = c;
            }
        }
    }
}

/**
 * State Transition Handler
 * Resets the integral term to prevent overshoot (Anti-Windup) at the start of a phase.
 */
void changeState(State newState) {
    currentState = newState;
    stateStartTime = millis();
    pid.err_sum = 0; // Reset integral term
    Serial.printf(">>> Event: Switched to %s MODE\n", newState == HEATING ? "HEATING" : "COOLING");
}

/**
 * PID Calculation Engine
 * pv: Process Value (Current), sv: Set Value (Target)
 */
float calculatePID(float pv, float sv) {
    unsigned long now = millis();
    if (pid.t_old == 0) {
        pid.t_old = now; pid.pv_old = pv;
        return 0;
    }
    float dt = (float)(now - pid.t_old) / 1000.0;
    if (dt <= 0) return 0;

    // Calculate error
    float err = sv - pv;
    
    // I-term: Accumulate error over time
    pid.err_sum += err * dt;
    
    // D-term: Derivative on Measurement
    // Use derivative of PV rather than error to avoid spikes during target value changes.
    float d_term = (pv - pid.pv_old) / dt;
    
    // Calculate output manipulation value
    float res = pid.P * (err + pid.I * pid.err_sum - pid.D * d_term);

    // Output Guarding (100% to -100%)
    if (res > 100.0) { 
        res = 100.0; 
        pid.err_sum -= err * dt; // Anti-windup: Stop accumulation at upper limit
    } else if (res < -100.0) { 
        res = -100.0; 
        pid.err_sum -= err * dt; // Anti-windup: Stop accumulation at lower limit
    }

    pid.pv_old = pv; 
    pid.t_old = now;
    return res;
}

/**
 * Sends Peltier power commands via I2C
 * duty_percent: -100.0 (Max Cooling) to 100.0 (Max Heating)
 */
void setPeltierOutput(float duty_percent) {
    // Scale command to integer (e.g., 50.5% -> 505)
    int16_t s_duty = (int16_t)(duty_percent * 10.0);
    
    // Split and send to PIC registers
    sendCommand(I2C_SADDR_PWM_L, (uint8_t)(s_duty & 0xFF));
    sendCommand(I2C_SADDR_PWM_H, (uint8_t)((s_duty >> 8) & 0xFF));
    
    // Clear PIC-side Watchdog. If ESP32 freezes and this stop sending, 
    // the PIC is designed to autonomously shut down output.
    sendCommand(I2C_SADDR_WDT_CLEAR, 0x00);
}

/**
 * Basic I2C Transmission Function
 */
void sendCommand(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(I2C_PELTIER_ADDR);
    Wire.write(reg);
    Wire.write(data);
    byte error = Wire.endTransmission();
    if (error != 0) {
        // Log I2C errors here if necessary
        // Serial.printf("[I2C ERR] Code:%d\n", error);
    }
}
