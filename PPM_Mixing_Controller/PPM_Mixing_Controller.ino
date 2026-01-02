/*
  =============================================================
  ==   Automatic TDS (PPM) Mixing Controller                 ==
  ==   VERSION 13 (Fixing Oscillation with Mixing Time)      ==
  =============================================================
  
  This version fixes the "flutter" oscillation (530-560).
  The problem is that the loop is too fast and the pulses
  are too small. The controller is reacting to false,
  unmixed readings.

  THE FIX:
  1. PUMP_DURATION: Increased from 200ms to 500ms.
     (A "bolder" pulse to make a real change).
  2. MIXING_DELAY: Increased from 800ms to 1500ms.
     (More "patience" to let the solution mix).
     
  This 2-second loop (0.5s ON, 1.5s OFF) is much
  more stable and will read the *true* PPM.
*/

// --- Include Libraries ---
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =============================================================
// ==   USER SETTINGS                                         ==
// =============================================================
LiquidCrystal_I2C lcd(0x27, 16, 2); 
const int SETPOINT_PPM = 300;
const int BRINE_PPM_HIGH_LIMIT = 625; 
const int WATER_PPM_LOW_LIMIT = 93;
const int DEADBAND_PPM = 5; 
const int MIN_PUMP_SPEED = 100; 
const int MAX_PUMP_SPEED = 255; 
const long STABILIZATION_TIME = 10000;

// -- !! NEW TIMING !! --
const long PUMP_DURATION = 500;  // Run pump for 0.5 sec (Was 200)
const long MIXING_DELAY = 1500; // Wait 1.5 sec (Was 800)
// Total loop time is now 2 seconds

// -- PI Gains --
float Kp = 1.0; 
float Ki = 0.1;
const float INTEGRAL_MAX = 500.0; 
// =============================================================

// --- Pin Definitions ---
const int TDS_PIN = A0;
const int BRINE_SPEED_PIN = 3; 
const int BRINE_IN1_PIN = 7;
const int BRINE_IN2_PIN = 6;
const int WATER_SPEED_PIN = 9;  
const int WATER_IN3_PIN = 5;
const int WATER_IN4_PIN = 4;

// --- Global Variables ---
float currentPPM = 0;
int pumpSpeed = 0;
unsigned long timeEnteredDeadband = 0;
unsigned long nextActionTime = 0;
float integralError = 0; 
enum ControllerState { STATE_READING, STATE_PUMPING };
ControllerState currentState = STATE_READING;

void setup() {
  Serial.begin(9600);
  
  pinMode(BRINE_SPEED_PIN, OUTPUT);
  pinMode(BRINE_IN1_PIN, OUTPUT);
  pinMode(BRINE_IN2_PIN, OUTPUT);
  pinMode(WATER_SPEED_PIN, OUTPUT);
  pinMode(WATER_IN3_PIN, OUTPUT);
  pinMode(WATER_IN4_PIN, OUTPUT);
  stopAllPumps();

  lcd.init();
  lcd.backlight();
  lcd.print("PPM Controller v13");
  lcd.setCursor(0, 1);
  lcd.print("PLOTTER MODE");
  delay(2000);
  
  currentPPM = readTDS();
  nextActionTime = millis();
  currentState = STATE_READING;
}

void loop() {
  if (millis() < nextActionTime) {
    return;
  }
  
  // --- Timer is ready, process the next state ---

  if (currentState == STATE_READING) {
    // --- STATE 1: READING ---
    // Pumps have been off for MIXING_DELAY.
    // The solution is mixed and noise is gone.
    
    // 1. READ the sensor
    currentPPM = readTDS();
  
    // 2. PLOTTER OUTPUT
    Serial.print(SETPOINT_PPM);
    Serial.print(" ");
    Serial.println(currentPPM, 0);
    
    // 3. DECIDE what to do
    controlLogic();
    
    // 4. Set timer for the *pump to stop*
    nextActionTime = millis() + PUMP_DURATION;
    currentState = STATE_PUMPING;
  }
  else if (currentState == STATE_PUMPING) {
    // --- STATE 2: PUMPING ---
    // The pump has run for PUMP_DURATION. Time to stop.
    
    // 1. STOP all pumps
    stopAllPumps();
    
    // 2. Set timer for the *mixing to finish*
    nextActionTime = millis() + MIXING_DELAY;
    currentState = STATE_READING;
  }
}

float readTDS() {
  int sensorRaw = analogRead(TDS_PIN);
  float sensorVoltage = sensorRaw * (5.0 / 1023.0);
  float tdsValue = (133.42 * sensorVoltage * sensorVoltage * sensorVoltage - 255.86 * sensorVoltage * sensorVoltage + 857.39 * sensorVoltage) * 0.5;
  if (tdsValue < 0) { tdsValue = 0; }
  return tdsValue;
}

void controlLogic() {
  int error = SETPOINT_PPM - (int)currentPPM;
  
  // Case 1: We are in the deadband
  if (abs(error) <= DEADBAND_PPM) {
    integralError = 0; // Reset Integral
    
    if (timeEnteredDeadband == 0) {
      timeEnteredDeadband = millis(); 
    }

    if (millis() - timeEnteredDeadband >= STABILIZATION_TIME) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Target Stable!");
      lcd.setCursor(0, 1);
      lcd.print("S:");
      lcd.print(SETPOINT_PPM);
      lcd.print(" C:");
      lcd.print(currentPPM, 0);
    } else {
      long remainingTime = (STABILIZATION_TIME - (millis() - timeEnteredDeadband)) / 1000;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Stabilizing...");
      lcd.setCursor(0, 1);
      lcd.print(remainingTime);
      lcd.print("s left C:");
      lcd.print(currentPPM, 0);
    }
    // No pumps are run
  }
  // Case 2 & 3: We are OUTSIDE the deadband. Run the PI Controller.
  else {
    timeEnteredDeadband = 0; 
    
    // --- THIS IS THE PI LOGIC ---
    integralError += error;
    integralError = constrain(integralError, -INTEGRAL_MAX, INTEGRAL_MAX);
    float pi_output = (Kp * error) + (Ki * integralError);
    pi_output = constrain(pi_output, -255, 255);
    // ----------------------------
    
    if (pi_output > 0) { // PPM is too low
      pumpSpeed = map(pi_output, 0, 255, MIN_PUMP_SPEED, MAX_PUMP_SPEED);
      runBrinePump(pumpSpeed); // This *starts* the pump
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("S:");
      lcd.print(SETPOINT_PPM);
      lcd.print(" C:");
      lcd.print(currentPPM, 0);
      lcd.print(" ^"); 
      lcd.setCursor(0, 1);
      lcd.print("P:S Spd:");
      lcd.print(pumpSpeed);
    }
    else if (pi_output < 0) { // PPM is too high
      pumpSpeed = map(abs(pi_output), 0, 255, MIN_PUMP_SPEED, MAX_PUMP_SPEED);
      runWaterPump(pumpSpeed); // This *starts* the pump
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("S:");
      lcd.print(SETPOINT_PPM);
      lcd.print(" C:");
      lcd.print(currentPPM, 0);
      lcd.print(" v"); 
      lcd.setCursor(0, 1);
      lcd.print("P:W Spd:");
      lcd.print(pumpSpeed);
    }
  }
}

// --- Low-Level Motor Control Functions ---
// These functions ONLY start the pumps.
// The main loop now handles stopping them.

void runBrinePump(int speed) {
  stopWaterPump(); 
  analogWrite(BRINE_SPEED_PIN, speed);
  digitalWrite(BRINE_IN1_PIN, HIGH);
  digitalWrite(BRINE_IN2_PIN, LOW);
}

void runWaterPump(int speed) {
  stopBrinePump(); 
  analogWrite(WATER_SPEED_PIN, speed);
  digitalWrite(WATER_IN3_PIN, HIGH);
  digitalWrite(WATER_IN4_PIN, LOW);
}

void stopBrinePump() {
  analogWrite(BRINE_SPEED_PIN, 0);
}

void stopWaterPump() {
  analogWrite(WATER_SPEED_PIN, 0);
}

void stopAllPumps() {
  stopBrinePump();
  stopWaterPump();
}