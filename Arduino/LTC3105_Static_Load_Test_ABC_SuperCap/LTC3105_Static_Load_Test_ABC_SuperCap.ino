//TRIPLE CHANNEL A, B & C MPPC CONTROLLER - IMPROVED HILL CLIMBING

#include <Wire.h>
#include "AD524X.h"

// INA219 Configuration for Channel A
const uint8_t INA219_ADDR_A1 = 0x40;         // Channel A Input (A0=GND, A1=GND)
const uint8_t INA219_ADDR_A2 = 0x41;         // Channel A Output (A0=VDD, A1=GND)
// INA219 Configuration for Channel B  
const uint8_t INA219_ADDR_B1 = 0x44;         // Channel B Input (A0=GND, A1=GND)
const uint8_t INA219_ADDR_B2 = 0x45;         // Channel B Output (A0=VDD, A1=GND)
// INA219 Configuration for Channel C
const uint8_t INA219_ADDR_C1 = 0x48;         // Channel C Input (A0=GND, A1=VDD)
const uint8_t INA219_ADDR_C2 = 0x49;         // Channel C Output (A0=VDD, A1=VDD)

const uint16_t INA219_CONFIG = 0x05DF;      // Configuration register value
const uint16_t INA219_CALIBRATION = 0x2000; // Calibration register value

// LSB (Least Significant Bit) constants for measurements
const float CURRENT_LSB = 0.000050;        // 50µA per bit
const float POWER_LSB = 20 * CURRENT_LSB;  // ~2mW per bit
const float V_SHUNT_LSB = 0.000010;        // 10µV per bit
const float V_BUS_LSB = 0.004;             // 4mV per bit

// AD5280 Digital Potentiometers
AD5280 AD01_A(0x2C);  // Channel A - AD0 & AD1 == GND
AD5280 AD01_B(0x2D);  // Channel B - AD0 & AD1 == GND
AD5280 AD01_C(0x2E);  // Channel C - AD0 & AD1 == GND

// Analog and Digital I/O Pin Definitions
const int ANALOG_INPUT_PIN = A0;     // Analog input pin for supercap monitoring
const int DIGITAL_OUTPUT_PIN = 13;   // Digital output pin for status indication

// Schmitt Trigger Parameters
const float SCHMITT_UPPER_THRESHOLD = 4.5;    // Upper threshold voltage
const float SCHMITT_LOWER_THRESHOLD = 3.5;    // Lower threshold voltage
const float SCHMITT_HYSTERESIS = 0;        // Hysteresis value
bool schmittState = false;                     // Current state of Schmitt trigger output

// Improved Hill Climbing Parameters
const int EFFICIENCY_HISTORY_SIZE = 3;        // Number of efficiency measurements to average
const float MIN_EFFICIENCY_IMPROVEMENT = 0.5; // Minimum improvement threshold (%)
const float STEP_SIZE_DECAY = 0.8;            // Factor to reduce step size when oscillating
const float STEP_SIZE_INCREASE = 1.2;         // Factor to increase step size when improving
const float MIN_STEP_SIZE = 0.01;             // Minimum step size
const float MAX_STEP_SIZE = 0.1;              // Maximum step size
const int STAGNATION_THRESHOLD = 5;           // Steps without improvement before trying larger steps

// Global variables for Channel A
float mppcValueA = 1.0;
float powerInA = 0.0;
float powerOutA = 0.0;
float efficiencyA = 0.0;
float efficiencyHistoryA[EFFICIENCY_HISTORY_SIZE] = {0.0};
int efficiencyIndexA = 0;
float avgEfficiencyA = 0.0;
float lastAvgEfficiencyA = 0.0;
float stepSizeA = 0.05;
int directionA = 1;
unsigned long lastAdjustmentA = 0;
float currentSourceVoltageA = 0.0;
bool mppcEnabledA = false;
int stagnationCounterA = 0;
float bestEfficiencyA = 0.0;
float bestMppcValueA = 1.0;

// Global variables for Channel B
float mppcValueB = 1.0;
float powerInB = 0.0;
float powerOutB = 0.0;
float efficiencyB = 0.0;
float efficiencyHistoryB[EFFICIENCY_HISTORY_SIZE] = {0.0};
int efficiencyIndexB = 0;
float avgEfficiencyB = 0.0;
float lastAvgEfficiencyB = 0.0;
float stepSizeB = 0.05;
int directionB = 1;
unsigned long lastAdjustmentB = 0;
float currentSourceVoltageB = 0.0;
bool mppcEnabledB = false;
int stagnationCounterB = 0;
float bestEfficiencyB = 0.0;
float bestMppcValueB = 1.0;

// Global variables for Channel C
float mppcValueC = 1.0;
float powerInC = 0.0;
float powerOutC = 0.0;
float efficiencyC = 0.0;
float efficiencyHistoryC[EFFICIENCY_HISTORY_SIZE] = {0.0};
int efficiencyIndexC = 0;
float avgEfficiencyC = 0.0;
float lastAvgEfficiencyC = 0.0;
float stepSizeC = 0.05;
int directionC = 1;
unsigned long lastAdjustmentC = 0;
float currentSourceVoltageC = 0.0;
bool mppcEnabledC = false;
int stagnationCounterC = 0;
float bestEfficiencyC = 0.0;
float bestMppcValueC = 1.0;

// Global MPPC clamp values for Channel A
const float MPPC_MIN_VOLTAGE_A = 0.25;         // Lower clamp for Channel A MPPC voltage
const float MPPC_MAX_VOLTAGE_A = 2.0;         // Upper clamp for Channel A MPPC voltage
const float MPPC_VOLTAGE_MARGIN_A = 0.2;     // Margin below Channel A source voltage

// Global MPPC clamp values for Channel B
const float MPPC_MIN_VOLTAGE_B = 0.25;         // Lower clamp for Channel B MPPC voltage
const float MPPC_MAX_VOLTAGE_B = 2.0;         // Upper clamp for Channel B MPPC voltage
const float MPPC_VOLTAGE_MARGIN_B = 0.2;     // Margin below Channel B source voltage

// Global MPPC clamp values for Channel C
const float MPPC_MIN_VOLTAGE_C = 0.25;         // Lower clamp for Channel C MPPC voltage
const float MPPC_MAX_VOLTAGE_C = 5.0;         // Upper clamp for Channel C MPPC voltage
const float MPPC_VOLTAGE_MARGIN_C = 0.2;     // Margin below Channel C source voltage

const unsigned long adjustmentInterval = 250; // Adjust every 250ms (doubled tick rate)

// ────────────────────────── Function Declarations ──────────────────────────

uint16_t readReg16(uint8_t addr, uint8_t reg);
void writeReg16(uint8_t addr, uint8_t reg, uint16_t value);
bool isConversionReady(uint8_t addr);
float readBusVoltage(uint8_t addr);
float readShuntVoltage(uint8_t addr);
float readCurrent(uint8_t addr);
float readPower(uint8_t addr);
void dumpRegisters(uint8_t addr);
void setMPPC_A(float targetVoltage);
void setMPPC_B(float targetVoltage);
void setMPPC_C(float targetVoltage);
void hillClimbMPPC_A();
void hillClimbMPPC_B();
void hillClimbMPPC_C();
float calculateAverageEfficiency(float* history, int size);
void updateEfficiencyHistory(float* history, int* index, float newValue);

// ─────────────────────────── Setup ───────────────────────────

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz for all devices

  // Initialize analog and digital I/O pins
  pinMode(ANALOG_INPUT_PIN, INPUT);
  pinMode(DIGITAL_OUTPUT_PIN, OUTPUT);
  digitalWrite(DIGITAL_OUTPUT_PIN, LOW);  // Initialize digital output to LOW

  // Initialize Channel A AD5280 digital potentiometer
  bool adConnectedA = AD01_A.begin();
  Serial.print(F("Channel A AD5280 connected: "));
  Serial.println(adConnectedA ? F("true") : F("false"));
  
  // Initialize Channel B AD5280 digital potentiometer
  bool adConnectedB = AD01_B.begin();
  Serial.print(F("Channel B AD5280 connected: "));
  Serial.println(adConnectedB ? F("true") : F("false"));

  // Initialize Channel C AD5280 digital potentiometer
  bool adConnectedC = AD01_C.begin();
  Serial.print(F("Channel C AD5280 connected: "));
  Serial.println(adConnectedC ? F("true") : F("false"));

  // Configure Channel A INA219 sensors
  writeReg16(INA219_ADDR_A1, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_A1, 0x05, INA219_CALIBRATION); // Calibration register
  writeReg16(INA219_ADDR_A2, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_A2, 0x05, INA219_CALIBRATION); // Calibration register

  // Configure Channel B INA219 sensors
  writeReg16(INA219_ADDR_B1, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_B1, 0x05, INA219_CALIBRATION); // Calibration register
  writeReg16(INA219_ADDR_B2, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_B2, 0x05, INA219_CALIBRATION); // Calibration register

  // Configure Channel C INA219 sensors
  writeReg16(INA219_ADDR_C1, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_C1, 0x05, INA219_CALIBRATION); // Calibration register
  writeReg16(INA219_ADDR_C2, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_C2, 0x05, INA219_CALIBRATION); // Calibration register

  Serial.println(F("All INA219 sensors initialized"));
  Serial.println(F("All I2C devices ready"));
}

// ─────────────────────────── Main Loop ───────────────────────────

void loop() {
  // ===== ANALOG INPUT READING =====
  int analogValue = analogRead(ANALOG_INPUT_PIN);
  float supercap_a = analogValue * (5.0 / 1023.0);  // Convert to voltage (assuming 5V reference)
  
  Serial.print("supercap_a=");
  Serial.print(supercap_a, 2);
  Serial.println();
  
  // ===== SCHMITT TRIGGER DIGITAL OUTPUT CONTROL =====
  // Schmitt trigger with bounds around 3.5V and 4.5V with 0.25V hysteresis
  if (schmittState == false) {
    // Currently LOW, check if we should go HIGH
    if (supercap_a >= (SCHMITT_UPPER_THRESHOLD - SCHMITT_HYSTERESIS)) {  // 4.25V
      schmittState = true;
      digitalWrite(DIGITAL_OUTPUT_PIN, HIGH);
    }
  } else {
    // Currently HIGH, check if we should go LOW  
    if (supercap_a <= (SCHMITT_LOWER_THRESHOLD + SCHMITT_HYSTERESIS)) {  // 3.75V
      schmittState = false;
      digitalWrite(DIGITAL_OUTPUT_PIN, LOW);
    }
  }

  // ===== CHANNEL A =====
  // Read Channel A input measurements from sensor A1
  if (isConversionReady(INA219_ADDR_A1)) {
    float sourceVoltageA = readBusVoltage(INA219_ADDR_A1);
    currentSourceVoltageA = sourceVoltageA;
    float currentInA = readCurrent(INA219_ADDR_A1);
    powerInA = readPower(INA219_ADDR_A1);
    
    // Check if MPPC should be enabled/disabled based on source voltage
    if (currentSourceVoltageA >= 0.3) {
      if (!mppcEnabledA) {
        // Just enabled - reset tracking variables
        stagnationCounterA = 0;
        bestEfficiencyA = 0.0;
        bestMppcValueA = mppcValueA;
        // Clear efficiency history
        for (int i = 0; i < EFFICIENCY_HISTORY_SIZE; i++) {
          efficiencyHistoryA[i] = 0.0;
        }
        efficiencyIndexA = 0;
      }
      mppcEnabledA = true;
    } else {
      // MPPC disabled - reset to 1.0V
      if (mppcEnabledA) {  // Only reset if transitioning from enabled to disabled
        mppcValueA = 1.0;
        setMPPC_A(mppcValueA);
      }
      mppcEnabledA = false;
    }
    
    Serial.print("source_voltage_a=");
    Serial.print(sourceVoltageA, 2);
    Serial.print(",power_in_a=");
    Serial.print(powerInA * 1000, 2);  // Convert to mW
    Serial.print(",current_in_a=");
    Serial.print(currentInA * 1000, 2); // Convert to mA
    Serial.print(",mppc_a=");
    Serial.print(mppcValueA, 2);
    Serial.print(",mppc_enabled_a=");
    Serial.print(mppcEnabledA ? 1 : 0);
    Serial.println();
  }
  
  // Read Channel A output measurements from sensor A2
  if (isConversionReady(INA219_ADDR_A2)) {
    float boostVoltageA = readBusVoltage(INA219_ADDR_A2);
    float currentOutA = readCurrent(INA219_ADDR_A2);
    powerOutA = readPower(INA219_ADDR_A2);
    
    // Calculate efficiency
    if (powerInA > 0) {
      efficiencyA = (powerOutA / powerInA) * 100.0;
      
      // Update efficiency history
      updateEfficiencyHistory(efficiencyHistoryA, &efficiencyIndexA, efficiencyA);
      avgEfficiencyA = calculateAverageEfficiency(efficiencyHistoryA, EFFICIENCY_HISTORY_SIZE);
    }
    
    Serial.print("boost_voltage_a=");
    Serial.print(boostVoltageA, 2);
    Serial.print(",power_out_a=");
    Serial.print(powerOutA * 1000, 2); // Convert to mW
    Serial.print(",current_out_a=");
    Serial.print(currentOutA * 1000, 2); // Convert to mA
    Serial.print(",efficiency_a=");
    Serial.print(efficiencyA, 6);
    Serial.print(",avg_efficiency_a=");
    Serial.print(avgEfficiencyA, 6);
    Serial.println();
    
    // Hill climbing algorithm - only run if enabled and timing is right
    if (mppcEnabledA && millis() - lastAdjustmentA > adjustmentInterval) {
      hillClimbMPPC_A();
      lastAdjustmentA = millis();
    }
  }

  // ===== CHANNEL B =====
  // Read Channel B input measurements from sensor B1
  if (isConversionReady(INA219_ADDR_B1)) {
    float sourceVoltageB = readBusVoltage(INA219_ADDR_B1);
    currentSourceVoltageB = sourceVoltageB;
    float currentInB = readCurrent(INA219_ADDR_B1);
    powerInB = readPower(INA219_ADDR_B1);
    
    // Check if MPPC should be enabled/disabled based on source voltage
    if (currentSourceVoltageB >= 0.3) {
      if (!mppcEnabledB) {
        // Just enabled - reset tracking variables
        stagnationCounterB = 0;
        bestEfficiencyB = 0.0;
        bestMppcValueB = mppcValueB;
        // Clear efficiency history
        for (int i = 0; i < EFFICIENCY_HISTORY_SIZE; i++) {
          efficiencyHistoryB[i] = 0.0;
        }
        efficiencyIndexB = 0;
      }
      mppcEnabledB = true;
    } else {
      // MPPC disabled - reset to 1.0V
      if (mppcEnabledB) {  // Only reset if transitioning from enabled to disabled
        mppcValueB = 1.0;
        setMPPC_B(mppcValueB);
      }
      mppcEnabledB = false;
    }
    
    Serial.print("source_voltage_b=");
    Serial.print(sourceVoltageB, 2);
    Serial.print(",power_in_b=");
    Serial.print(powerInB * 1000, 2);  // Convert to mW
    Serial.print(",current_in_b=");
    Serial.print(currentInB * 1000, 2); // Convert to mA
    Serial.print(",mppc_b=");
    Serial.print(mppcValueB, 2);
    Serial.print(",mppc_enabled_b=");
    Serial.print(mppcEnabledB ? 1 : 0);
    Serial.println();
  }
  
  // Read Channel B output measurements from sensor B2
  if (isConversionReady(INA219_ADDR_B2)) {
    float boostVoltageB = readBusVoltage(INA219_ADDR_B2);
    float currentOutB = readCurrent(INA219_ADDR_B2);
    powerOutB = readPower(INA219_ADDR_B2);
    
    // Calculate efficiency
    if (powerInB > 0) {
      efficiencyB = (powerOutB / powerInB) * 100.0;
      
      // Update efficiency history
      updateEfficiencyHistory(efficiencyHistoryB, &efficiencyIndexB, efficiencyB);
      avgEfficiencyB = calculateAverageEfficiency(efficiencyHistoryB, EFFICIENCY_HISTORY_SIZE);
    }
    
    Serial.print("boost_voltage_b=");
    Serial.print(boostVoltageB, 2);
    Serial.print(",power_out_b=");
    Serial.print(powerOutB * 1000, 2); // Convert to mW
    Serial.print(",current_out_b=");
    Serial.print(currentOutB * 1000, 2); // Convert to mA
    Serial.print(",efficiency_b=");
    Serial.print(efficiencyB, 6);
    Serial.print(",avg_efficiency_b=");
    Serial.print(avgEfficiencyB, 6);
    Serial.println();
    
    // Hill climbing algorithm - only run if enabled and timing is right
    if (mppcEnabledB && millis() - lastAdjustmentB > adjustmentInterval) {
      hillClimbMPPC_B();
      lastAdjustmentB = millis();
    }
  }

  // ===== CHANNEL C =====
  // Read Channel C input measurements from sensor C1
  if (isConversionReady(INA219_ADDR_C1)) {
    float sourceVoltageC = readBusVoltage(INA219_ADDR_C1);
    currentSourceVoltageC = sourceVoltageC;
    float currentInC = readCurrent(INA219_ADDR_C1);
    powerInC = readPower(INA219_ADDR_C1);
    
    // Check if MPPC should be enabled/disabled based on source voltage
    if (currentSourceVoltageC >= 0.3) {
      if (!mppcEnabledC) {
        // Just enabled - reset tracking variables
        stagnationCounterC = 0;
        bestEfficiencyC = 0.0;
        bestMppcValueC = mppcValueC;
        // Clear efficiency history
        for (int i = 0; i < EFFICIENCY_HISTORY_SIZE; i++) {
          efficiencyHistoryC[i] = 0.0;
        }
        efficiencyIndexC = 0;
      }
      mppcEnabledC = true;
    } else {
      // MPPC disabled - reset to 1.0V
      if (mppcEnabledC) {  // Only reset if transitioning from enabled to disabled
        mppcValueC = 1.0;
        setMPPC_C(mppcValueC);
      }
      mppcEnabledC = false;
    }
    
    Serial.print("source_voltage_c=");
    Serial.print(sourceVoltageC, 2);
    Serial.print(",power_in_c=");
    Serial.print(powerInC * 1000, 2);  // Convert to mW
    Serial.print(",current_in_c=");
    Serial.print(currentInC * 1000, 2); // Convert to mA
    Serial.print(",mppc_c=");
    Serial.print(mppcValueC, 2);
    Serial.print(",mppc_enabled_c=");
    Serial.print(mppcEnabledC ? 1 : 0);
    Serial.println();
  }
  
  // Read Channel C output measurements from sensor C2
  if (isConversionReady(INA219_ADDR_C2)) {
    float boostVoltageC = readBusVoltage(INA219_ADDR_C2);
    float currentOutC = readCurrent(INA219_ADDR_C2);
    powerOutC = readPower(INA219_ADDR_C2);
    
    // Calculate efficiency
    if (powerInC > 0) {
      efficiencyC = (powerOutC / powerInC) * 100.0;
      
      // Update efficiency history
      updateEfficiencyHistory(efficiencyHistoryC, &efficiencyIndexC, efficiencyC);
      avgEfficiencyC = calculateAverageEfficiency(efficiencyHistoryC, EFFICIENCY_HISTORY_SIZE);
    }
    
    Serial.print("boost_voltage_c=");
    Serial.print(boostVoltageC, 2);
    Serial.print(",power_out_c=");
    Serial.print(powerOutC * 1000, 2); // Convert to mW
    Serial.print(",current_out_c=");
    Serial.print(currentOutC * 1000, 2); // Convert to mA
    Serial.print(",efficiency_c=");
    Serial.print(efficiencyC, 6);
    Serial.print(",avg_efficiency_c=");
    Serial.print(avgEfficiencyC, 6);
    Serial.println();
    
    // Hill climbing algorithm - only run if enabled and timing is right
    if (mppcEnabledC && millis() - lastAdjustmentC > adjustmentInterval) {
      hillClimbMPPC_C();
      lastAdjustmentC = millis();
    }
  }
}

// ────────────────────────── Function Definitions ──────────────────────────

/* Read 16-bit register from INA219 */
uint16_t readReg16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);              // Repeated start

  Wire.requestFrom(addr, (uint8_t)2);
  while (Wire.available() < 2);             // Wait for data

  uint16_t val = (uint16_t)Wire.read() << 8; // MSB
  val |= Wire.read();                        // LSB
  return val;
}

/* Write 16-bit value to INA219 register */
void writeReg16(uint8_t addr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));        // MSB
  Wire.write((uint8_t)(value & 0xFF));      // LSB
  Wire.endTransmission();
}

/* Check if conversion is ready */
bool isConversionReady(uint8_t addr) {
  uint16_t rawBus = readReg16(addr, 0x02);  // Bus voltage register
  return rawBus & 0x0002;                   // Bit 1: Conversion Ready Flag
}

/* Read bus voltage in Volts */
float readBusVoltage(uint8_t addr) {
  uint16_t rawBus = readReg16(addr, 0x02);
  uint16_t busData = rawBus >> 3;           // Ignore 3 LSBs
  return busData * V_BUS_LSB;
}

/* Read shunt voltage in Volts */
float readShuntVoltage(uint8_t addr) {
  uint16_t rawShunt = readReg16(addr, 0x01);
  int16_t signedShunt = (int16_t)rawShunt;  // Handle two's complement
  return signedShunt * V_SHUNT_LSB;
}

/* Read current in Amps */
float readCurrent(uint8_t addr) {
  uint16_t rawCurrent = readReg16(addr, 0x04);
  int16_t signedCurrent = (int16_t)rawCurrent; // Handle two's complement
  return signedCurrent * CURRENT_LSB;
}

/* Read power in Watts */
float readPower(uint8_t addr) {
  uint16_t rawPower = readReg16(addr, 0x03);
  return rawPower * POWER_LSB;
}

/* Optional: Debug function to dump all registers */
void dumpRegisters(uint8_t addr) {
  uint16_t cfg  = readReg16(addr, 0x00);
  uint16_t shnt = readReg16(addr, 0x01);
  uint16_t bus  = readReg16(addr, 0x02);
  uint16_t pwr  = readReg16(addr, 0x03);
  uint16_t cur  = readReg16(addr, 0x04);
  uint16_t cal  = readReg16(addr, 0x05);

  Serial.print(F("INA219 Registers - CFG:0x"));
  Serial.print(cfg, HEX);
  Serial.print(F(" SHUNT:0x"));
  Serial.print(shnt, HEX);
  Serial.print(F(" BUS:0x"));
  Serial.print(bus, HEX);
  Serial.print(F(" PWR:0x"));
  Serial.print(pwr, HEX);
  Serial.print(F(" CUR:0x"));
  Serial.print(cur, HEX);
  Serial.print(F(" CAL:0x"));
  Serial.print(cal, HEX);
  Serial.println();
}

/* Update efficiency history with new value */
void updateEfficiencyHistory(float* history, int* index, float newValue) {
  history[*index] = newValue;
  *index = (*index + 1) % EFFICIENCY_HISTORY_SIZE;
}

/* Calculate average efficiency from history */
float calculateAverageEfficiency(float* history, int size) {
  float sum = 0.0;
  int count = 0;
  
  for (int i = 0; i < size; i++) {
    if (history[i] > 0.0) {  // Only count valid readings
      sum += history[i];
      count++;
    }
  }
  
  return (count > 0) ? (sum / count) : 0.0;
}

/* Set Channel A MPPC voltage by adjusting digital potentiometer */
void setMPPC_A(float targetVoltage) {
  // Clamp input to valid range using Channel A constants
  if (targetVoltage < MPPC_MIN_VOLTAGE_A) targetVoltage = MPPC_MIN_VOLTAGE_A;
  if (targetVoltage > MPPC_MAX_VOLTAGE_A) targetVoltage = MPPC_MAX_VOLTAGE_A;
  
  // Map 0-2.0V range to 0-256 range
  float mapped = targetVoltage * 256.0 / MPPC_MAX_VOLTAGE_A;
  int potValue = (int)mapped;
  
  // Ensure we don't exceed the AD5280's maximum value
  if (potValue > 255) potValue = 255;
  
  // Write to Channel A digital potentiometer
  AD01_A.write(potValue);
}

/* Set Channel B MPPC voltage by adjusting digital potentiometer */
void setMPPC_B(float targetVoltage) {
  // Clamp input to valid range using Channel B constants
  if (targetVoltage < MPPC_MIN_VOLTAGE_B) targetVoltage = MPPC_MIN_VOLTAGE_B;
  if (targetVoltage > MPPC_MAX_VOLTAGE_B) targetVoltage = MPPC_MAX_VOLTAGE_B;
  
  // Map 0-2.0V range to 0-256 range
  float mapped = targetVoltage * 256.0 / MPPC_MAX_VOLTAGE_B;
  int potValue = (int)mapped;
  
  // Ensure we don't exceed the AD5280's maximum value
  if (potValue > 255) potValue = 255;
  
  // Write to Channel B digital potentiometer
  AD01_B.write(potValue);
}

/* Set Channel C MPPC voltage by adjusting digital potentiometer */
void setMPPC_C(float targetV