//DUAL CHANNEL A & B MPPC CONTROLLER

#include <Wire.h>
#include "AD524X.h"

// INA219 Configuration for Channel A
const uint8_t INA219_ADDR_A1 = 0x40;         // Channel A Input (A0=GND, A1=GND)
const uint8_t INA219_ADDR_A2 = 0x41;         // Channel A Output (A0=VDD, A1=GND)
// INA219 Configuration for Channel B  
const uint8_t INA219_ADDR_B1 = 0x44;         // Channel B Input (A0=GND, A1=GND)
const uint8_t INA219_ADDR_B2 = 0x45;         // Channel B Output (A0=VDD, A1=GND)

const uint16_t INA219_CONFIG = 0x05DF;      // Configuration register value
const uint16_t INA219_CALIBRATION = 0x2000; // Calibration register value

// LSB (Least Significant Bit) constants for measurements
const float CURRENT_LSB = 0.000050;        // 50µA per bit
const float POWER_LSB = 20 * CURRENT_LSB;  // ~2mW per bit
const float V_SHUNT_LSB = 0.000010;        // 10µV per bit
const float V_BUS_LSB = 0.004;             // 4mV per bit

// AD5280 Digital Potentiometers
AD5280 AD01_A(0x2C);  // Channel A - AD0 & AD1 == GND
AD5280 AD01_B(0x2E);  // Channel B - AD0 & AD1 == GND

// Global variables for Channel A
float mppcValueA = 1.0;
float powerInA = 0.0;
float powerOutA = 0.0;
float efficiencyA = 0.0;
float lastEfficiencyA = 0.0;
float stepSizeA = 0.05;
int directionA = 1;
unsigned long lastAdjustmentA = 0;
float currentSourceVoltageA = 0.0;
bool mppcEnabledA = false;

// Global variables for Channel B
float mppcValueB = 1.0;
float powerInB = 0.0;
float powerOutB = 0.0;
float efficiencyB = 0.0;
float lastEfficiencyB = 0.0;
float stepSizeB = 0.05;
int directionB = 1;
unsigned long lastAdjustmentB = 0;
float currentSourceVoltageB = 0.0;
bool mppcEnabledB = false;

const unsigned long adjustmentInterval = 500; // Adjust every 500ms

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
void hillClimbMPPC_A();
void hillClimbMPPC_B();

// ─────────────────────────── Setup ───────────────────────────

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz for all devices

  // Initialize Channel A AD5280 digital potentiometer
  bool adConnectedA = AD01_A.begin();
  Serial.print(F("Channel A AD5280 connected: "));
  Serial.println(adConnectedA ? F("true") : F("false"));
  
  // Initialize Channel B AD5280 digital potentiometer
  bool adConnectedB = AD01_B.begin();
  Serial.print(F("Channel B AD5280 connected: "));
  Serial.println(adConnectedB ? F("true") : F("false"));

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

  Serial.println(F("All INA219 sensors initialized"));
  Serial.println(F("All I2C devices ready"));
}

// ─────────────────────────── Main Loop ───────────────────────────

void loop() {
  // ===== CHANNEL A =====
  // Read Channel A input measurements from sensor A1
  if (isConversionReady(INA219_ADDR_A1)) {
    float sourceVoltageA = readBusVoltage(INA219_ADDR_A1);
    currentSourceVoltageA = sourceVoltageA;
    float currentInA = readCurrent(INA219_ADDR_A1);
    powerInA = readPower(INA219_ADDR_A1);
    
    // Check if MPPC should be enabled/disabled based on source voltage
    if (currentSourceVoltageA >= 0.3) {
      mppcEnabledA = true;
    } else {
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
    efficiencyA = powerOutA/powerInA * 100.0;
    
    Serial.print("boost_voltage_a=");
    Serial.print(boostVoltageA, 2);
    Serial.print(",power_out_a=");
    Serial.print(powerOutA * 1000, 2); // Convert to mW
    Serial.print(",current_out_a=");
    Serial.print(currentOutA * 1000, 2); // Convert to mA
    Serial.print(",efficiency_a=");
    Serial.print(efficiencyA, 6);
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
      mppcEnabledB = true;
    } else {
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
    efficiencyB = powerOutB/powerInB * 100.0;
    
    Serial.print("boost_voltage_b=");
    Serial.print(boostVoltageB, 2);
    Serial.print(",power_out_b=");
    Serial.print(powerOutB * 1000, 2); // Convert to mW
    Serial.print(",current_out_b=");
    Serial.print(currentOutB * 1000, 2); // Convert to mA
    Serial.print(",efficiency_b=");
    Serial.print(efficiencyB, 6);
    Serial.println();
    
    // Hill climbing algorithm - only run if enabled and timing is right
    if (mppcEnabledB && millis() - lastAdjustmentB > adjustmentInterval) {
      hillClimbMPPC_B();
      lastAdjustmentB = millis();
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

/* Set Channel A MPPC voltage by adjusting digital potentiometer */
void setMPPC_A(float targetVoltage) {
  // Clamp input to valid range
  if (targetVoltage < 0.0) targetVoltage = 0.0;
  if (targetVoltage > 2.0) targetVoltage = 2.0;
  
  // Map 0-2.0V range to 0-256 range
  float mapped = targetVoltage * 256.0 / 2.0;
  int potValue = (int)mapped;
  
  // Ensure we don't exceed the AD5280's maximum value
  if (potValue > 255) potValue = 255;
  
  // Write to Channel A digital potentiometer
  AD01_A.write(potValue);
}

/* Set Channel B MPPC voltage by adjusting digital potentiometer */
void setMPPC_B(float targetVoltage) {
  // Clamp input to valid range
  if (targetVoltage < 0.0) targetVoltage = 0.0;
  if (targetVoltage > 2.0) targetVoltage = 2.0;
  
  // Map 0-2.0V range to 0-256 range
  float mapped = targetVoltage * 256.0 / 2.0;
  int potValue = (int)mapped;
  
  // Ensure we don't exceed the AD5280's maximum value
  if (potValue > 255) potValue = 255;
  
  // Write to Channel B digital potentiometer
  AD01_B.write(potValue);
}

/* Channel A Hill Climbing Algorithm */
void hillClimbMPPC_A() {
  // Calculate maximum allowed MPPC value (sourceVoltage - 0.2V)
  float maxAllowedMPPC = currentSourceVoltageA - 0.2;
  
  // Check if efficiency improved from last adjustment
  if (efficiencyA > lastEfficiencyA) {
    // Keep going in the same direction
    mppcValueA += directionA * stepSizeA;
  } else {
    // Efficiency decreased, reverse direction
    directionA *= -1;
    mppcValueA += directionA * stepSizeA;
  }
  
  // Constrain mppcValue to valid range with voltage constraint
  mppcValueA = constrain(mppcValueA, 0.0, min(maxAllowedMPPC, 2.0));
  
  // If we hit the voltage constraint, reverse direction for next iteration
  if (mppcValueA >= maxAllowedMPPC) {
    directionA = -1;  // Force downward direction
  }
  
  // Apply the new MPPC value
  setMPPC_A(mppcValueA);
  
  // Store current efficiency for next comparison
  lastEfficiencyA = efficiencyA;
}

/* Channel B Hill Climbing Algorithm */
void hillClimbMPPC_B() {
  // Calculate maximum allowed MPPC value (sourceVoltage - 0.2V)
  float maxAllowedMPPC = currentSourceVoltageB - 0.2;
  
  // Check if efficiency improved from last adjustment
  if (efficiencyB > lastEfficiencyB) {
    // Keep going in the same direction
    mppcValueB += directionB * stepSizeB;
  } else {
    // Efficiency decreased, reverse direction
    directionB *= -1;
    mppcValueB += directionB * stepSizeB;
  }
  
  // Constrain mppcValue to valid range with voltage constraint
  mppcValueB = constrain(mppcValueB, 0.0, min(maxAllowedMPPC, 2.0));
  
  // If we hit the voltage constraint, reverse direction for next iteration
  if (mppcValueB >= maxAllowedMPPC) {
    directionB = -1;  // Force downward direction
  }
  
  // Apply the new MPPC value
  setMPPC_B(mppcValueB);
  
  // Store current efficiency for next comparison
  lastEfficiencyB = efficiencyB;
}