//SINGLE CHANNEL MPPC CONTROLLER

#include <Wire.h>
#include "AD524X.h"

// INA219 Configuration
const uint8_t INA219_ADDR_1 = 0x4E;         // Input sensor
const uint8_t INA219_ADDR_2 = 0x4F;         // Output sensor

const uint16_t INA219_CONFIG = 0x05DF;      // Configuration register value
const uint16_t INA219_CALIBRATION = 0x2000; // Calibration register value

// LSB (Least Significant Bit) constants for measurements
const float CURRENT_LSB = 0.000050;        // 50µA per bit
const float POWER_LSB = 20 * CURRENT_LSB;  // ~2mW per bit
const float V_SHUNT_LSB = 0.000010;        // 10µV per bit
const float V_BUS_LSB = 0.004;             // 4mV per bit

// AD5280 Digital Potentiometer
AD5280 AD01(0x2F);  // Digital potentiometer

// Global variables
int mppcValue = 128;  // Default to middle value (roughly 1.0V for 2V range)
float powerIn = 0.0;
float powerOut = 0.0;
float efficiency = 0.0;
float currentSourceVoltage = 0.0;
bool mppcEnabled = false;

// Hill-climbing MPPT variables
float previousPowerOut = 0.0;         // Previous power output for comparison
int mpptStepSize = 5;                 // Step size for MPPT adjustment
int mpptDirection = 1;                // Direction of last step (1 or -1)
bool mpptInitialized = false;         // Flag to track if MPPT has been initialized

// ────────────────────────── Function Declarations ──────────────────────────

uint16_t readReg16(uint8_t addr, uint8_t reg);
void writeReg16(uint8_t addr, uint8_t reg, uint16_t value);
bool isConversionReady(uint8_t addr);
float readBusVoltage(uint8_t addr);
float readShuntVoltage(uint8_t addr);
float readCurrent(uint8_t addr);
float readPower(uint8_t addr);
void dumpRegisters(uint8_t addr);
void setMPPC(int potValue);
void updateMPPT();

// ─────────────────────────── Setup ───────────────────────────

void setup() {
  Serial.begin(115200);

  Serial.println("hihi");
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz for all devices

  // Initialize AD5280 digital potentiometer
  bool adConnected = AD01.begin();
  Serial.print(F("AD5280 connected: "));
  Serial.println(adConnected ? F("true") : F("false"));

  // Configure INA219 sensors
  writeReg16(INA219_ADDR_1, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_1, 0x05, INA219_CALIBRATION); // Calibration register
  writeReg16(INA219_ADDR_2, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_2, 0x05, INA219_CALIBRATION); // Calibration register

  Serial.println(F("INA219 sensors initialized"));
  Serial.println(F("I2C devices ready"));
}

// ─────────────────────────── Main Loop ───────────────────────────

void loop() {
  // Read input measurements from sensor 1
  if (isConversionReady(INA219_ADDR_1)) {
    float sourceVoltage = readBusVoltage(INA219_ADDR_1);
    currentSourceVoltage = sourceVoltage;
    float currentIn = readCurrent(INA219_ADDR_1);
    powerIn = readPower(INA219_ADDR_1);
    
    // Check if MPPC should be enabled/disabled based on source voltage
    if (currentSourceVoltage >= 0.3) {
      mppcEnabled = true;
    } else {
      // MPPC disabled - reset to default value
      if (mppcEnabled) {  // Only reset if transitioning from enabled to disabled
        mppcValue = 128;  // Reset to middle value
        setMPPC(mppcValue);
        mpptInitialized = false;  // Reset MPPT initialization
      }
      mppcEnabled = false;
    }
    
    Serial.print("source_voltage=");
    Serial.print(sourceVoltage, 2);
    Serial.print(",power_in=");
    Serial.print(powerIn * 1000, 2);  // Convert to mW
    Serial.print(",current_in=");
    Serial.print(currentIn * 1000, 2); // Convert to mA
    Serial.print(",mppc=");
    Serial.print(mppcValue);
    Serial.print(",mppc_enabled=");
    Serial.print(mppcEnabled ? 1 : 0);
    Serial.println();
  }
  
  // Read output measurements from sensor 2
  if (isConversionReady(INA219_ADDR_2)) {
    float boostVoltage = readBusVoltage(INA219_ADDR_2);
    float currentOut = readCurrent(INA219_ADDR_2);
    powerOut = readPower(INA219_ADDR_2);
    
    // Calculate efficiency
    if (powerIn > 0) {
      efficiency = (powerOut / powerIn) * 100.0;
    }
    
    Serial.print("boost_voltage=");
    Serial.print(boostVoltage, 2);
    Serial.print(",power_out=");
    Serial.print(powerOut * 1000, 2); // Convert to mW
    Serial.print(",current_out=");
    Serial.print(currentOut * 1000, 2); // Convert to mA
    Serial.print(",efficiency=");
    Serial.print(efficiency, 6);
    Serial.println();

    // ===== HILL-CLIMBING MPPT =====
    updateMPPT();
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
  return rawBus & 0x0002;                   // Conversion Ready Flag
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

/* Set MPPC by writing digital potentiometer value (0-255) */
void setMPPC(int potValue) {
  // Clamp input to valid range (0-255)
  if (potValue < 0) potValue = 0;
  if (potValue > 255) potValue = 255;
  
  // Write to digital potentiometer
  AD01.write(potValue);
}

/* Hill-climbing MPPT algorithm */
void updateMPPT() {
  // Only run MPPT if MPPC is enabled
  if (!mppcEnabled) {
    return;
  }
  
  // Initialize on first run
  if (!mpptInitialized) {
    previousPowerOut = powerOut;
    mpptInitialized = true;
    return;
  }
  
  // Calculate power change
  float powerChange = powerOut - previousPowerOut;
  
  // Hill-climbing algorithm
  if (powerChange > 0) {
    // Power increased, continue in the same direction
    // Keep the same direction as last step
  } else if (powerChange < 0) {
    // Power decreased, reverse direction
    mpptDirection = -mpptDirection;
  } else {
    // Power unchanged, try a small perturbation in current direction
    // Keep current direction
  }
  
  // Calculate new MPPC value
  int newMppcValue = mppcValue + (mpptStepSize * mpptDirection);
  
  // Clamp to valid range (0-255)
  if (newMppcValue < 0) {
    newMppcValue = 0;
    mpptDirection = 1;  // Reverse direction when hitting boundary
  } else if (newMppcValue > 255) {
    newMppcValue = 255;
    mpptDirection = -1; // Reverse direction when hitting boundary
  }
  
  // Update MPPC value and apply it
  mppcValue = newMppcValue;
  setMPPC(mppcValue);
  
  // Store current power for next iteration
  previousPowerOut = powerOut;
}