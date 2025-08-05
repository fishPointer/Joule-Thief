//Quad Channel Joule Thief - Refactored with Struct Methodology

#include <Wire.h>
#include "AD524X.h"

// ----------- ADDRESSES -----------
// Main Board (Channel A)
const uint8_t INA219_ADDR_A1 = 0x40; 
const uint8_t INA219_ADDR_A2 = 0x44;     
AD5280 AD01_A(0x2C); 
// Module 0 (Channel B)
const uint8_t INA219_ADDR_B1 = 0x4E; 
const uint8_t INA219_ADDR_B2 = 0x4F;  
AD5280 AD01_B(0x2F);  
// Module 1 (Channel C)
const uint8_t INA219_ADDR_C1 = 0x4C; 
const uint8_t INA219_ADDR_C2 = 0x4D;  
AD5280 AD01_C(0x2E);
// Module 2 (Channel D)
const uint8_t INA219_ADDR_D1 = 0x4A; 
const uint8_t INA219_ADDR_D2 = 0x4B;  
AD5280 AD01_D(0x2D);  
// Module 3 (Channel E)
const uint8_t INA219_ADDR_E1 = 0x48; 
const uint8_t INA219_ADDR_E2 = 0x49;  
AD5280 AD01_E(0x2B);

// ----------- CALIBRATION -----------
const uint16_t INA219_CONFIG = 0x05DF;      // Configuration register value
const uint16_t INA219_CALIBRATION = 0x2000; // Calibration register value

// LSB (Least Significant Bit) constants for measurements
const float CURRENT_LSB = 0.000050;        // 50µA per bit
const float POWER_LSB = 20 * CURRENT_LSB;  // ~2mW per bit
const float V_SHUNT_LSB = 0.000010;        // 10µV per bit
const float V_BUS_LSB = 0.004;             // 4mV per bit

// Booster Buffer Schmitt Trigger Parameters
const float SCHMITT_UPPER_THRESHOLD = 5;    // Upper threshold voltage
const float SCHMITT_LOWER_THRESHOLD = 4;    // Lower threshold voltage
bool schmittState = false;                  // Current state of Schmitt trigger output

// Charger Buffer Schmitt Trigger Parameters
const float SCHMITT_UPPER_THRESHOLD_2 = 5.25; // Upper threshold voltage for boost
const float SCHMITT_LOWER_THRESHOLD_2 = 4.25; // Lower threshold voltage for boost
bool schmittState2 = false;                    // Current state of second Schmitt trigger output

// ----------- DECLARATIONS -----------
// Analog and Digital I/O Pin Definitions
const int BOOST_PIN = 5;    // Chief Booster Enable HIGH
const int CHARGE_PIN = 6;   // Chief Charger Enable HIGH
const int SHDN_0_PIN = 13;  // Pull LOW to SHDN Module 0
const int SHDN_1_PIN = 12;  // Pull LOW to SHDN Module 1
const int SHDN_2_PIN = 11;  // Pull LOW to SHDN Module 2
const int SHDN_3_PIN = 10;  // Pull LOW to SHDN Module 3

// ----------- CHANNEL DATA STRUCTURE -----------
struct ChannelData {
  uint8_t inputSensorAddr;   // Input sensor address
  uint8_t outputSensorAddr;  // Output sensor address
  AD5280* digitalPot;        // Pointer to digital potentiometer
  float currentSourceVoltage;
  float powerIn;
  float powerOut;
  float efficiency;
  bool mppcEnabled;
  uint8_t mppcValue;
  bool mpptInitialized;
  char channelId;
  // MPPT variables
  float previousPowerOut;
  int mpptStepSize;
  int mpptDirection;
};

// ----------- GLOBAL VARIABLES -----------
// Channel A special variables (kept separate due to unique behavior)
int mppcValueA = 128;
float powerInA = 0.0;
float powerOutA = 0.0;
float efficiencyA = 0.0;
float currentSourceVoltageA = 0.0;
bool mppcEnabledA = false;
float previousPowerOutA = 0.0;  // Changed from previousEfficiencyA
int mpptStepSizeA = 5;
int mpptDirectionA = 1;
bool mpptInitializedA = false;

// Array of channels B through E
ChannelData channels[4];

// ----------- FUNCTION DECLARATIONS -----------
uint16_t readReg16(uint8_t addr, uint8_t reg);
void writeReg16(uint8_t addr, uint8_t reg, uint16_t value);
bool isConversionReady(uint8_t addr);
float readBusVoltage(uint8_t addr);
float readShuntVoltage(uint8_t addr);
float readCurrent(uint8_t addr);
float readPower(uint8_t addr);
void dumpRegisters(uint8_t addr);
void setMPPC_A(int potValue);
void updateMPPT_A();
void processChannelInput(ChannelData& channel);
void processChannelOutput(ChannelData& channel);
void setChannelMPPC(ChannelData& channel, uint8_t value);
void updateChannelMPPT(ChannelData& channel);
void initializeChannels();

// ----------- Setup -----------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz for all devices

  // Initialize digital I/O pins
  pinMode(BOOST_PIN, OUTPUT);
  digitalWrite(BOOST_PIN, LOW);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW);
  pinMode(SHDN_0_PIN, OUTPUT);
  digitalWrite(SHDN_0_PIN, HIGH);
  pinMode(SHDN_1_PIN, OUTPUT);
  digitalWrite(SHDN_1_PIN, HIGH);
  pinMode(SHDN_2_PIN, OUTPUT);
  digitalWrite(SHDN_2_PIN, HIGH);
  pinMode(SHDN_3_PIN, OUTPUT);
  digitalWrite(SHDN_3_PIN, HIGH);

  // Initialize channels array
  initializeChannels();

  // ----------- AD5280 INIT -----------
  // Initialize Channel A AD5280 digital potentiometer
  bool adConnectedA = AD01_A.begin();
  Serial.print(F("Channel A AD5280 connected: "));
  Serial.println(adConnectedA ? F("true") : F("false"));
  
  // Initialize channels B-E digital potentiometers
  for (int i = 0; i < 4; i++) {
    bool adConnected = channels[i].digitalPot->begin();
    Serial.print(F("Channel "));
    Serial.print(channels[i].channelId);
    Serial.print(F(" AD5280 connected: "));
    Serial.println(adConnected ? F("true") : F("false"));
  }

  // ----------- INA219 INIT -----------
  // Configure Channel A INA219 sensors
  writeReg16(INA219_ADDR_A1, 0x00, INA219_CONFIG);
  writeReg16(INA219_ADDR_A1, 0x05, INA219_CALIBRATION);
  writeReg16(INA219_ADDR_A2, 0x00, INA219_CONFIG);
  writeReg16(INA219_ADDR_A2, 0x05, INA219_CALIBRATION);
  
  // Configure channels B-E INA219 sensors
  for (int i = 0; i < 4; i++) {
    writeReg16(channels[i].inputSensorAddr, 0x00, INA219_CONFIG);
    writeReg16(channels[i].inputSensorAddr, 0x05, INA219_CALIBRATION);
    writeReg16(channels[i].outputSensorAddr, 0x00, INA219_CONFIG);
    writeReg16(channels[i].outputSensorAddr, 0x05, INA219_CALIBRATION);
  }

  Serial.println(F("All INA219 sensors initialized"));
  Serial.println(F("All I2C devices ready"));
}

// Initialize channels array
void initializeChannels() {
  // Channel B (index 0)
  channels[0].inputSensorAddr = INA219_ADDR_B1;
  channels[0].outputSensorAddr = INA219_ADDR_B2;
  channels[0].digitalPot = &AD01_B;
  channels[0].currentSourceVoltage = 0.0;
  channels[0].powerIn = 0.0;
  channels[0].powerOut = 0.0;
  channels[0].efficiency = 0.0;
  channels[0].mppcEnabled = false;
  channels[0].mppcValue = 128;
  channels[0].mpptInitialized = false;
  channels[0].channelId = 'B';
  channels[0].previousPowerOut = 0.0;
  channels[0].mpptStepSize = 5;
  channels[0].mpptDirection = 1;
  
  // Channel C (index 1)
  channels[1].inputSensorAddr = INA219_ADDR_C1;
  channels[1].outputSensorAddr = INA219_ADDR_C2;
  channels[1].digitalPot = &AD01_C;
  channels[1].currentSourceVoltage = 0.0;
  channels[1].powerIn = 0.0;
  channels[1].powerOut = 0.0;
  channels[1].efficiency = 0.0;
  channels[1].mppcEnabled = false;
  channels[1].mppcValue = 128;
  channels[1].mpptInitialized = false;
  channels[1].channelId = 'C';
  channels[1].previousPowerOut = 0.0;
  channels[1].mpptStepSize = 5;
  channels[1].mpptDirection = 1;
  
  // Channel D (index 2)
  channels[2].inputSensorAddr = INA219_ADDR_D1;
  channels[2].outputSensorAddr = INA219_ADDR_D2;
  channels[2].digitalPot = &AD01_D;
  channels[2].currentSourceVoltage = 0.0;
  channels[2].powerIn = 0.0;
  channels[2].powerOut = 0.0;
  channels[2].efficiency = 0.0;
  channels[2].mppcEnabled = false;
  channels[2].mppcValue = 128;
  channels[2].mpptInitialized = false;
  channels[2].channelId = 'D';
  channels[2].previousPowerOut = 0.0;
  channels[2].mpptStepSize = 5;
  channels[2].mpptDirection = 1;
  
  // Channel E (index 3)
  channels[3].inputSensorAddr = INA219_ADDR_E1;
  channels[3].outputSensorAddr = INA219_ADDR_E2;
  channels[3].digitalPot = &AD01_E;
  channels[3].currentSourceVoltage = 0.0;
  channels[3].powerIn = 0.0;
  channels[3].powerOut = 0.0;
  channels[3].efficiency = 0.0;
  channels[3].mppcEnabled = false;
  channels[3].mppcValue = 128;
  channels[3].mpptInitialized = false;
  channels[3].channelId = 'E';
  channels[3].previousPowerOut = 0.0;
  channels[3].mpptStepSize = 5;
  channels[3].mpptDirection = 1;
}

// ─────────────────────────── Main Loop ───────────────────────────
void loop() {
  // ===== CHANNEL A (Special handling) =====
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
      if (mppcEnabledA) {
        mppcValueA = 128;
        setMPPC_A(mppcValueA);
        mpptInitializedA = false;
      }
      mppcEnabledA = false;
    }
    
    Serial.print("source_voltage_a=");
    Serial.print(sourceVoltageA, 2);
    Serial.print(",power_in_a=");
    Serial.print(powerInA * 1000, 2);
    Serial.print(",current_in_a=");
    Serial.print(currentInA * -1000, 2);
    Serial.print(",mppc_a=");
    Serial.print(mppcValueA);
    Serial.print(",mppc_enabled_a=");
    Serial.print(mppcEnabledA ? 1 : 0);
    Serial.print(",schmitt_1=");
    Serial.print(schmittState ? 1: 0);
    Serial.print(",schmitt_2=");
    Serial.print(schmittState2 ? 1: 0);
    Serial.println();

    // ===== SCHMITT TRIGGER DIGITAL OUTPUT CONTROL =====
    if (schmittState == false) {
      if (sourceVoltageA >= SCHMITT_UPPER_THRESHOLD) {
        schmittState = true;
        digitalWrite(BOOST_PIN, HIGH);
      }
    } else {
      if (sourceVoltageA <= SCHMITT_LOWER_THRESHOLD) {
        schmittState = false;
        digitalWrite(BOOST_PIN, LOW);
      }
    }
  }

  // Read Channel A output measurements from sensor A2
  if (isConversionReady(INA219_ADDR_A2)) {
    float boostVoltageA = readBusVoltage(INA219_ADDR_A2);
    float currentOutA = readCurrent(INA219_ADDR_A2);
    powerOutA = readPower(INA219_ADDR_A2);
    
    if (powerInA > 0) {
      efficiencyA = (powerOutA / powerInA) * 100.0;
    }
    
    Serial.print("boost_voltage_a=");
    Serial.print(boostVoltageA, 2);
    Serial.print(",power_out_a=");
    Serial.print(powerOutA * 1000, 2);
    Serial.print(",current_out_a=");
    Serial.print(currentOutA * -1000, 2);
    Serial.print(",efficiency_a=");
    Serial.print(efficiencyA, 6);
    Serial.println();

    updateMPPT_A();

    // ===== SECOND SCHMITT TRIGGER FOR BOOST VOLTAGE =====
    if (schmittState2 == false) {
      if (boostVoltageA >= SCHMITT_UPPER_THRESHOLD_2) {
        schmittState2 = true;
        digitalWrite(CHARGE_PIN, HIGH);
      }
    } else {
      if (boostVoltageA <= SCHMITT_LOWER_THRESHOLD_2) {
        schmittState2 = false;
        digitalWrite(CHARGE_PIN, LOW);
      }
    }
  }

  // ===== CHANNELS B-E (Struct-based processing) =====
  for (int i = 0; i < 4; i++) {
    processChannelInput(channels[i]);
    processChannelOutput(channels[i]);
  }
}

// Process input sensor for a channel
void processChannelInput(ChannelData& channel) {
  if (isConversionReady(channel.inputSensorAddr)) {
    float sourceVoltage = readBusVoltage(channel.inputSensorAddr);
    channel.currentSourceVoltage = sourceVoltage;
    float currentIn = readCurrent(channel.inputSensorAddr);
    channel.powerIn = readPower(channel.inputSensorAddr);
    
    // Check if MPPC should be enabled/disabled based on source voltage
    if (channel.currentSourceVoltage >= 0.3) {
      channel.mppcEnabled = true;
    } else {
      if (channel.mppcEnabled) {
        channel.mppcValue = 128;
        setChannelMPPC(channel, channel.mppcValue);
        channel.mpptInitialized = false;
      }
      channel.mppcEnabled = false;
    }
    
    // Serial output
    char channelLower = tolower(channel.channelId);
    Serial.print("source_voltage_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(sourceVoltage, 2);
    Serial.print(",power_in_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(channel.powerIn * 1000, 2);
    Serial.print(",current_in_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(currentIn * -1000, 2);
    Serial.print(",mppc_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(channel.mppcValue);
    Serial.print(",mppc_enabled_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(channel.mppcEnabled ? 1 : 0);
    Serial.println();
  }
}

// Process output sensor for a channel
void processChannelOutput(ChannelData& channel) {
  if (isConversionReady(channel.outputSensorAddr)) {
    float boostVoltage = readBusVoltage(channel.outputSensorAddr);
    float currentOut = readCurrent(channel.outputSensorAddr);
    channel.powerOut = readPower(channel.outputSensorAddr);
    
    // Calculate efficiency
    if (channel.powerIn > 0) {
      channel.efficiency = (channel.powerOut / channel.powerIn) * 100.0;
    }
    
    // Serial output
    char channelLower = tolower(channel.channelId);
    Serial.print("boost_voltage_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(boostVoltage, 2);
    Serial.print(",power_out_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(channel.powerOut * 1000, 2);
    Serial.print(",current_out_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(currentOut * -1000, 2);
    Serial.print(",efficiency_");
    Serial.print(channelLower);
    Serial.print("=");
    Serial.print(channel.efficiency, 6);
    Serial.println();

    // Update MPPT
    updateChannelMPPT(channel);
  }
}

// Set MPPC value for specific channel
void setChannelMPPC(ChannelData& channel, uint8_t value) {
  if (value < 0) value = 0;
  if (value > 255) value = 255;
  channel.digitalPot->write(value);
}

// Update MPPT for specific channel
void updateChannelMPPT(ChannelData& channel) {
  // Only run MPPT if channel is enabled
  if (!channel.mppcEnabled) {
    return;
  }
  
  // Initialize on first run
  if (!channel.mpptInitialized) {
    channel.previousPowerOut = channel.powerOut;
    channel.mpptInitialized = true;
    return;
  }
  
  // Calculate power change
  float powerChange = channel.powerOut - channel.previousPowerOut;
  
  // Hill-climbing algorithm
  if (powerChange > 0) {
    // Power increased, continue in the same direction
  } else if (powerChange < 0) {
    // Power decreased, reverse direction
    channel.mpptDirection = -channel.mpptDirection;
  }
  
  // Calculate new MPPC value
  int newMppcValue = channel.mppcValue + (channel.mpptStepSize * channel.mpptDirection);
  
  // Clamp to valid range (0-255)
  if (newMppcValue < 0) {
    newMppcValue = 0;
    channel.mpptDirection = 1;
  } else if (newMppcValue > 255) {
    newMppcValue = 255;
    channel.mpptDirection = -1;
  }
  
  // Update MPPC value and apply it
  channel.mppcValue = newMppcValue;
  setChannelMPPC(channel, channel.mppcValue);
  
  // Store current power for next iteration
  channel.previousPowerOut = channel.powerOut;
}

// ────────────────────────── Original Function Definitions ──────────────────────────

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

/* Set Channel A MPPC by writing digital potentiometer value (0-255) */
void setMPPC_A(int potValue) {
  if (potValue < 0) potValue = 0;
  if (potValue > 255) potValue = 255;
  AD01_A.write(potValue);
}

/* Hill-climbing MPPT algorithm for Channel A - optimized for power output */
void updateMPPT_A() {
  // Only run MPPT if Channel A is enabled
  if (!mppcEnabledA) {
    return;
  }
  
  // Initialize on first run
  if (!mpptInitializedA) {
    previousPowerOutA = powerOutA;
    mpptInitializedA = true;
    return;
  }
  
  // Calculate power change
  float powerChange = powerOutA - previousPowerOutA;
  
  // Hill-climbing algorithm for power output optimization
  if (powerChange > 0) {
    // Power increased, continue in the same direction
  } else if (powerChange < 0) {
    // Power decreased, reverse direction
    mpptDirectionA = -mpptDirectionA;
  }
  
  // Calculate new MPPC value
  int newMppcValue = mppcValueA + (mpptStepSizeA * mpptDirectionA);
  
  // Clamp to valid range (0-255)
  if (newMppcValue < 0) {
    newMppcValue = 0;
    mpptDirectionA = 1;
  } else if (newMppcValue > 255) {
    newMppcValue = 255;
    mpptDirectionA = -1;
  }
  
  // Update MPPC value and apply it
  mppcValueA = newMppcValue;
  setMPPC_A(mppcValueA);
  
  // Store current power output for next iteration
  previousPowerOutA = powerOutA;
}