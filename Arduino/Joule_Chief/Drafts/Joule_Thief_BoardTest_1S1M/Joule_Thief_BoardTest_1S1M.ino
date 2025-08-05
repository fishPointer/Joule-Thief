//TRIPLE CHANNEL A, B & C MPPC CONTROLLER - SIMPLIFIED

#include <Wire.h>
#include "AD524X.h"

// INA219 Configuration for Channel A
const uint8_t INA219_ADDR_A1 = 0x40;         // Channel A Input (A0=GND, A1=GND)
const uint8_t INA219_ADDR_A2 = 0x44;         // Channel A Output (A0=VDD, A1=GND)
// INA219 Configuration for Channel B  
const uint8_t INA219_ADDR_B1 = 0x4E;         // Channel B Input (A0=GND, A1=GND)
const uint8_t INA219_ADDR_B2 = 0x4F;         // Channel B Output (A0=VDD, A1=GND)
// // INA219 Configuration for Channel C
// const uint8_t INA219_ADDR_C1 = 0x48;         // Channel C Input (A0=GND, A1=VDD)
// const uint8_t INA219_ADDR_C2 = 0x49;         // Channel C Output (A0=VDD, A1=VDD)

const uint16_t INA219_CONFIG = 0x05DF;      // Configuration register value
const uint16_t INA219_CALIBRATION = 0x2000; // Calibration register value

// LSB (Least Significant Bit) constants for measurements
const float CURRENT_LSB = 0.000050;        // 50µA per bit
const float POWER_LSB = 20 * CURRENT_LSB;  // ~2mW per bit
const float V_SHUNT_LSB = 0.000010;        // 10µV per bit
const float V_BUS_LSB = 0.004;             // 4mV per bit

// AD5280 Digital Potentiometers
AD5280 AD01_A(0x2C);  // Channel A - AD0 & AD1 == GND
AD5280 AD01_B(0x2F);  // Channel B - AD0 & AD1 == GND
// AD5280 AD01_C(0x2E);  // Channel C - AD0 & AD1 == GND

// Analog and Digital I/O Pin Definitions
const int ANALOG_INPUT_PIN = A0;     // Analog input pin for supercap monitoring
const int DIGITAL_OUTPUT_PIN = 5;   // Chief Booster pin
const int DIGITAL_OUTPUT_PIN_2 = 6; // Output to Charger pin
const int DIGITAL_OUTPUT_PIN_3 = 13; // SHDN state of charger channel1

// Schmitt Trigger Parameters
const float SCHMITT_UPPER_THRESHOLD = 5;    // Upper threshold voltage
const float SCHMITT_LOWER_THRESHOLD = 4;    // Lower threshold voltage
const float SCHMITT_HYSTERESIS = 0;           // Hysteresis value
bool schmittState = false;                     // Current state of Schmitt trigger output

// Second Schmitt Trigger Parameters for boost voltage monitoring
const float SCHMITT_UPPER_THRESHOLD_2 = 5.25; // Upper threshold voltage for boost
const float SCHMITT_LOWER_THRESHOLD_2 = 4.25; // Lower threshold voltage for boost
const float SCHMITT_HYSTERESIS_2 = 0;         // Hysteresis value for boost trigger
bool schmittState2 = false;                   // Current state of second Schmitt trigger output

// Global variables for Channel A
int mppcValueA = 128;  // Default to middle value (roughly 1.0V for 2V range)
float powerInA = 0.0;
float powerOutA = 0.0;
float efficiencyA = 0.0;
float currentSourceVoltageA = 0.0;
bool mppcEnabledA = false;

// Global variables for Channel B
int mppcValueB = 128;  // Default to middle value (roughly 1.0V for 2V range)
float powerInB = 0.0;
float powerOutB = 0.0;
float efficiencyB = 0.0;
float currentSourceVoltageB = 0.0;
bool mppcEnabledB = false;

// Global variables for Channel C
int mppcValueC = 128;   // Default to roughly 1.0V for 5V range (51/255 * 5V ≈ 1V)
float powerInC = 0.0;
float powerOutC = 0.0;
float efficiencyC = 0.0;
float currentSourceVoltageC = 0.0;
bool mppcEnabledC = false;

// Hill-climbing MPPT variables for Channel B
float previousPowerOutB = 0.0;         // Previous power output for comparison
int mpptStepSizeB = 5;                 // Step size for MPPT adjustment
int mpptDirectionB = 1;                // Direction of last step (1 or -1)
bool mpptInitializedB = false;         // Flag to track if MPPT has been initialized

// Hill-climbing MPPT variables for Channel C
float previousPowerOutC = 0.0;         // Previous power output for comparison
int mpptStepSizeC = 5;                 // Step size for MPPT adjustment
int mpptDirectionC = 1;                // Direction of last step (1 or -1)
bool mpptInitializedC = false;         // Flag to track if MPPT has been initialized

// ────────────────────────── Function Declarations ──────────────────────────

uint16_t readReg16(uint8_t addr, uint8_t reg);
void writeReg16(uint8_t addr, uint8_t reg, uint16_t value);
bool isConversionReady(uint8_t addr);
float readBusVoltage(uint8_t addr);
float readShuntVoltage(uint8_t addr);
float readCurrent(uint8_t addr);
float readPower(uint8_t addr);
void dumpRegisters(uint8_t addr);
void setMPPC_A(int potValue);
void setMPPC_B(int potValue);
void setMPPC_C(int potValue);
void updateMPPT_B();  // New function for Channel B MPPT
void updateMPPT_C();  // New function for Channel C MPPT

// ─────────────────────────── Setup ───────────────────────────

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz for all devices

  // Initialize digital I/O pins
  pinMode(DIGITAL_OUTPUT_PIN, OUTPUT);
  digitalWrite(DIGITAL_OUTPUT_PIN, LOW);  // Initialize digital output to LOW
  pinMode(DIGITAL_OUTPUT_PIN_2, OUTPUT);
  digitalWrite(DIGITAL_OUTPUT_PIN_2, LOW);  // Initialize second digital output to LOW
  pinMode(DIGITAL_OUTPUT_PIN_3, OUTPUT);
  digitalWrite(DIGITAL_OUTPUT_PIN_3, HIGH);  // Initialize second digital output to LOW

  // Initialize Channel A AD5280 digital potentiometer
  bool adConnectedA = AD01_A.begin();
  Serial.print(F("Channel A AD5280 connected: "));
  Serial.println(adConnectedA ? F("true") : F("false"));
  
  // Initialize Channel B AD5280 digital potentiometer
  bool adConnectedB = AD01_B.begin();
  Serial.print(F("Channel B AD5280 connected: "));
  Serial.println(adConnectedB ? F("true") : F("false"));

  // // Initialize Channel C AD5280 digital potentiometer
  // bool adConnectedC = AD01_C.begin();
  // Serial.print(F("Channel C AD5280 connected: "));
  // Serial.println(adConnectedC ? F("true") : F("false"));

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

  // // Configure Channel C INA219 sensors
  // writeReg16(INA219_ADDR_C1, 0x00, INA219_CONFIG);      // Config register
  // writeReg16(INA219_ADDR_C1, 0x05, INA219_CALIBRATION); // Calibration register
  // writeReg16(INA219_ADDR_C2, 0x00, INA219_CONFIG);      // Config register
  // writeReg16(INA219_ADDR_C2, 0x05, INA219_CALIBRATION); // Calibration register

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
      // MPPC disabled - reset to default value
      if (mppcEnabledA) {  // Only reset if transitioning from enabled to disabled
        mppcValueA = 128;  // Reset to middle value
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
    Serial.print(mppcValueA);
    Serial.print(",mppc_enabled_a=");
    Serial.print(mppcEnabledA ? 1 : 0);
    Serial.print(",schmitt_1=");
    Serial.print(schmittState ? 1: 0);
    Serial.print(",schmitt_2=");
    Serial.print(schmittState2 ? 1: 0);
    Serial.println();

    // ===== SUPERCAP READING =====
    float supercap_a = sourceVoltageA;
    
    Serial.print("supercap_a=");
    Serial.print(supercap_a, 2);
    Serial.println();
    
    // ===== SCHMITT TRIGGER DIGITAL OUTPUT CONTROL =====
    if (schmittState == false) {
      // Currently LOW, check if we should go HIGH
      if (supercap_a >= (SCHMITT_UPPER_THRESHOLD - SCHMITT_HYSTERESIS)) {
        schmittState = true;
        digitalWrite(DIGITAL_OUTPUT_PIN, HIGH);
      }
    } else {
      // Currently HIGH, check if we should go LOW  
      if (supercap_a <= (SCHMITT_LOWER_THRESHOLD + SCHMITT_HYSTERESIS)) {
        schmittState = false;
        digitalWrite(DIGITAL_OUTPUT_PIN, LOW);
      }
    }
  }
  
  // Read Channel A output measurements from sensor A2
  if (isConversionReady(INA219_ADDR_A2)) {
    float boostVoltageA = readBusVoltage(INA219_ADDR_A2);
    float currentOutA = readCurrent(INA219_ADDR_A2);
    powerOutA = readPower(INA219_ADDR_A2);
    
    // Calculate efficiency
    if (powerInA > 0) {
      efficiencyA = (powerOutA / powerInA) * 100.0;
    }
    
    Serial.print("boost_voltage_a=");
    Serial.print(boostVoltageA, 2);
    Serial.print(",power_out_a=");
    Serial.print(powerOutA * 1000, 2); // Convert to mW
    Serial.print(",current_out_a=");
    Serial.print(currentOutA * 1000, 2); // Convert to mA
    Serial.print(",efficiency_a=");
    Serial.print(efficiencyA, 6);
    Serial.println();

    // ===== SECOND SCHMITT TRIGGER FOR BOOST VOLTAGE =====
    if (schmittState2 == false) {
      // Currently LOW, check if we should go HIGH
      if (boostVoltageA >= (SCHMITT_UPPER_THRESHOLD_2 - SCHMITT_HYSTERESIS_2)) {
        schmittState2 = true;
        digitalWrite(DIGITAL_OUTPUT_PIN_2, HIGH);
        // digitalWrite(DIGITAL_OUTPUT_PIN, LOW);
      }
    } else {
      // Currently HIGH, check if we should go LOW  
      if (boostVoltageA <= (SCHMITT_LOWER_THRESHOLD_2 + SCHMITT_HYSTERESIS_2)) {
        schmittState2 = false;
        digitalWrite(DIGITAL_OUTPUT_PIN_2, LOW);
        // digitalWrite(DIGITAL_OUTPUT_PIN, HIGH);
      }
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
      // MPPC disabled - reset to default value
      if (mppcEnabledB) {  // Only reset if transitioning from enabled to disabled
        mppcValueB = 128;  // Reset to middle value
        setMPPC_B(mppcValueB);
        mpptInitializedB = false;  // Reset MPPT initialization
      }
      mppcEnabledB = false;
    }
    
    Serial.print("source_voltage_b=");
    Serial.print(sourceVoltageB, 2);
    Serial.print(",power_in_b=");
    Serial.print(powerInB * 1000, 2);  // Convert to mW
    Serial.print(",current_in_b=");
    Serial.print(currentInB * -1000, 2); // Convert to mA
    Serial.print(",mppc_b=");
    Serial.print(mppcValueB);
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
    }
    
    Serial.print("boost_voltage_b=");
    Serial.print(boostVoltageB, 2);
    Serial.print(",power_out_b=");
    Serial.print(powerOutB * 1000, 2); // Convert to mW
    Serial.print(",current_out_b=");
    Serial.print(currentOutB * -1000, 2); // Convert to mA
    Serial.print(",efficiency_b=");
    Serial.print(efficiencyB, 6);
    Serial.println();

    // ===== CHANNEL B HILL-CLIMBING MPPT =====
    updateMPPT_B();
  }

  // // ===== CHANNEL C =====
  // // Read Channel C input measurements from sensor C1
  // if (isConversionReady(INA219_ADDR_C1)) {
  //   float sourceVoltageC = readBusVoltage(INA219_ADDR_C1);
  //   currentSourceVoltageC = sourceVoltageC;
  //   float currentInC = readCurrent(INA219_ADDR_C1);
  //   powerInC = readPower(INA219_ADDR_C1);
    
  //   // Check if MPPC should be enabled/disabled based on source voltage
  //   if (currentSourceVoltageC >= 0.05) {
  //     mppcEnabledC = true;
  //   } else {
  //     // MPPC disabled - reset to default value
  //     if (mppcEnabledC) {  // Only reset if transitioning from enabled to disabled
  //       mppcValueC = 128;   // Reset to roughly 1V equivalent
  //       setMPPC_C(mppcValueC);
  //       mpptInitializedC = false;  // Reset MPPT initialization
  //     }
  //     mppcEnabledC = false;
  //   }
    
  //   Serial.print("source_voltage_c=");
  //   Serial.print(sourceVoltageC, 2);
  //   Serial.print(",power_in_c=");
  //   Serial.print(powerInC * 1000, 2);  // Convert to mW
  //   Serial.print(",current_in_c=");
  //   Serial.print(currentInC * 1000, 2); // Convert to mA
  //   Serial.print(",mppc_c=");
  //   Serial.print(mppcValueC);
  //   Serial.print(",mppc_enabled_c=");
  //   Serial.print(mppcEnabledC ? 1 : 0);
  //   Serial.println();
  // }
  
  // // Read Channel C output measurements from sensor C2
  // if (isConversionReady(INA219_ADDR_C2)) {
  //   float boostVoltageC = readBusVoltage(INA219_ADDR_C2);
  //   float currentOutC = readCurrent(INA219_ADDR_C2);
  //   powerOutC = readPower(INA219_ADDR_C2);
    
  //   // Calculate efficiency
  //   if (powerInC > 0) {
  //     efficiencyC = (powerOutC / powerInC) * 100.0;
  //   }
    
  //   Serial.print("boost_voltage_c=");
  //   Serial.print(boostVoltageC, 2);
  //   Serial.print(",power_out_c=");
  //   Serial.print(powerOutC * 1000, 2); // Convert to mW
  //   Serial.print(",current_out_c=");
  //   Serial.print(currentOutC * 1000, 2); // Convert to mA
  //   Serial.print(",efficiency_c=");
  //   Serial.print(efficiencyC, 6);
  //   Serial.println();

  //   // ===== CHANNEL C HILL-CLIMBING MPPT =====
  //   updateMPPT_C();
  // }
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

/* Set Channel A MPPC by writing digital potentiometer value (0-255) */
void setMPPC_A(int potValue) {
  // Clamp input to valid range (0-255)
  if (potValue < 0) potValue = 0;
  if (potValue > 255) potValue = 255;
  
  // Write to Channel A digital potentiometer
  AD01_A.write(potValue);
}

/* Set Channel B MPPC by writing digital potentiometer value (0-255) */
void setMPPC_B(int potValue) {
  // Clamp input to valid range (0-255)
  if (potValue < 0) potValue = 0;
  if (potValue > 255) potValue = 255;
  
  // Write to Channel B digital potentiometer
  AD01_B.write(potValue);
}

/* Hill-climbing MPPT algorithm for Channel B */
void updateMPPT_B() {
  // Only run MPPT if Channel B is enabled
  if (!mppcEnabledB) {
    return;
  }
  
  // Initialize on first run
  if (!mpptInitializedB) {
    previousPowerOutB = powerOutB;
    mpptInitializedB = true;
    return;
  }
  
  // Calculate power change
  float powerChange = powerOutB - previousPowerOutB;
  
  // Hill-climbing algorithm
  if (powerChange > 0) {
    // Power increased, continue in the same direction
    // Keep the same direction as last step
  } else if (powerChange < 0) {
    // Power decreased, reverse direction
    mpptDirectionB = -mpptDirectionB;
  } else {
    // Power unchanged, try a small perturbation in current direction
    // Keep current direction
  }
  
  // Calculate new MPPC value
  int newMppcValue = mppcValueB + (mpptStepSizeB * mpptDirectionB);
  
  // Clamp to valid range (0-255)
  if (newMppcValue < 0) {
    newMppcValue = 0;
    mpptDirectionB = 1;  // Reverse direction when hitting boundary
  } else if (newMppcValue > 255) {
    newMppcValue = 255;
    mpptDirectionB = -1; // Reverse direction when hitting boundary
  }
  
  // Update MPPC value and apply it
  mppcValueB = newMppcValue;
  setMPPC_B(mppcValueB);
  
  // Store current power for next iteration
  previousPowerOutB = powerOutB;
}

// /* Set Channel C MPPC by writing digital potentiometer value (0-255) */
// void setMPPC_C(int potValue) {
//   // Clamp input to valid range (0-255)
//   if (potValue < 0) potValue = 0;
//   if (potValue > 255) potValue = 255;
  
//   // Write to Channel C digital potentiometer
//   AD01_C.write(potValue);
// }

// /* Hill-climbing MPPT algorithm for Channel C */
// void updateMPPT_C() {
//   // Only run MPPT if Channel C is enabled
//   if (!mppcEnabledC) {
//     return;
//   }
  
//   // Initialize on first run
//   if (!mpptInitializedC) {
//     previousPowerOutC = powerOutC;
//     mpptInitializedC = true;
//     return;
//   }
  
//   // Calculate power change
//   float powerChange = powerOutC - previousPowerOutC;
  
//   // Hill-climbing algorithm
//   if (powerChange > 0) {
//     // Power increased, continue in the same direction
//     // Keep the same direction as last step
//   } else if (powerChange < 0) {
//     // Power decreased, reverse direction
//     mpptDirectionC = -mpptDirectionC;
//   } else {
//     // Power unchanged, try a small perturbation in current direction
//     // Keep current direction
//   }
  
//   // Calculate new MPPC value
//   int newMppcValue = mppcValueC + (mpptStepSizeC * mpptDirectionC);
  
//   // Clamp to valid range (0-255)
//   if (newMppcValue < 0) {
//     newMppcValue = 0;
//     mpptDirectionC = 1;  // Reverse direction when hitting boundary
//   } else if (newMppcValue > 255) {
//     newMppcValue = 255;
//     mpptDirectionC = -1; // Reverse direction when hitting boundary
//   }
  
//   // Update MPPC value and apply it
//   mppcValueC = newMppcValue;
//   setMPPC_C(mppcValueC);
  
//   // Store current power for next iteration
//   previousPowerOutC = powerOutC;
// }