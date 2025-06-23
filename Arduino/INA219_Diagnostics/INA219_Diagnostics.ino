#include <Wire.h>

// INA219 Configuration
const uint8_t INA219_ADDR_1 = 0x40;         // Default address (A0=GND, A1=GND)
const uint8_t INA219_ADDR_2 = 0x41;         // Alternative address (A0=VDD, A1=GND)
const uint16_t INA219_CONFIG = 0x05DF;      // Configuration register value
const uint16_t INA219_CALIBRATION = 0x2000; // Calibration register value

// LSB (Least Significant Bit) constants for measurements
const float CURRENT_LSB = 0.000050;        // 50µA per bit
const float POWER_LSB = 20 * CURRENT_LSB;  // ~2mW per bit
const float V_SHUNT_LSB = 0.000010;        // 10µV per bit
const float V_BUS_LSB = 0.004;             // 4mV per bit

// ────────────────────────── Function Declarations ──────────────────────────

uint16_t readReg16(uint8_t addr, uint8_t reg);
void writeReg16(uint8_t addr, uint8_t reg, uint16_t value);
bool isConversionReady(uint8_t addr);
float readBusVoltage(uint8_t addr);
float readShuntVoltage(uint8_t addr);
float readCurrent(uint8_t addr);
float readPower(uint8_t addr);
void dumpRegisters(uint8_t addr);

// ─────────────────────────── Setup ───────────────────────────

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configure first INA219
  writeReg16(INA219_ADDR_1, 0x00, INA219_CONFIG);      // Config register
  writeReg16(INA219_ADDR_1, 0x05, INA219_CALIBRATION); // Calibration register

  // Configure second INA219 (uncomment if using second sensor)
  // writeReg16(INA219_ADDR_2, 0x00, INA219_CONFIG);      // Config register
  // writeReg16(INA219_ADDR_2, 0x05, INA219_CALIBRATION); // Calibration register

  Serial.println(F("INA219 initialized"));
}

// ─────────────────────────── Main Loop ───────────────────────────

void loop() {
  if (isConversionReady(INA219_ADDR_1)) {
    // Output: Bus Voltage, Shunt Voltage (mV), Current (mA), Power (mW)
    Serial.print(readBusVoltage(INA219_ADDR_1), 9);
    Serial.print(",");
    Serial.print(1000 * readShuntVoltage(INA219_ADDR_1), 6);
    Serial.print(",");
    Serial.print(1000 * readCurrent(INA219_ADDR_1), 6);
    Serial.print(",");
    Serial.print(1000 * readPower(INA219_ADDR_1), 6);
    Serial.println();
  }

  // Second INA219 readings (uncomment if using second sensor)
  // if (isConversionReady(INA219_ADDR_2)) {
  //   Serial.print("INA2: ");
  //   Serial.print(readBusVoltage(INA219_ADDR_2), 9);
  //   Serial.print(",");
  //   Serial.print(1000 * readShuntVoltage(INA219_ADDR_2), 6);
  //   Serial.print(",");
  //   Serial.print(1000 * readCurrent(INA219_ADDR_2), 6);
  //   Serial.print(",");
  //   Serial.print(1000 * readPower(INA219_ADDR_2), 6);
  //   Serial.println();
  // }
}

// ────────────────────────── Function Implementations ──────────────────────────

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