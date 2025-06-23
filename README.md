# Joule Thief

**Purpose:** Project Demo at Open Sauce July 18-20, 2025

**Description:** High efficiency battery draining system with diagnostics and energy storage

**Domain:** Low-power/precision power electronics

**Objectives:**
- Extract all usable electrical energy from used/"dead" Alkaline Cells (AA, AAA, etc.)
- Measure & Log energy extraction data from each cell & deliver analytics
- Efficiently convert and push energy extracted into a Li-Ion power bank

**Topology**
- **INA219** - Energy Metering
- **LTC3105** - Low Voltage Boost Converter
- **AD5280** - Digital Potentiometer, 200kΩ
- **DW01** - 18650 Single-cell BMS
- **AO3400** - Low Voltage N-E-FET

**Status:** Currently working on implementing parallel extraction architecture
