<img width="2023" height="1139" alt="Pasted image 20250805165937" src="https://github.com/user-attachments/assets/19df0c38-cbfb-4dc0-b03c-34496b627569" />

# Joule Thief

## Overview
- **Purpose:** Exhibition project at Open Sauce July 18-20, 2025, Practice
- **Description:** High efficiency alkaline battery draining system with diagnostics and energy storage
- **Domain:** Precision energy harvesting & instrumentation

**Objectives:**
- Extract all usable electrical energy from used/"dead" Alkaline Cells (AA, AAA, etc.)
- Measure & Log energy extraction data from each cell & deliver analytics
- Efficiently convert and push energy extracted into a Li-Ion power bank

**Topology**
- **INA219** - Energy Metering
- **LTC3105** - Low Voltage Boost Converter
- **AD5280** - Digital Potentiometer, 200kΩ
- **DW01** - 18650 Single-cell BMS
- **TP4056** - Single-cell 18650 Charger
- **FS8205** - Dual-FET package for DW01
- **AO3400** - Low Voltage N-E-FET
- **1N5817** - Low Voltage Schottky Diode

**Status:** Complete. Exhibited. Working on Finalizing Documentation.

**Additional Documentation:** available on my Obsidian at [suneaterlabs.com](https://suneaterlabs.com).

**Why Joule Thief?**
This project was started as a New Year's Resolution to exhibit at Open Sauce 2025, and was finished 2 days prior to the event.
I have no commercial intent with this project. I don't believe that it is particularly novel, and a close look by anyone experienced with energy harvesting & power electronics should rightly reveal it as something akin to a useless machine.

I teach myself new skills by designing projects that require to develop the skills I want in order to complete them. I put a high level of effort into this project because I wanted to go through the process of designing a relatively performant embedded system without referencing anyone else's work. Just pick functionalities and features, set a spec, and then flip through datasheets until you find the right combination. Then put it all together and prove that it works. Design the schematic, route a board, write the firmware, etc. and then package it all up and provide thorough documentation on the whole thing.

I had never done any of that before and I have no professional experience working as an electronics engineer. I do have an EE degree and currently work in power system analysis for hyperscale datacenters, so this project is actually something like a microcosm of my day job. There is little overlap though, and I consider myself self-taught here. I learned a lot!




## System Description

### Joule Thief - Core Functionality
<img width="3723" height="1935" alt="Joule Thief Schematic" src="https://github.com/user-attachments/assets/adaf917b-5191-41b6-ad15-f664e4fb7f68" />
The Joule Thief is designed to fully deplete alkaline cells of any remaining energy, long after they will fail to turn on their devices. The core of this design is the LTC3105, which can handle input voltages as low as 250mV. A quick look at an alkaline cell's SOC-OCV curve will show that draining to 300mV is sufficient to say >99% of total energy has been drained.

The classic Joule Thief is a simple party trick involving a dead battery, a hand-wound toroid, and a BJT. It is essentially using the voltage spike from inductive kickback to PWM an LED back to life. My motivation for this project was to demonstrate that there is a considerable amount of energy left in alkaline cells when they are discarded, so I set out to build an overengineered energy harvester that could precisely quantify how much energy was actually left in a dead cell, and then store it into a lithium-based battery bank to further prove the point.

 In order to improve performance and maximize both energy throughput and conversion efficiency, there is an INA219 shunt meter on either side of the LTC3105 boost chip allowing for efficiency tracking during processing & live diagnostics. 
 Additionally, the LTC3105 includes an MPPC pin, which enables a variable resistance to throttle the inductor's switching in order to achieve a specified input voltage. With the addition of an AD5280-200kΩ digital potentiometer attached to the MPPC pin, it is possible to read in energy measurements from the INA219's into a microcontroller, and then adjust the setting of the LTC3105 accordingly. The result is a control loop that can maximize the energy extraction rate regardless of the target cell's health or state of charge. 
 
 This INA219+LTC3105+AD5280 system can be tuned for maximum efficiency or maximum average power extracted, but optimizing for energy throughput has far greater performance gains. Using a DC Electronic Load to produce a slow sweep of load resistance values across a fresh AA alkaline cell will result in no more than 400-500mW peak extraction rate, and chemical effects such as electrolyte gradient formation and ion depletion make this power output unstable and liable to rapidly decline. The dynamic approach explored in the Joule Thief is able to achieve 900-1100mW peak output power, and can reliably maintain 600-700mW extraction rate in the early life of a cell. Additionally, the MPPC schema prevents the cell's output capacity from being overwhelmed by the battery chemistry effects, and the cell will continuously output higher power than traditional Joule Thief circuits or static loads.

The Joule Thief Extraction Module is also designed to run in "headless" mode without an MCU. No diagnostics are available in this mode (obviously) and the board will be able to move the energy from a single AA cell into a single 18650 cell safely.




### Joule Chief - Core Functionality
<img width="3733" height="1946" alt="Joule Chief Schematic" src="https://github.com/user-attachments/assets/ef424ffd-7532-478e-89b7-e94d4fb1f291" />
The Joule Chief was designed to be a mainboard that allows for the consolidation of harvested energy from an arbitrary number of Joule Thief Extraction Modules collecting energy from different sources at different rates. The Joule Chief serves as both an aggregator and an abstraction layer for the energy harvested. 

The Joule Chief begins with a low-voltage schottky diode on the output of the extraction module to prevent backflow. After the energy is delivered to the mainboard's input bus, all the energy is stored in a 1F supercapacitor. The Joule Thief Extraction Modules are all set to boost whatever energy source they are targeting to 5V and charge the 1F supercap to 5V accordingly. 

Readings from the mainboard's INA219 allow the MCU to precisely track the voltage of the supercap as well as the power flowing into this buffer. The MCU is programmed to operate a schmitt trigger with thresholds at 2.5V and 5V, only enabling the 1F supercap's energy to pass through the mainboard's LTC3105 after 5V has been reached, and until 2.5V. This region of input voltage allows the boost chip to operate in a high-efficiency mode.

Once the energy has moved through the 1F supercap and through the LTC3105, it is stored in a second 1F supercap connected to the lithium cell charger. The cell charger is controlled by a second schmitt trigger running on the MCU as well. This schmitt trigger keeps the second 1F energy buffer between 4.5V and 5.0V, such that the buffer will always be an appropriate voltage for charging a lithium cell (3.0V to 4.3V).

The Joule Chief can handle an arbitrary number of extraction modules each working on arbitrarily weak or intermittent energy sources. This mainboard can effectively accumulate these energy sources and efficiently deposit this energy via intermittent pulses of energy into a rechargeable battery bank for later use while providing precise diagnostics at every stage of the harvesting process. The frequency of these charging pulses is dependent on the strength of the sources. Harvesting a sufficiently powerful source, such as a fresh AA cell, will allow the mainboard to achieve steady-state operation, and a continuous current flow into the 18650 becomes possible.

Both the Joule Thief and Joule Chief boards currently use a generic and inefficient BMS stack consisting of a DW01 for OC/OCD/SC protection, an FS8205 dual-FET package for breaking the circuit, and a TP4056 charging IC. This BMS is largely unnecessary given the low-power nature of this application, but was included for safety and peace of mind.


 

