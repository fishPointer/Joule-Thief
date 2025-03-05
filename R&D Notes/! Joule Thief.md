---
tags:
  - joule-thief
  - drop
---
![[Pasted image 20250127234538.png]]
## Overview
**Description:** High efficiency battery draining system with diagnostics and energy storage 
**Domain:** Low-power/precision power electronics
**Objectives:**
- Extract all usable electrical energy from used/"dead" Alkaline Cells (AA, AAA, etc.)
- Measure & Log energy extraction data from each cell & deliver analytics
- Efficiently convert and push energy extracted into a LiPo power bank

Plan is to demo this at Open Sauce in July.

## Lab Notes
(Last Updated: 2025-02-10)
### Research
1. [[Joule Thief - Initial Research]]
2. [[Joule Thief - Basic Simulation]]
3. [[How to wind a toroid inductor]]
4. [[Joule Thief Research 2]]
5. [[IEC Battery Designations]]
6. [[Joule Thief Topologies]]
7. [[AA Cell Hard Limits]]
8. [[AA Cell Hard Limits 2]]
9. [[Load and ESR Investigations]]
10. [[Fast-Drain vs. Slow-Drain]]
11. [[Note - Maximum Power Transfer Theorem]]
12. [[ESR Meter Research]]
13. [[Quick Voltmeter Circuit]]
14. [[Quick Ammeter Circuit]]
15. [[ESR Measuring Circuit Sketch]]
16. [[Dynamic Load - Low R_on FET + Opamp]]
17. [[LV Boost Converter IC Table]]
18. [[LTC3105 - LV Boost Converter]]
19. [[MCP1640 - Battery Boost Board]]
20. [[BMS Design Series]]
21. [[Table of BMS ICs]]
22. [[LTC4015 - Integrated BMS]]
23. [[LTC4015 Battery Management System I2C Signals]]
24. [[BMS Comparison - LTC4015 vs. LTC6804]]
25. [[Multi Channel LTC3105]]
26. [[Supercapacitor Energy Buffer for Intermittent Pulse Charging]]
27. [[Detailed Supercapacitor Charging System for Pulse Charging]]
28. [[Pulse Charge-Discharge Rough Calculations]]
29. **COURSE:** [[Battery Boot Camp Course]]
30. [[Misc. Joule Thief References]]
31. [[LTSpice LT3105 Deeper Dive]] (new!)
32. [[LTC3105 Better SPICE]] (new!)
33. [[DeepSeek Barebones Discrete BMS]] (new!)
34. [[Some Discrete BMS Builds & Parts]] (new!)
35. [[Digital Pots to Consider]] (new!)
36. [[Cheap MCU Research]] (new!)
37. [[Introduction to the ATTiny85]] (new!)
### Development
1. [[Joule Thief Build 1]]
2. [[Custom Toroid Inductor Design]]
3. **SIM: [[LTSpice - Random Inductance and Battery Values]]**
4. **SIM: [[LTSpice - Schottky Diode and Capacitor]]**
5. [[Actually winding the First Toroid]]
6. **BUILD LOG:** [[Build Log - Breadboarding the Joule Thief]]
7. [[Joule Thief Waveforms 1.canvas|Joule Thief Waveforms 1]]
8. [[GFM - Battery Holder]]
9. **SIM: [[LTSpice - LTC3105]]**
10. [[Joule Thief MVP sketch]]
11. **BUILD LOG:** [[Build Log - MVP Build Day]]
12. [[Joule Thief - Current State]]
13. **SIM:** [[LTSpice - Rechargeable Battery Simulation]] 
14. [[Joule Thief Chips and Bits Purchasing]] 
15. **BUILD LOG:** [[Joule Thief Build Day - Pi Mount and Schmitt Trigger Design]] 
16. [[Chips on the Amazon BMS]] 
17. **SIM:** [[LTSpice - Charge Pulse Energy Delivery]] 
18. [[Testing New Amazon BMS and 18650s]] 
19. [[Power Constraints - How to Kill a Battery]] 
20. [[IRF9540N SPICE Model Generation]] (new!)
21. [[DIY Overdischarge Circuit]] (new!)
22. [[First Lithium Charge Logs]] (new!)
23. [[Alkaline Cell Pulsed Loading Tests]] (new!)
24. [[Old Inductor MPPC Settings + Larry]] (new!)
25. [[Joule Thief Path Review]] (new!)
26. [[New Inductor MPPC Testing + Larry]] (new!)
27. [[18650 MPPC Charging Tests]] (new!)
28. **DATASHEET:** [[INA219.pdf]] (new!)
29. [[INA219 Pop Quiz]] (new!)
30. [[INA219 Annotation]] (new!)

### Maps
[[Joule Thief Maps]] <---
- [[Joule Thief Sketch 0 - Basics.canvas|Joule Thief Sketch 0 - Basics]]
- [[Joule Thief Sketch 1 - Full System.canvas|Joule Thief Sketch 1 - Full System]]
- [[Joule Thief Sketch 2 - Topologies.canvas|Joule Thief Sketch 2 - Topologies]]
- [[Joule Thief Sketch 3 - Expanded System.canvas|Joule Thief Sketch 3 - Expanded System]]
- [[Joule Thief Sketch 4 - Processing.canvas|Joule Thief Sketch 4 - Processing]]
- [[Joule Thief Sketch 5 - Data Flow.canvas|Joule Thief Sketch 5 - Data Flow]]
- [[Joule Thief Sketch 6 - Timeline.canvas|Joule Thief Sketch 6 - Timeline]]
- [[Joule Thief Sketch 7 - Master.canvas|Joule Thief Sketch 7 - Master]]
- [[Joule Thief Sketch 8 - Tyler.canvas|Joule Thief Sketch 8 - Tyler]] 
#### Current Map
![[Joule Thief Sketch 7 - Master 2.png]]


## Misc
### To Do
#### Approx. 90 Day Development Timeline
Phase 1 - MVP - 2 days
Phase 2 - Single-Celled Diagnostics Board - 35 days
Phase 3 - Supercap Pulse Charger & BMS Board - 30 days
Phase 4 - Database and Frontend on Pi - 14 days
Phase 5 - Cell Extraction Array & Extinguish Board - 30 days
Phase 6 - Finalization

Phase 1 Complete by 2/14?
Phase 6 Complete by 5/1?

---

#### List 1
- [x] Introduction of the Schottky and Capacitor Load Buffer
	- [x] Data on the 1N5817, 8, 9
	- [x] GPT of Schottky operating principles and key electrical characteristics
- [x] Research on Darlington or other Low VBE, Low VCE, High Beta BJTs
- [x] Collecting the Ferrite Core Characteristics:
- [x] Introduction to the LD1117
- [x] Introduction to the LT1111
- [x] Introduction to the LT1110
- [x] Introduction to the OnSemi MBRS140 and MBR0520L
- [x] Recap of GPT  - Inductance Winding Calculations
	- [x] Resistance of Winding Calculations
- [x] Recap of GPT - Joule Thief Activation Voltage Explorations
- [x] Exploration of Methodologies for Trickle Charging LiPo Batteries from a 5V/5mA source
	- [x] Slow extraction rate (25mW) means a human could reasonably outperform the thief.
- [x] Varying ESR Characteristics of Alkaline Cells
- [x] Advantages of Tantalum Capacitors
- [ ] LTSpice .meas syntax, data extraction and table creation workflow
#### List 2
- [x] See how many Series LEDs I can drive before this thing gets mad at me
- [x] Find a way to measure the battery voltage and current at the same time
- [x] Set the breadboard up for Test & Measure so I can swap components out
- [x] Derive energy efficiency metrics for different circuit/part configurations
- [x] Order some second stage modules
- [x] Start playing with BMS chips and LiPo Cells, figure out standard charging and trickle charging
- [x] Drain a battery completely and see what happens, log some data
- [x] Connect a few different loads and see what happens
- [x] Start looking into microcontroller-level precision current and voltage metering options
- [ ] Once I have some pipelines for harvesting this data, switch to AAA cells
- [x] See if I can do anything to kill the battery faster
- [x] See if I can do anything to drain the battery as slowly as possible
- [x] Compare total energy extracted and energy efficiency metrics for both extremes
#### List 3
- [x] Review the design with Tyler
- [x] Purchase all the chips in whatever form factor is most convenient
- [x] Purchase any additional parts and tools 
- [x] Find or make SMD to THT breakouts for each of the chips' form factors so I can breadboard the system
- [x] Consult Beuter for what solder/flux/etc. and methods to practice given my equipment
- [x] Research other BMS chipsets and solutions
- [x] Review the BMS boards and designs from previous LiPo battery pack youtube annotations
- [ ] Design & Order Single-celled Diagnostics board ASAP
- [ ] Then, Design & Order supercap pulse charger & BMS board ASAP afterwards
- [ ] Then, in the time between design and build, learn how to SMD solder. Practice.
- [ ] Select and order the Barcode Scanner & Label Printer
- [ ] Commission any artwork for the aesthetics
- [ ] Build out the Single-Celled Diagnostics Board and begin testing
- [ ] Build out the LiPo Battery Bank and begin testing
- [ ] Build out the database and software end of the Pi and begin testing
- [ ] Design and order the Board for the Cell-Extraction Array
- [ ] Design and order the Board for the Cell-Extinguish Module
- [ ] Finalize all of the designs and work on the aesthetics and logistics.

#### List 4
- [x] Test negative voltage, high frequency methods for battery killing
- [x] Learn how to roll my own BMS so I can ensure charging circuit is efficient
- [x] Actually charge an 18650 using an AA using the LTC3105
- [x] Rebuild Breadboard circuit with INA219s and the LTC3105
- [ ] Start setting up Pi's UI/Database app

#### List 5
- [x] Efficiency Testing with 18650
- [ ] Finish KiCAD Schematic
	- [x] Set up KiCAD 9 customizations
	- [x] Read & Annotate INA219 datasheet and set up basic implementation
		- [ ] [[INA219.pdf]]
	- [ ] Confirm symbols & footprints for each part
	- [ ] Organize visually
- [ ] Layout R0 PCB
	- [ ] Decide what "branch" and "test" features I want to include
		- [ ] Dummy Load
		- [ ] Battery Switcher
		- [ ] Mechanical and Electrical Cutoffs
		- [ ] Generic Button, Switches
		- [ ] Addressing Solder Pads
	- [ ] Decide on IO, Battery Holders, Battery Switcher, etc.
	- [ ] Shopping
		- [ ] Inductors
		- [ ] FETs
		- [ ] Schottkys
		- [ ] Switches
		- [ ] Battery Protection ICs
		- [ ] SoC Monitoring
- [ ] Pi UI
	- [ ] Live Update of metrics
	- [ ] Persistent Database
	- [ ] Battery Channel Switching
	- [ ] Possibly a Node-RED/Interface thing