
#include "AD524X.h"

AD5280 AD01(0x2F);  //  AD0 & AD1 == GND
String inputString = "";
bool stringComplete = false;

void setMPPC(float targetVoltage);
void serialEvent();

void setup()
{
  Serial.begin(115200);

  Wire.begin();  //  adjust if needed
  Wire.setClock(400000);

  bool b = AD01.begin();
  Serial.println(b ? "true" : "false");
  Serial.println(AD01.isConnected());
}


// void loop() {
//   if (stringComplete) {
//     float receivedFloat = inputString.toFloat();
//     Serial.print("Received Float: ");
//     Serial.println(receivedFloat);
//     inputString = "";
//     stringComplete = false;

//     setMPPC(receivedFloat);
//   }
// }

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input.length() > 0) {
      setMPPC(input.toFloat());
    }
  }
  // Your other code here
}

// ----------------------------------------------------

// void serialEvent() {
//   while (Serial.available()) {
//     char inChar = (char)Serial.read();
//     inputString += inChar;
//     if (inChar == '\n') {
//       stringComplete = true;
//     }
//   }
// }

void setMPPC(float targetVoltage) {
  // Clamp input to valid range
  if (targetVoltage < 0.0) targetVoltage = 0.0;
  if (targetVoltage > 2.0) targetVoltage = 2.0;
  
  // Map 0-2.0V range to 0-256 range
  float mapped = targetVoltage * 256.0 / 2.0;  // Simplified: targetVoltage * 128.0
  int potValue = (int)mapped;
  
  // Ensure we don't exceed the AD5280's maximum value (typically 255 or 256)
  if (potValue > 255) potValue = 255;
  
  // Write to digital potentiometer
  Serial.print("pot value: ");
  Serial.println(potValue);
  AD01.write(potValue);
}


