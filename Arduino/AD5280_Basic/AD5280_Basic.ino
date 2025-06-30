//
//    FILE: AD5280_write_ESP32.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: AD524X demo program
//     URL: https://github.com/RobTillaart/AD524X


#include "AD524X.h"

AD5280 AD01(0x2F);  //  AD0 & AD1 == GND

void setup()
{
  Serial.begin(115200);

  Wire.begin();  //  adjust if needed
  Wire.setClock(400000);

  bool b = AD01.begin();
  Serial.println(b ? "true" : "false");
  Serial.println(AD01.isConnected());
}


void loop()
{
  for (int val = 0; val < 255; val++)
  {
    AD01.write(val);
    if (val == 200)
    {
      AD01.write(val);
    }
    if (val == 0)
    {
      AD01.write(val);
    }
    Serial.println(val);
    delay(20);
  }
}


//  -- END OF FILE --
