#include <Wire.h>
#include <LiquidCrystal.h>

//Declare RS, E, D4, D5, D6, D7
LiquidCrystal lcd(7,8,9,10,11,12);

void setup() 
{
  Serial.begin(115200);
  
  //Initialize I2C Communication
  Wire.begin();
  
  //Initialize the LCD Display
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.write("TEMP(C): ");

}

void loop() 
{
  //call the sensorRead function to retrieve the temperature
  double temperature = sensorRead();

  //Print the current temperature to the right of the label
  lcd.setCursor(9,0);
  lcd.print(temperature);

  //wait 1/2 second
  delay(200);
}

double sensorRead(void)
{
  //temp holds the two bytes of data read back from the TMP102
  uint8_t temp[2];

  //tempc holds the modified combination of bytes
  int16_t tempc;

  //Point to device 0x48
  Wire.beginTransmission(0x48);
  //Point to register 0x00 (Temperature Register)
  Wire.write(0x00);
  //Relinquish master control of I2C line
  Wire.endTransmission();
  //delay to allow for sufficient conversion time
  delay(10);
  
  //Request temperature data
  Wire.requestFrom(0x48, 2);

    //if the two bytes are sucessfully returned
    if (2 <= Wire.available())
    {       
      //read out the data
      temp[0]  = Wire.read();
      temp[1] = Wire.read();

      //ignore the lower 4 bits of byte 2
      temp[1] = temp[1] >> 4;
      //combine to make one 12 bit binary number
      tempc = ((temp[0] << 4) | temp[1]);

      //Convert to celcius (0.0625C resolution) and return
      return tempc*0.0625;
    }
}


