  #include <SparkFunTMP102.h>
  #include <SparkFun_MMA8452Q.h>
  #include <LiquidCrystal.h>
  #include <Wire.h>



  #define MMA8452_ADDRESS 0x1D
  #define OUT_X_MSB 0x01
  #define XYZ_DATA_CFG  0x0E
  #define WHO_AM_I   0x0D
  #define CTRL_REG1  0x2A
  #define GSCALE 2            // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

  LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

  
  float temperatureSent;
  int BPMSent;
  float x;
  float y;
  float z;
  float checksum;
  
  const byte SCALE = 2;
  const byte dataRate = 0;

  int int1Pin = 2;  
  int int2Pin = 3; 

  const int ALERT_PIN = A3;
  
  int pulsePin = 0;                 // Pulse Sensor purple wire connected to analog pin 0
  int blinkPin = 13;                // pin to blink led at each beat
  int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
  int fadeRate = 0;                 // used to fade LED on with PWM on fadePin

  volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
  volatile int Signal;                // holds the incoming raw data
  volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
  volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
  volatile boolean QS = false; 
  #define SERIAL_PLOTTER  2
  static int outputType = SERIAL_PLOTTER;
  
  byte c;
  
  TMP102 sensor0(0x48);

  byte heart[8] = {
  B00000,
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
};

  
void setup() {
  
  Serial.begin(115200);
  Wire.begin();

  initMMA8452(); 

  lcd.createChar(0, heart);
  lcd.begin(16, 2);      
  

  

  pinMode(ALERT_PIN, INPUT); // Declare alertPin as an input
  sensor0.begin();  // Join I2C bus

  sensor0.setFault(0);

  sensor0.setAlertPolarity(1); 

  sensor0.setAlertMode(0); 

  sensor0.setConversionRate(2);

  sensor0.setExtendedMode(1);

  sensor0.setHighTempC(29.4);

  sensor0.setLowTempC(26.67); 

  
  pinMode(blinkPin, OUTPUT);  
  pinMode(fadePin, OUTPUT);                      
  interruptSetup();   


  lcd.begin(16, 2);
}


void loop() {

  float temperature;

  sensor0.wakeup();

  temperature = sensor0.readTempC();

  sensor0.sleep();

  setTemperature(temperature);

  delay(50);

  serialOutput() ;

  if (QS == true) {    // A Heartbeat Was Found
    // BPM and IBI have been Determined
    // Quantified Self "QS" true when arduino finds a heartbeat
    fadeRate = 255;         // Makes the LED Fade Effect Happen
    // Set 'fadeRate' Variable to 255 to fade LED with pulse
    //serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
    QS = false;                      // reset the Quantified Self flag for next time
  }

  ledFadeToBeat();                      // Makes the LED Fade Effect Happen
  delay(20);                             //  take a break

  setHeartbeat(BPM);


//XYZ


  int accelCount[3];  // Stores the 12-bit signed value
  readAccelData(accelCount);  // Read the x/y/z adc values

  // Now we'll calculate the accleration value into actual g's
  float accelG[3];  // Stores the real accel value in g's
  for (int i = 0 ; i < 3 ; i++)
  {
    accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE));  // get actual g value, this depends on scale being set
  }

  setXAccelero(accelG[0]);
  setYAccelero(accelG[1]);
  setZAccelero(accelG[2]);
  

  delay(10);  // Delay here for visibility

//XYZ

createChecksum();
concatString();




  lcd.setCursor(0,0);

  lcd.print(temperatureSent);
  lcd.print((char)223);
  lcd.print(String("C"));

  lcd.setCursor(7,0);

  lcd.write(byte(0));
  lcd.print(BPMSent);
  lcd.print(String(" "));

  lcd.setCursor(0,1);
  lcd.print(String("X: "));
  lcd.print(x);

  lcd.setCursor(9,1);
  lcd.print(String("Y: "));
  lcd.print(y);

  lcd.setCursor(11,0);
  lcd.print(String("Z:"));
  lcd.print(z);




}





void ledFadeToBeat() {
  fadeRate -= 15;                         //  set LED fade value
  fadeRate = constrain(fadeRate, 0, 255); //  keep LED fade value from going into negative numbers!
  analogWrite(fadePin, fadeRate);         //  fade LED
}






// Methodes XYZ

void readAccelData(int *destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(OUT_X_MSB, 6, rawData);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1];  //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer

    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {  
      gCount -= 0x1000;
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

// Initialize the MMA8452 registers 
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte c = readRegister(WHO_AM_I);  // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {  
    Serial.println("MMA8452Q is online...");
  }
  else
  {
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  byte fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, fsr);

  //The default data rate is 800Hz and we don't modify it in this example code

  MMA8452Active();  // Set to active to start reading
}

// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();    
}

// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default

  while(!Wire.available()) ; //Wait for the data to come back
  return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}







void setTemperature(float temperature) {
  temperatureSent = temperature;


}

void setHeartbeat(int BPM) {
  BPMSent = BPM ;
}

void setXAccelero(float xxx) {
  x = xxx;

}

void setYAccelero(float yyy) {
  y = yyy;

}

void setZAccelero(float zzz) {
  z = zzz;

}

void createChecksum(){
  checksum = temperatureSent + BPMSent + x + y + z;
}

void concatString(){
  Serial.println(String("ABCDE") + temperatureSent + String("|") + BPMSent 
  + String("XXX") + x + String("YYY") + y + String("ZZZ") + z + String("CCC") 
  + checksum +String("EDCBA"));
}

