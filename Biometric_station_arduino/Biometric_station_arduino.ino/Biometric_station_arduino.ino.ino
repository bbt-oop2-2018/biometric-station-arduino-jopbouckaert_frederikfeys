//#include <LiquidCrystal.h>
  #include <Wire.h>


//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

  float temperatureSent;
  int BPMSent;
  float x;
  float y;
  float z;
  
  const byte SCALE = 2;
  
  const byte dataRate = 0;

  int int1Pin = 2;  
  int int2Pin = 3;

  int accelCount[3];  
  float accelG[3];  

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
  #include "SparkFunTMP102.h"

  TMP102 sensor0(0x48);

  #include "i2c.h"



void setup() {
  Serial.begin(115200);
 

  #define PROCESSING_VISUALIZER 1
  
  #define SA0 1
  #if SA0
  #define MMA8452_ADDRESS 0x1D  // SA0 is high, 0x1C if low
  #else
  #define MMA8452_ADDRESS 0x1C
  #endif

  

  

  

  
  
  pinMode(int1Pin, INPUT);
  digitalWrite(int1Pin, LOW);
  pinMode(int2Pin, INPUT);
  digitalWrite(int2Pin, LOW);

  c = readRegister(0x0D);
  if (c == 0x2A) 
  {
    initMMA8452(SCALE, dataRate);
    Serial.println("MMA8452Q is online...");
  }
  else
  {
    Serial.print("Could not connect to MMA8452Q: 0x");
    Serial.println(c, HEX);
    while (1) ;
  }

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




}


//  Where the Magic Happens
void loop() {

  //XYZ

  static byte source;

  // If int1 goes high, all data registers have new data
  if (digitalRead(int1Pin) == 1) // Interrupt pin, should probably attach to interrupt function
  {
    readAccelData(accelCount);  // Read the x/y/z adc values



    // Now we'll calculate the accleration value into actual g's
    for (int i = 0; i < 3; i++)
      accelG[i] = (float) accelCount[i] / ((1 << 12) / (2 * SCALE)); // get actual g value, this depends on scale being set
    // Print out values
    for (int i = 0; i < 3; i++)
    {
//      Serial.print(accelG[i], 4);  // Print g values
      if(i == 0){  // tabs in between axes
        setXAccelero(accelG[i]);             
      }else if( i==1){
        setYAccelero(accelG[i]);
      }else if( i==2){
        setZAccelero(accelG[i]);
      }
    }
    Serial.println();
  }
  
  delay(200);  // Delay here for visibility

  float temperature;

  sensor0.wakeup();

  temperature = sensor0.readTempC();

  sensor0.sleep();

  setTemperature(temperature);

  delay(200);

  serialOutput() ;

  if (QS == true) {    // A Heartbeat Was Found
    // BPM and IBI have been Determined
    // Quantified Self "QS" true when arduino finds a heartbeat
    fadeRate = 255;         // Makes the LED Fade Effect Happen
    // Set 'fadeRate' Variable to 255 to fade LED with pulse
    serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
    QS = false;                      // reset the Quantified Self flag for next time
  }

  ledFadeToBeat();                      // Makes the LED Fade Effect Happen
  delay(20);                             //  take a break

setHeartbeat(BPM);

concatString();

}





void ledFadeToBeat() {
  fadeRate -= 15;                         //  set LED fade value
  fadeRate = constrain(fadeRate, 0, 255); //  keep LED fade value from going into negative numbers!
  analogWrite(fadePin, fadeRate);         //  fade LED
}





// Methodes XYZ


void readAccelData(int * destination)
{
  byte rawData[6];  // x/y/z accel register data stored here

  readRegisters(0x01, 6, &rawData[0]);  // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for (int i = 0; i < 6; i += 2)
  {
    destination[i / 2] = ((rawData[i] << 8) | rawData[i + 1]) >> 4; // Turn the MSB and LSB into a 12-bit value
    if (rawData[i] > 0x7F)
    {
      // If the number is negative, we have to make it so manually (no 12-bit data type)
      destination[i / 2] -= 0x1000;
    }
  }
}

// This function will read the status of the tap source register.
// Print if there's been a single or double tap, and on what axis.
void tapHandler()
{
  byte source = readRegister(0x22);  // Reads the PULSE_SRC register

  if ((source & 0x10) == 0x10) // If AxX bit is set
  {
    if ((source & 0x08) == 0x08) // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on X");  // tabbing here for visibility
    else
      Serial.print("Single (1) tap on X");

    if ((source & 0x01) == 0x01) // If PoIX is set
      Serial.println(" +");
    else
      Serial.println(" -");
  }
  if ((source & 0x20) == 0x20) // If AxY bit is set
  {
    if ((source & 0x08) == 0x08) // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on Y");
    else
      Serial.print("Single (1) tap on Y");

    if ((source & 0x02) == 0x02) // If PoIY is set
      Serial.println(" +");
    else
      Serial.println(" -");
  }
  if ((source & 0x40) == 0x40) // If AxZ bit is set
  {
    if ((source & 0x08) == 0x08) // If DPE (double puls) bit is set
      Serial.print("    Double Tap (2) on Z");
    else
      Serial.print("Single (1) tap on Z");
    if ((source & 0x04) == 0x04) // If PoIZ is set
      Serial.println(" +");
    else
      Serial.println(" -");
  }
}

void initMMA8452(byte fsr, byte dataRate)
{
  MMA8452Standby();  // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  if ((fsr == 2) || (fsr == 4) || (fsr == 8))
    writeRegister(0x0E, fsr >> 2);
  else
    writeRegister(0x0E, 0);

  // Setup the 3 data rate bits, from 0 to 7
  writeRegister(0x2A, readRegister(0x2A) & ~(0x38));
  if (dataRate <= 7)
    writeRegister(0x2A, readRegister(0x2A) | (dataRate << 3));

  // Set up portrait/landscap registers - 4 steps:
  // 1. Enable P/L
  // 2. Set the back/front angle trigger points (z-lock)
  // 3. Set the threshold/hysteresis angle
  // 4. Set the debouce rate
  // For more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4068.pdf
  writeRegister(0x11, 0x40);  // 1. Enable P/L
  writeRegister(0x13, 0x44);  // 2. 29deg z-lock (don't think this register is actually writable)
  writeRegister(0x14, 0x84);  // 3. 45deg thresh, 14deg hyst (don't think this register is writable either)
  writeRegister(0x12, 0x50);  // 4. debounce counter at 100ms (at 800 hz)

  // Set up single and double tap - 5 steps:
  // 1. Set up single and/or double tap detection on each axis individually.
  //2. Set the threshold - minimum required acceleration to cause a tap.
  //3. Set the time limit - the maximum time that a tap can be above the threshold
  //4. Set the pulse latency - the minimum required time between one pulse and the next
  //5. Set the second pulse window - maximum allowed time between end of latency and start of second pulse
  //for more info check out this app note: http://cache.freescale.com/files/sensors/doc/app_note/AN4072.pdf
  writeRegister(0x21, 0x7F);  // 1. enable single/double taps on all axes
  // writeRegister(0x21, 0x55);  // 1. single taps only on all axes
  // writeRegister(0x21, 0x6A);  // 1. double taps only on all axes
  writeRegister(0x23, 0x20);  // 2. x thresh at 2g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x24, 0x20);  // 2. y thresh at 2g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x25, 0x08);  // 2. z thresh at .5g, multiply the value by 0.0625g/LSB to get the threshold
  writeRegister(0x26, 0x30);  // 3. 30ms time limit at 800Hz odr, this is very dependent on data rate, see the app note
  writeRegister(0x27, 0xA0);  // 4. 200ms (at 800Hz odr) between taps min, this also depends on the data rate
  writeRegister(0x28, 0xFF);  // 5. 318ms (max value) between taps max

  // Set up interrupt 1 and 2
  writeRegister(0x2C, 0x02);  // Active high, push-pull interrupts
  writeRegister(0x2D, 0x19);  // DRDY, P/L and tap ints enabled
  writeRegister(0x2E, 0x01);  // DRDY on INT1, P/L and taps on INT2

  MMA8452Active();  // Set to active to start reading
}

// Sets the MMA8452 to standby mode.
// It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(0x2A);
  writeRegister(0x2A, c & ~(0x01));
}

// Sets the MMA8452 to active mode.
// Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(0x2A);
  writeRegister(0x2A, c | 0x01);
}

// Read i registers sequentially, starting at address into the dest byte array
void readRegisters(byte address, int i, byte * dest)
{
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS << 1)); // write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address);  // write register address
  i2cWaitForComplete();

  i2cSendStart();
  i2cSendByte((MMA8452_ADDRESS << 1) | 0x01); // write 0xB5
  i2cWaitForComplete();
  for (int j = 0; j < i; j++)
  {
    i2cReceiveByte(TRUE);
    i2cWaitForComplete();
    dest[j] = i2cGetReceivedByte(); // Get MSB result
  }
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN); // Disable TWI
  sbi(TWCR, TWEN); // Enable TWI
}

// Read a single byte from address and return it as a byte
byte readRegister(uint8_t address)
{
  byte data;

  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS << 1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address); // Write register address
  i2cWaitForComplete();

  i2cSendStart();

  i2cSendByte((MMA8452_ADDRESS << 1) | 0x01); // Write 0xB5
  i2cWaitForComplete();
  i2cReceiveByte(TRUE);
  i2cWaitForComplete();

  data = i2cGetReceivedByte();  // Get MSB result
  i2cWaitForComplete();
  i2cSendStop();

  cbi(TWCR, TWEN);  // Disable TWI
  sbi(TWCR, TWEN);  // Enable TWI

  return data;
}

// Writes a single byte (data) into address
void writeRegister(unsigned char address, unsigned char data)
{
  i2cSendStart();
  i2cWaitForComplete();

  i2cSendByte((MMA8452_ADDRESS << 1)); // Write 0xB4
  i2cWaitForComplete();

  i2cSendByte(address); // Write register address
  i2cWaitForComplete();

  i2cSendByte(data);
  i2cWaitForComplete();

  i2cSendStop();

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

void concatString(){
  Serial.print(String("ABCDE") + temperatureSent + String("|") + BPMSent + String("XXX") + x + String("YYY") + y + String("ZZZ") + z + String("EDCBA"));
}


