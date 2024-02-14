#include <HX711.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_ADS7830.h>
#include <Bounce2.h>
#include "EEPROM.h"

// Pump Switch Pin
const int PUMP_SWITCH = 14; 

int SwitchState = 0;

//Encoder setup
ESP32Encoder encoder;
const int clk = 25;
const int dt = 33;

// Instantiate eeprom to store temp variable so it remembers what the last temp was set at
EEPROMClass  TEMPS("eeprom0");
int SetTemp = 100; // tempurature that the water should heat up to

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(15, 2, 4, 16);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);
// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
//TEMP
const int RREF = 430.0;
const int RNOMINAL = 100.0;

//PRESSURE
Adafruit_ADS7830 ad7830;

//WEIGHT
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 19;
const int LOADCELL_SCK_PIN = 18;

//SCREEN
const int SCREEN_WIDTH = 128;  // OLED display width, in pixels
const int SCREEN_HEIGHT = 128; // OLED display height, in pixels
const int OLED_RESET = -1;     // can set an oled reset pin if desired
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);
HX711 scale;

//Timer
float ShotTimer = 0.0;
float InitialVal = 0;

//Boiler

const int BOILER_PIN = 26;

//Pump

const int PUMP_PIN = 27;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SetupScreen();
  SetupSensors();
  SetupEncoder();
  //Set the group head switch 
  pinMode(PUMP_SWITCH, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(PUMP_SWITCH),PumpOnOrOff,CHANGE);
  ///testing relays for boiler and pump /// Boiler pin 27; Pump pin 26

pinMode(PUMP_PIN, OUTPUT);
pinMode(BOILER_PIN, OUTPUT);

///

}

void loop() {
  
   Serial.println(digitalRead(PUMP_SWITCH)); 
    
  ////
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);     // Start at top-left corner
  //TEMP
  DisplayTemp();
  //PRESSURE
    display.print("PSI = "); display.println((int)((ad7830.readADCsingle(0)-41)*.6444444));
   //SHOTTIMER
  StartTimer();
  DisplayTime();
  
  //SCALE
  if(scale.is_ready())
  {
      display.print(scale.get_units(),1); display.println(F("g"));
  }
  else
  {
    display.println("ERROR");
  }
  display.display();
  delay(1000);

 


}

void SetupScreen()
{

  delay(250); // wait for the OLED to power up

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.

  display.begin(0x3c, true); // Address 0x3D default
 //display.setContrast (0); // dim display
  display.clearDisplay();
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();

  // Show the display buffer on the hardware.
  // NOTE: You _must_ call display after making any drawing commands
  // to make them visible on the display hardware!
  display.display();
  delay(2000);
  display.clearDisplay();
}

void SetupEncoder()
{
  ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachHalfQuad(clk, dt);
  encoder.clearCount();
}

void SetupSensors()
{
  ////TEMP
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  ad7830.begin();
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();  
  
}

void DisplayTemp(){
   //read from sensors
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  //uint8_t fault = thermo.readFault();
  display.print("Temp = ");
  display.print((float)((thermo.temperature(RNOMINAL, RREF) * (9./5.)) + 32.));
  display.println(F(" F"));
}

void DisplayTime()
{
  display.print("Shot Time = ");
  display.println((float)(ShotTimer));

}

void StartTimer()
{ 
  if(SwitchState == HIGH){
    if(InitialVal == 0){InitialVal = micros();}
    ShotTimer = (micros() - InitialVal)/1000000.; 
  } else{
    InitialVal = 0;
    ShotTimer = 0; 
  }
}

void PumpOnOrOff()
{
  SwitchState = digitalRead(PUMP_SWITCH);
   if(SwitchState == HIGH){
   digitalWrite(PUMP_PIN, LOW);
  }else{
   digitalWrite(PUMP_PIN, HIGH);
  }
  
 
}
