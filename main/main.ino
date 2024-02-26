#include <HX711.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_ADS7830.h>
#include <Bounce2.h>
#include "EEPROM.h"
#include "ArduPID.h"




// Pump Switch Pin
const int PUMP_SWITCH = 14; 
int SwitchState = 1;

//Encoder setup

ESP32Encoder encoder;
const int CLK = 25;
const int DT = 33;
const int SW = 32;
int encoderButtonState;
int encoderPosition;
long oldPosition = 0;
int menuPosition;

// Instantiate eeprom to store temp variable so it remembers what the last temp was set at
EEPROMClass  TEMPS("eeprom0");
double setTemp = 100; // tempurature that the water should heat up to

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
double BOILER_TEMP;

//PRESSURE
Adafruit_ADS7830 ad7830;
int BOILER_PRESSURE;

//WEIGHT
// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 19;
const int LOADCELL_SCK_PIN = 18;
int currentWeight;
int scaleSetWeight;
int scaleOffset;

//SCREEN
const int SCREEN_WIDTH = 128;  // OLED display width, in pixels
const int SCREEN_HEIGHT = 128; // OLED display height, in pixels
const int OLED_RESET = -1;     // can set an oled reset pin if desired
Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 100000);
HX711 scale;

//Timer
double ShotTimer;
double InitialVal;

//Boiler
const int BOILER_PIN = 27;

//Pump
const int PUMP_PIN = 26;

//PID
ArduPID myController;
double input;
double output;

// Arbitrary setpoint and gains - adjust these as fit for your project:
double setpoint = 170;
double p = 5;
double i = 1;
double d = 0.5;
//



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SetupScreen();
  SetupSensors();
  SetupEncoder();
  //Set the group head switch 
  pinMode(PUMP_SWITCH, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(PUMP_SWITCH),PumpOnOrOff,CHANGE);
  ///Rotary Encoder Button
  pinMode(SW,INPUT_PULLUP);  
  /// Relays for boiler and pump /// Boiler pin 27; Pump pin 26
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(BOILER_PIN, OUTPUT);
  ///PID setup
  myController.begin(&input, &output, &setpoint, p, i, d);
  myController.setSampleTime(10);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  myController.setOutputLimits(0, 255);
  myController.setBias(255.0 / 2.0);
  myController.setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  myController.start();
}

void loop() {
  
  
  ////PID Boiler control test
  input = BOILER_TEMP; // 
  myController.compute();
 // myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
//                                              PRINT_OUTPUT   | // in the Serial plotter
//                                              PRINT_SETPOINT |
//                                              PRINT_BIAS     |
//                                              PRINT_P        |
//                                              PRINT_I        |
//                                              PRINT_D);
  
  
  // turns Boiler on or off based on output of PID
  analogWrite(BOILER_PIN, output); 

  //read Boiler pressure
  BOILER_PRESSURE = (ad7830.readADCsingle(0)-41)*.6444444;
  
  //READ temps
  BOILER_TEMP = (thermo.temperature(RNOMINAL, RREF) * (9./5.)) + 32.;
  uint16_t rtd = thermo.readRTD();
  double ratio = rtd;
  ratio /= 32768;
    //uint8_t fault = thermo.readFault();
  //Start timer when pump starts
  StartTimer();
  //Scroll through menu
  EncoderScroll();
//screen stuff  
 if(menuPosition == 1){
   display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);  
  DisplayTemp();
  display.display();
  delay(1000);
  
 } else{
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 10);     // Start at top-left corner
  DisplayTemp();
  DisplayTime();
  DisplayPressure();
  DisplayScale();
  display.display();
  delay(1000);
  }
   

///Menus    1st menu = data, 2nd menu= variables; variables include: Temp, High pressure shut off, PID variables, Shot timer set, weight set,  

 









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
  encoder.attachSingleEdge(CLK, DT);
  //encoder.attachHalfQuad(CLK, DT);
  encoder.clearCount();
  encoder.setFilter(1023);

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
  
  display.print("Temp = ");
  display.print((double)(BOILER_TEMP));
  display.println(F(" F"));
}


void DisplayPressure(){
  display.print("PSI = "); display.println((int)(BOILER_PRESSURE));
  
}


void DisplayTime()
{
  display.print("Shot Time = ");
  display.println((double)(ShotTimer));

}

void DisplayScale()
{
if(scale.is_ready())
  {
      currentWeight = scale.get_units(),1;
      display.print(currentWeight); display.println(F("g"));
  }
  else
  {
    display.println("ERROR");
  }
}

////Starts timer when pump turns on
void StartTimer()
{ 
  if(SwitchState == LOW){
    if(InitialVal == 0){InitialVal = micros();}
    ShotTimer = (micros() - InitialVal)/1000000.; 
  } else{
    delay(1000);///delay to show shottimer for a moment
    InitialVal = 0;
    ShotTimer = 0; 
  }
}

////Turns pump on or off using lever on front
void PumpOnOrOff()
{
  SwitchState = digitalRead(PUMP_SWITCH);
   if(SwitchState == HIGH){
   digitalWrite(PUMP_PIN, LOW);
  }else{
   digitalWrite(PUMP_PIN, HIGH);
  }
  }

//Turns Off Pump when Weight is achieved
void ScaleShutOff(){
  if(currentWeight == (scaleSetWeight + scaleOffset) || currentWeight > (scaleSetWeight + scaleOffset))
  {
    digitalWrite(PUMP_PIN, HIGH);
  }
}


////Encoder test
void EncoderScroll(){
  int numOfMenuItems = 1;
  long newPosition = encoder.getCount();
  if (newPosition != oldPosition) {
    if(menuPosition == numOfMenuItems){menuPosition = 0;}else{menuPosition++;}
  }
  oldPosition = newPosition;
}
