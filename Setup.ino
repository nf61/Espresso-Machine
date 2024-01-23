#include <HX711.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_MAX31865.h>
#include <Adafruit_ADS7830.h>



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
const int P_GPIO = 34;
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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.print('hello')
  
  SetupScreen();
  SetupSensors();


}

void loop() {
  // put your main code here, to run repeatedly:
 // m.display.drawPixel(10, 10, SH110X_WHITE);
 // m.display.display();
 
  //read from sensors
  uint16_t rtd = thermo.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  //uint8_t fault = thermo.readFault();

  //Serial.print("Ratio = "); Serial.println(ratio,8);
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 10);     // Start at top-left corner
  //TEMP
  display.print((int)(thermo.temperature(RNOMINAL, RREF) * (9./5.) + 32.)); display.println(F("F"));
  //PRESSURE

  
  //display.println(analogRead(P_GPIO));
  display.println(F("N/A BAR"));
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
  //WEIGHT
  


  //write to screen




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

void SetupSensors()
{
  ////TEMP
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();  
}
