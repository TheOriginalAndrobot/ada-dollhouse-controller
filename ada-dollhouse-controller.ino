/*
   Ada's Dollhouse Controller

   Board: Sparkfun Pro Micro 3.3V
*/

//
// Enable debug mode
//
#define DEBUG

//
// Libraries
//
#include <Arduino.h>
#include <TimerThree.h>
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.
#include <Wire.h> // Include the I2C library
#include <SparkFunSX1509.h> // Include SX1509 library
#include <Adafruit_TLC5947.h>
#include "Adafruit_Soundboard.h"


//
// Pin defs
//
const int PIN_IOB_SCL = 3;
const int PIN_IOB_SDA = 2;
const int PIN_IOB_INTn = 7;
//
const int PIN_PWM_CLK = 15;
const int PIN_PWM_DIN = 16;
const int PIN_PWM_LAT = 14;
const int PIN_PWM_BLANK = 8;
//
const int PIN_AMP_SHDNn = 4;
const int PIN_SFX_RST = 5;


//
// IO Board Constants
//
const byte IOB_LEDS[7] = {4, 5, 6, 7, 12, 13, 14};
const byte IOB_BTNS[8] = {0, 1, 2, 3, 8, 9 ,10 ,11};


//
// Light data
//
const unsigned int NUM_LIGHTS = 6;
const unsigned int LIGHT_LED_BRIGHTNESS = 4095;
const unsigned int LIGHT_LED_STEP = 64;
const byte LIGHT_BUTTON_BRIGHTNESS = 255;

// Maps light number (1st index) to pair of PWM channels (99 if unused)
const unsigned int LIGHT_CHAN_MAP[NUM_LIGHTS][2] = { {0, 1},
                                                     {2, 3},
                                                     {4, 99},
                                                     {5, 6},
                                                     {7, 99},
                                                     {8, 23} };
// Maps light nubmer to pushbutton lamp channel
const byte PB_LAMP_CHAN_MAP[NUM_LIGHTS] = {4, 5, 6, 7, 12, 13};

// Current on/off state of a light
volatile bool lightState[NUM_LIGHTS] = {0,0,0,0,0,0};

// Current light value
volatile unsigned int lightValue[NUM_LIGHTS] = {0,0,0,0,0,0};


//
// Objects
//
SX1509 io;
Adafruit_TLC5947 tlc = Adafruit_TLC5947(1, PIN_PWM_CLK, PIN_PWM_DIN, PIN_PWM_LAT);
Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial1, &Serial, PIN_SFX_RST);


//
// Global variables
//
volatile bool buttonPressed = false; // Flag button press in ISR
volatile bool lightsNeedUpdate = false; // Cause sync to hardware


//
// Run once at boot time
//
void setup() {
  
  byte ii;
  
  //
  // Initial states
  //
  pinMode(PIN_PWM_BLANK, OUTPUT);
  pwmBlankOn();
  //
  pinMode(PIN_AMP_SHDNn, OUTPUT);
  ampOff();
  //
  digitalWrite(PIN_SFX_RST, LOW);
  pinMode(PIN_SFX_RST, OUTPUT);

  
  //
  // Debug
  //
  #ifdef DEBUG
    delay(4000);
  #endif
  Serial.begin(9600);
  Serial.println("*** Ada's Dollhouse ***");
  
  
  //
  // IO Board stuff
  //
  
  // External interrupt pin (Interrupt enabled at end)
  pinMode(PIN_IOB_INTn, INPUT);
  
  // Call io.begin(<I2C address>) to initialize the I/O
  // expander. It'll return 1 on success, 0 on fail.
  if (!io.begin(0x3E))
  {
    // We failed to communicate, loop forever
    while (1){
      Serial.println("ERROR: Could not init IO Expander");
      delay(5000);
    }
  }
  
  // Use the internal 2MHz oscillator.
  // Set LED clock to 500kHz (2MHz / (2^(3-1)):
  io.clock(INTERNAL_CLOCK_2MHZ, 3);
  
  // The SX1509 has built-in debounce features, so a single 
  // button-press doesn't accidentally create multiple ints.
  // Use io.debounceTime(<time_ms>) to set the GLOBAL SX1509 
  // debounce time.
  // <time_ms> can be either 0, 1, 2, 4, 8, 16, 32, or 64 ms.
  io.debounceTime(32); // Set debounce time to 32 ms.
  
  // LED Pins
  for (ii=0; ii<7; ii++) {
    io.analogWrite(IOB_LEDS[ii], 0);
    io.pinMode(IOB_LEDS[ii], ANALOG_OUTPUT);
  }
  
  // Button Contact Pins
  for (ii=0; ii<8; ii++) {
    io.pinMode(IOB_BTNS[ii], INPUT_PULLUP);
    io.enableInterrupt(IOB_BTNS[ii], FALLING);
    io.debouncePin(IOB_BTNS[ii]);
  }
  
  
  //
  // LED driver board setup
  //
  if (!tlc.begin()) {
    while (1){
      Serial.println("ERROR: Could not init PWM Board");
      delay(5000);
    }
  }
  
  // Clear all LEDs (buffer starts as all zero)
  tlc.write();
  
  // Un-blank
  pwmBlankOff();
  
  
  //
  // Audio stuff
  //
  Serial1.begin(9600);  // This is the HW UART to the SFX chip
  ampOff();             // Turn off the Amp (also configs the PIN)
  flushSFXInput();      // Clear serial buffer
  
  if (!sfx.reset()) {
    while (1){
      Serial.println("ERROR: Could not init SFX Board");
      delay(5000);
    }
  }
  
  
  //
  // Timer for periodic interrupts every 10ms
  //
  Timer3.initialize(10000);
  Timer3.attachInterrupt(runOnTimer);


  //
  // Enable interrupts
  //

  // Clear any pending interrupts
  io.interruptSource();
  // Button down ISR
  attachInterrupt(digitalPinToInterrupt(PIN_IOB_INTn), iobISR, FALLING);

}


//
// Main Loop (runs forever)
//
void loop() {
    
  //
  // Handle button presses
  //
  if (buttonPressed) {
    // Clear the flag early to avoid race condition
    buttonPressed = false; 
    
    // read io.interruptSource() find out which pin generated
	  // an interrupt and clear the SX1509's interrupt output.
    unsigned int intStatus = io.interruptSource();
    
	  // For debugging handiness, print the intStatus variable.
	  // Each bit in intStatus represents a single SX1509 IO.
    //Serial.println("intStatus = " + String(intStatus, BIN));

    // Do the things
    buttonDecode(intStatus);
  }

  //
  // Process light updates
  //
  syncLightsIfNeeded();
}


//
// ISR for IOB changes (aka button presses)
//
void iobISR() {
  buttonPressed = true; // Set the buttonPressed flag to true
  // We can't do I2C communication in an Arduino ISR. The best
  // we can do is set a flag, to tell the loop() to check next 
  // time through.
}


//
// Do periodic work on timer interrupt
//
void runOnTimer(){
  // Perform light value fading
  updateLightValues();
}

//
// Decodes which button(s) were pressed and calls button action on each
//
void buttonDecode(unsigned int btnBitmap) {
  for (int btnNum=0; btnNum<16; btnNum++){
    if (btnBitmap & (1<<btnNum)){
      Serial.println("Button " + String(btnNum) + " pressed");
      buttonAction(btnNum);
    }
  }
}


//
// Perform action for given button number
//
void buttonAction(int button) {
  switch (button) {
    // Living room
    case 0:
      toggleLight(0);
      break;
    // Kitchen
    case 1:
      toggleLight(1);
      break;
    // Porch
    case 2:
      toggleLight(2);
      break;
    // Master Bedroom
    case 3:
      toggleLight(3);
      break;
    // Bathroom
    case 8:
      toggleLight(4);
      break;
    // Kids Room
    case 9:
      toggleLight(5);
      break;
    // Doorbell
    case 10:
      ringDoorbell();
      break;
    // All Off
    case 11:
      allOff();
      break;
  }
}


//
// Toggle a light fixture and indicator lamp
//
void toggleLight(int lightNum) {
  setLight(lightNum, !lightState[lightNum]);
}

//
// Set light state
//
void setLight(int lightNum, bool newState){

  if (newState) {
    // Turn on
    io.analogWrite(PB_LAMP_CHAN_MAP[lightNum], LIGHT_BUTTON_BRIGHTNESS);
    lightState[lightNum] = true;
  } else {
    // Turn off
    io.analogWrite(PB_LAMP_CHAN_MAP[lightNum], 0);
    lightState[lightNum] = false;
  }
}


//
// Update light values
//
//   This performs fade-in and fade-out of the light values
//
void updateLightValues() {
  int lVal;
  bool needsUpdate = false;

  for (int lightNum=0; lightNum<NUM_LIGHTS; lightNum++){
    lVal = lightValue[lightNum];
    // Going up
    if (lightState[lightNum] && lVal < LIGHT_LED_BRIGHTNESS){
      lVal += LIGHT_LED_STEP;
      if (lVal > LIGHT_LED_BRIGHTNESS){
        lVal = LIGHT_LED_BRIGHTNESS;
      }
      lightValue[lightNum] = lVal;
      needsUpdate = true;
    }
    // Going down
    else if (!lightState[lightNum] && lVal > 0){
      lVal -= LIGHT_LED_STEP;
      if (lVal < 0){
        lVal = 0;
      }
      lightValue[lightNum] = lVal;
      needsUpdate = true;
    }
  }

  // Only set, don't clear
  if (needsUpdate){
    lightsNeedUpdate = true;
  }
}


//
// Sync light values out to hardware if needed
//
bool syncLightsIfNeeded(){
  unsigned int lVal;
  
  // Bail if nothing needs to be done
  if (!lightsNeedUpdate){
    return false;
  }

  // Clear flag first thing so we don't miss an asyc update to values
  lightsNeedUpdate = false;

  // Load all light values
  for (int lightNum=0; lightNum<NUM_LIGHTS; lightNum++){
    // Grab light value in a safe way
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      lVal = lightValue[lightNum];
    }
    tlc.setPWM(LIGHT_CHAN_MAP[lightNum][0], lVal);
    tlc.setPWM(LIGHT_CHAN_MAP[lightNum][1], lVal);
  }

  // Sync with hardware
  tlc.write();

  return true;
}


//
// Ring the doorbell
//
void ringDoorbell() {
  // Turn on amp
  ampOn();
  
  // Play Doorbell sound
  sfx.playTrack((uint8_t)0);

}


//
// Amplifier power control
//
// Note: Pull-up on SHDN pin has been removed from amp board
//       and a 100k pull-down has been isntalled on mobo
//
void ampOff() {
  digitalWrite(PIN_AMP_SHDNn, LOW);
}
void ampOn() {
  digitalWrite(PIN_AMP_SHDNn, HIGH);
  delay(10);
}


//
// LED PWM helpers
//
// Note: Pull-down on SHDN pin has been removed from TLC board
//       and a 100k pull-up has been isntalled on mobo
//
void pwmBlankOn() {
  //pinMode(PIN_PWM_BLANK, OUTPUT);
  digitalWrite(PIN_PWM_BLANK, HIGH);
}
void pwmBlankOff() {
  //pinMode(PIN_PWM_BLANK, OUTPUT);
  digitalWrite(PIN_PWM_BLANK, LOW);
}


//
// Flush out SFX Serial buffers
//
void flushSFXInput() {
  // Read all available serial input to flush pending data.
  uint16_t timeoutloop = 0;
  while (timeoutloop++ < 40) {
    while(Serial1.available()) {
      Serial1.read();
      timeoutloop = 0;  // If char was received reset the timer
    }
    delay(1);
  }
}

//
// Turn everything off
//
void allOff() {
  
  // Kill any audio
  sfx.stop();
  ampOff();
  
  // Turn off all lights
  for (int lightNum=0; lightNum<NUM_LIGHTS; lightNum++){
    setLight(lightNum, false);
  }
  
  // Flush LED PWM data
  //tlc.write();
  
  
}
