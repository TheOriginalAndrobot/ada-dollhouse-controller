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
#include <util/atomic.h>    // this library includes the ATOMIC_BLOCK macro
#include <Wire.h>           // Include the I2C library
#include <SparkFunSX1509.h>
#include <Adafruit_TLC5947.h>
#include <TimerThree.h>

//
// Macros
//
#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
#endif

// System modes
#define SM_OFF    0
#define SM_ON     1
#define SM_QUIET  2


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
// SFX constants
//
const uint8_t NUM_DOORBELL_TRACKS = 5;


//
// IO Board Constants
//
const byte IOB_LEDS[7] = {4, 5, 6, 7, 12, 13, 14};
const byte IOB_BTNS[8] = {0, 1, 2, 3, 8, 9 ,10 ,11};
const byte PB_LAMP_SYS_MODE = 14;


//
// Timing parameters
//
const unsigned int TIMER_PERIOD_US = 10000;               // 10ms
const unsigned int AMP_POWER_TIMEOUT_TICKS = 1000;        // 10s
const unsigned long LIGHT_POWER_TIMEOUT_TICKS = 360000;   // 1h
const unsigned long SYS_POWER_TIMEOUT_TICKS = 361000;     // 1h+10s
const unsigned int LIGHT_LED_STEP = 64;

//
// Light data
//
const unsigned int NUM_LIGHTS = 6;
const unsigned int LIGHT_LED_BRIGHTNESS = 4095;
const byte LIGHT_BUTTON_BRIGHTNESS = 255;
const byte SYS_BUTTON_BRIGHTNESS_HI = 200;
const byte SYS_BUTTON_BRIGHTNESS_LOW = 30;

// Maps light number (1st index) to pair of PWM channels (99 if unused)
const unsigned int LIGHT_CHAN_MAP[NUM_LIGHTS][2] = { {0, 1},
                                                     {2, 3},
                                                     {4, 99},
                                                     {99, 6},
                                                     {7, 99},
                                                     {8, 99} };

// Maps light nubmer to pushbutton lamp channel
const byte PB_LAMP_CHAN_MAP[NUM_LIGHTS] = {4, 5, 6, 7, 12, 13};

// Current on/off state of a light
volatile bool lightState[NUM_LIGHTS] = {0,0,0,0,0,0};

// Current light value
volatile unsigned int lightValue[NUM_LIGHTS] = {0,0,0,0,0,0};

// SFX track nubmer map for light on/off events (use 0 for none)
const byte LIGHT_ON_SFX_MAP[NUM_LIGHTS] =  {5, 4, 3, 0, 0, 0};
const byte LIGHT_OFF_SFX_MAP[NUM_LIGHTS] = {0, 0, 0, 2, 1, 6};


//
// Objects
//
SX1509 io;
Adafruit_TLC5947 tlc = Adafruit_TLC5947(1, PIN_PWM_CLK, PIN_PWM_DIN, PIN_PWM_LAT);


//
// Global variables
//
volatile bool buttonPressed = false;      // Flag button press in ISR
volatile bool lightsNeedUpdate = false;   // Cause sync to hardware
volatile bool sysPowerState = false;      // System power state
volatile bool sysPowerTimeout = false;    // Flag to cause system to power down
volatile long sysPowerTimer = 0;          // Counts timer ticks toward timeout
volatile bool ampPowerState = false;      // Amp power state
volatile bool ampPowerTimeout = false;    // Flag to cause amp to power down
volatile int  ampPowerTimer = 0;          // Counts timer ticks toward timeout
volatile bool lightPowerTimeout = false;  // Flag to cause lights to power down
volatile long lightPowerTimer = 0;        // Counts timer ticks toward timeout
uint8_t doorbellTrack = 0;                // Holds next doorbell track number to play
byte sysMode = SM_OFF;                    // System mode

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
  delay(10);

  
  //
  // Debug
  //
  #ifdef DEBUG
    delay(2000);
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
  doorbellTrack = random(NUM_DOORBELL_TRACKS);

  // Take out of reset
  pinMode(PIN_SFX_RST, INPUT);
  delay(1500); // give a bit of time to 'boot up'

  // Adjust volume up
  Serial1.println("+");
  Serial1.println("+");
  delay(50);
  
  
  //
  // Welcome message
  //
  setSysMode(SM_ON);
  playSFX(9);
  
  
  //
  // Timer for periodic interrupts
  //
  Timer3.initialize(TIMER_PERIOD_US);
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

    // Do the things
    buttonDecode(intStatus);
  }

  //
  // Process light updates
  //
  syncLightsIfNeeded();

  //
  // Timeout logic checks
  //
  if (sysPowerTimeout){
    DEBUG_PRINT("System power timeout");
    // Turn off system
    setSysMode(SM_OFF);
    resetSysPowerTimer();
  }

  if (lightPowerTimeout){
    // Turn off all lights
    for (byte lightNum=0; lightNum<NUM_LIGHTS; lightNum++){
      setLight(lightNum, false);
    }
    resetLightPowerTimer();
    DEBUG_PRINT("Light power timeout");
  }

  if (ampPowerTimeout){
    ampOff();
    resetAmpPowerTimer();
    DEBUG_PRINT("Amp power timeout");
  }

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

  // Sys power timeout logic
  if (sysPowerState){
    sysPowerTimer++;
    if (sysPowerTimer > SYS_POWER_TIMEOUT_TICKS){
      sysPowerTimeout = true;
    }
  }

  // Light timeout logic
  if (anyLightOn()){
    lightPowerTimer++;
    if (lightPowerTimer > LIGHT_POWER_TIMEOUT_TICKS){
      lightPowerTimeout = true;
    }
  }

  // Amp timeout logic
  if (ampPowerState){
    ampPowerTimer++;
    if (ampPowerTimer > AMP_POWER_TIMEOUT_TICKS){
      ampPowerTimeout = true;
    }
  }
}


//
// Timeout timer reset fucntions
//
void resetAmpPowerTimer(){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    ampPowerTimeout = false;
    ampPowerTimer = 0;
  }
}
void resetLightPowerTimer(){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    lightPowerTimeout = false;
    lightPowerTimer = 0;
  }
}
void resetSysPowerTimer(){
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sysPowerTimeout = false;
    sysPowerTimer = 0;
  }
}


//
// Decodes which button(s) were pressed and calls button action on each
//
void buttonDecode(unsigned int btnBitmap) {
  for (int btnNum=0; btnNum<16; btnNum++){
    if (btnBitmap & (1<<btnNum)){
      DEBUG_PRINT("Button " + String(btnNum) + " pressed");
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
    // System mode
    case 11:
      toggleSysMode();
      break;
  }
}


//
// Toggle a light fixture and indicator lamp
//
void toggleLight(int lightNum) {
  // Toggle light state
  setLight(lightNum, !lightState[lightNum]);

  // Play sound effects
  if (lightState[lightNum]){
    playSFX(LIGHT_ON_SFX_MAP[lightNum]);
  }
  else {
    playSFX(LIGHT_OFF_SFX_MAP[lightNum]);
  }
}

//
// Set light state
//
void setLight(int lightNum, bool newState){

  // Turn on
  if (newState) {
    // Feed watchdogs
    resetLightPowerTimer();
    resetSysPowerTimer();

    // Turn on system if need be
    if (sysMode == SM_OFF){
      setSysMode(SM_ON);
    }
    
    // Turn on lamp and lights
    io.analogWrite(PB_LAMP_CHAN_MAP[lightNum], LIGHT_BUTTON_BRIGHTNESS);
    lightState[lightNum] = true;
  }
  // Turn off
  else {
    io.analogWrite(PB_LAMP_CHAN_MAP[lightNum], 0);
    lightState[lightNum] = false;
  }
}

//
// Return true if any lights are on
//
bool anyLightOn(){
  bool result = false;
  for (byte lightNum=0; lightNum<NUM_LIGHTS; lightNum++){
    result = result || lightState[lightNum];
  }
  return result;
}


//
// Update light values
//
//   This performs fade-in and fade-out of the light values
//
void updateLightValues() {
  int lVal;
  bool needsUpdate = false;

  for (byte lightNum=0; lightNum<NUM_LIGHTS; lightNum++){
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
  // Feed watchdogs
  resetSysPowerTimer();

  // Turn on system if need be
  if (sysMode == SM_OFF){
    setSysMode(SM_ON);
  }

  // Play next track and advance the sequence
  playSFX("T00RAND" + String(doorbellTrack) + "OGG");
  doorbellTrack = (doorbellTrack + 1) % NUM_DOORBELL_TRACKS;
}

//
// Play sound effect
//
void playSFX(uint8_t track){
  if (track == 0) return;
  playSFX("T0" + String(track) + "     OGG");
}
void playSFX(String filename){
  if (sysMode == SM_QUIET) return;
  DEBUG_PRINT("Playing track " + filename);
  ampOn();
  Serial1.println("P" + filename);
}

//
// Amplifier power control
//
// Note: Pull-up on SHDN pin has been removed from amp board
//       and a 100k pull-down has been installed on mobo
//
void ampOff() {
  ampPowerState = false;
  digitalWrite(PIN_AMP_SHDNn, LOW);
}
void ampOn() {
  resetAmpPowerTimer();
  digitalWrite(PIN_AMP_SHDNn, HIGH);
  if (!ampPowerState){
    delay(10);
  }
  ampPowerState = true;
}


//
// LED PWM helpers
//
// Note: Pull-down on blank pin has been removed from TLC board
//       and a 100k pull-up has been installed on mobo
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
// Toggle through system modes
//
void setSysMode(byte newMode) {

  switch (newMode) {
    case SM_ON:
      sysPowerState = true;
      io.breathe(PB_LAMP_SYS_MODE, 600, 300, 2000, 2500, SYS_BUTTON_BRIGHTNESS_HI, SYS_BUTTON_BRIGHTNESS_LOW, true);
      break;
    case SM_OFF:
      sysPowerState = false;
      io.blink(PB_LAMP_SYS_MODE, 0, 0, 0, 0);
      io.analogWrite(PB_LAMP_SYS_MODE, 0);
      allOff();
      break;
    case SM_QUIET:
      sysPowerState = true;
      io.blink(PB_LAMP_SYS_MODE, 400, 900, SYS_BUTTON_BRIGHTNESS_HI, SYS_BUTTON_BRIGHTNESS_LOW);
      ampOff();
      break;
  }

  sysMode = newMode;
}

//
// Toggle through system modes
//
void toggleSysMode() {
  switch (sysMode) {
    case SM_ON:
      setSysMode(SM_QUIET);
      break;
    case SM_OFF:
      setSysMode(SM_ON);
      playSFX(9);
      break;
    case SM_QUIET:
      setSysMode(SM_OFF);
      break;
  }
}


//
// Turn everything off
//
void allOff() {

  // Kill any playing audio
  Serial1.println("q");
  
  ampOff();
  
  // Turn off all lights
  for (byte lightNum=0; lightNum<NUM_LIGHTS; lightNum++){
    setLight(lightNum, false);
  }
}
