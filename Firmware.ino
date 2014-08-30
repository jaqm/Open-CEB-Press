  //******************
  // ** TODO **
  // - Review the blinking procedure. Specially for auto, manual and test mode.
  // *****************
  // ** DONE **
  // - SHAKER: we want to be able to move the shaker when the cylinders are NOT moving under timing. (<- Beginning of the auto-mode)
  // - Added Test mode.
  // - MAIN CYLINDER: we want all the time travel
  // If the potM value is close to the maximum we want to move it until High pressure
  // - DRAW CYLINDER: we want to be able to move it from the 1/4 to the 3/4
  //******************
  
//***********************************
// NOTES:
// * untilHighPressure functions always release pressure
// * Every mode, stage and substage must release pressure after reaching the high pressure point.
//     This is a must and will be applied using the releasePressure() function,
//     which is a non-stop function that runs until the high pressure pressure reaches the disabled status.
//***********************************


// -- CONFIG
// Debug mode
const boolean DEBUG_MODE=true;
const boolean DEBUG_VERBOSE_MODE=false;
const boolean DEBUG_LED_MODE=false;
// CONST - LED timing for blinking
const unsigned long VALUE_TIME_BLINKING_MANUAL=1000;
const unsigned long VALUE_TIME_BLINKING_AUTO=250;
const unsigned long VALUE_TIME_BLINKING_HIGH_PRESSURE=500;
//CONST - timers
const unsigned long VALUE_INPUT_READ_DELAY = 5;  // Delay (milliseconds) used to consider a stable input read.
const unsigned long VALUE_HP_READ_DELAY = 5;
const unsigned long VALUE_TIME_RELEASE_PRESSURE_STAGE = 200;
const unsigned long VALUE_MAX_TIME_RELEASE_PRESSURE = 1000;
// -- END OF CONFIG

// STANDARD VALUES
// inputs :: UP-LEFT-RIGHT-DOWN-SHAKER
const uint8_t VALUE_INPUT_ENABLED = LOW;
const uint8_t VALUE_INPUT_DISABLED = HIGH;
// inputs :: SWON - SWAUTO
const uint8_t VALUE_INPUT_SW_ENABLED = HIGH;
const uint8_t VALUE_INPUT_SW_DISABLED = LOW;
// inputs - high pressure sensor
//// Real values
const uint8_t VALUE_HP_ENABLED = HIGH;
const uint8_t VALUE_HP_DISABLED = LOW;
// For testing auto-mode purpose.
//const uint8_t VALUE_HP_ENABLED = LOW;
//const uint8_t VALUE_HP_DISABLED = HIGH;
// outputs - Solenoids
const uint8_t VALUE_SOL_ENABLED=HIGH;
const uint8_t VALUE_SOL_DISABLED=LOW;
// outputs - leds
const uint8_t VALUE_LED_ENABLED = LOW;
const uint8_t VALUE_LED_DISABLED = HIGH;
const uint8_t VALUE_LED_HIGHPRESSURE_ENABLED = HIGH;
const uint8_t VALUE_LED_HIGHPRESSURE_DISABLED = LOW;

// CONST - STATES OF THE AUTO-MODE
const short FAILSAFE_STAGE=0;
const short CALIBRATE_SOLENOIDS = 1;
const short EJECT_BRICK = 2;
const short PUSH_BRICK = 3;
const short LOAD_SOIL = 4;
const short CLOSE_CHAMBER = 5;
const short COMPRESS_SOIL = 6;
const short OPEN_CHAMBER = 7;
//CONST - NULL VALUES
const int VALUE_PIN_NULL=-1;
const unsigned long VALUE_TIMER_NULL=0;

// VALUE_MAX_POTM=2^10=1024 so the range is 0-1023
const int VALUE_MAX_POTM=1023;
const int VALUE_MAX_POTD=VALUE_MAX_POTM;

// INPUTS
int PIN_BUTTON_SHAKER=PIN_C1;    //button for shaker
int PIN_BUTTON_RIGHT=PIN_C2;    //button for right
int PIN_BUTTON_LEFT=PIN_C3;    //button for left
int PIN_BUTTON_DOWN=PIN_C4;    //button for down
int PIN_BUTTON_UP=PIN_C5;    //button for up
int PIN_SWAUTO=PIN_C6;    //auto/manual switch
int PIN_SWON=PIN_C7;    //on/off switch
int PIN_PRESSURE=PIN_F0;
int PIN_POTM=PIN_F1;    //potentiometer for the main cylinder
int PIN_POTD=PIN_F2;    //potentiometer for the drawer

// OUTPUTS - Solenoids
int PIN_SOLS=PIN_B2;    //solenoid for shaker motor 
int PIN_SOLR=PIN_B3;    //solenoid for drawer right
int PIN_SOLL=PIN_B4;    //solenoid for drawer left
int PIN_SOLD=PIN_B5;    //solenoid for cylinder down
int PIN_SOLU=PIN_B6;    //solenoid for cylinder up

// OUTPUTS - leds
int PIN_LED_HIGH_PRESSURE=PIN_E0;
int PIN_LED_STATUS=PIN_E1;

// PANEL ARRAY - it contains all the input panel values.
const int ID_SWON=0;
const int ID_SWAUTO=1;
const int ID_BUTTON_UP=2;
const int ID_BUTTON_DOWN=3;
const int ID_BUTTON_LEFT=4;
const int ID_BUTTON_RIGHT=5;
const int ID_BUTTON_SHAKER=6;
const int ID_PRESSURE=7;
const int AMOUNT_DIGITAL_INPUTS=8;
uint8_t digitalInputs[AMOUNT_DIGITAL_INPUTS];  // The array which contains all the digital input panel variables.

// ANALOG INPUTS ARRAY
const int ID_POTD=0;
const int ID_POTM=1;
const int AMOUNT_ANALOG_INPUTS=2;
int analogInputs[AMOUNT_ANALOG_INPUTS];

// TIMESARRAY
unsigned long timesArray[]={VALUE_TIMER_NULL,VALUE_TIMER_NULL,VALUE_TIMER_NULL,VALUE_TIMER_NULL,VALUE_TIMER_NULL};
const int ID_TIME_SOLU=0;
const int ID_TIME_SOLD=1;
const int ID_TIME_SOLL=2;
const int ID_TIME_SOLR=3;
const int ID_TIME_SOLS=4;

// loop() variables - general purpose
boolean flagHighPressure=false; // Flag to track if it was received a highPressure signal.
boolean chronoIsRunning=false;  // Flag to know if we are running the chrono.
// loop() variables - auto-mode
short stage;       // Defines the stage for the auto-mode.
short substage;    // Defines the substage for the auto-mode.
// loop() variables - test-mode
int testModeCylinderPin;  // test mode pin
// loop() variables - Timers
unsigned long blinkingStatusTimer;  // Timer to track the blinking procedure.
unsigned long blinkingHighPressureTimer;  // Timer to track the timing for the blinking procedure of the pressure sensor led.
unsigned long timer;          // Timer to track times.
unsigned long timestamp;
unsigned long movementTimer;          // Used to calculate the time that is being applied to move the main and the drawer cylinder.
unsigned long auxTimer;  // used to store times for debug purposes at any time in the code. 
unsigned long startingPoint;
unsigned long variableTravelTime;

// *** END OF CONSTANTS && VARIABLES
//**********************************

// ******* GETTERS && SETTERS *****
// Description: Sets all solenoids into mode.
// mode: HIGH or LOW values expected.
void setSolenoids(uint8_t mode){
    digitalWrite(PIN_SOLU,mode);            //turn all solenoids into mode
    digitalWrite(PIN_SOLD,mode);
    digitalWrite(PIN_SOLL,mode);
    digitalWrite(PIN_SOLR,mode);
    digitalWrite(PIN_SOLS,mode);
}

// Description: returns the status and only the status of the pin selected. 
// Input: pin: the value corresponding to the ping selected.
//        d (delay): amount of milliseconds between the first and the 2nd digital read to confirm the value.
// Return: the status as it is.
uint8_t pinDigitalValueIs(int pin, int d){

    uint8_t value0 = VALUE_INPUT_ENABLED;
    uint8_t value1 = VALUE_INPUT_DISABLED;

    do{
	value0 = digitalRead(pin);
	delay(d);
	value1 = digitalRead(pin);
        if (DEBUG_VERBOSE_MODE){
          Serial.print("InputIs value0: ");Serial.println(value1);
          Serial.print("InputIs value1: ");Serial.println(value1);
          if (value0!=value1){Serial.print("WARNING!! Different values were read. It could be a debounce. Too short delay?");}
        }
    }while (value0!=value1);
 
    return value0;
}

// Receiving a pin, it returns the pin of the cylinder in charge of the opposite movement.
uint8_t getOppositeSolenoid(uint8_t pinSol){
  
  uint8_t opposite=VALUE_PIN_NULL;
  if (pinSol==PIN_SOLU){opposite=PIN_SOLD;}
  else if (pinSol==PIN_SOLD){opposite=PIN_SOLU;}
  else if (pinSol==PIN_SOLL){opposite=PIN_SOLR;}
  else if (pinSol==PIN_SOLR){opposite=PIN_SOLL;}
  else if (pinSol==PIN_SOLS){opposite=PIN_SOLS;}
  else Serial.println("ERROR: Unexpected pin value received by getOppositeSolenoid()");

  if (DEBUG_VERBOSE_MODE){
    Serial.print("getOpposite() received the pin ");Serial.print(pinSol);
    Serial.print(" and is going to return ");Serial.println(opposite);
  }

  return opposite;
}

// Description: Receiving the digitalInputs array, it returns the enabled solenoid pin.
// array: the array containing the digitalInputs array.
// Default: VALUE_PIN_NULL
int getEnabledCylinder(uint8_t array[]){

  int activePin=VALUE_PIN_NULL;
  
  if (array[ID_BUTTON_UP]==VALUE_INPUT_ENABLED){
    activePin=PIN_SOLU;
  }else if (array[ID_BUTTON_DOWN]==VALUE_INPUT_ENABLED){
    activePin=PIN_SOLD;
  }else if (array[ID_BUTTON_LEFT]==VALUE_INPUT_ENABLED) {
    activePin=PIN_SOLL;
  }else if (array[ID_BUTTON_RIGHT]==VALUE_INPUT_ENABLED){
    activePin=PIN_SOLR;
  }
  if (DEBUG_VERBOSE_MODE) {Serial.print("The enabled solenoid pin is: ");Serial.println(activePin);}
  return activePin;
}

// **** END of GETTERS && SETTERS

// **** DATA HANDLING

// This function inverts a digital value read from a pin like HIGH or LOW.
// val: HIGH or LOW
// return: if val=HGH then return LOW, if val=LOW then return HIGH. Default: B00001111.
uint8_t revertDigitalSignalValue(uint8_t val){

 uint8_t oppositeValue=B00001111;
 
 if (val==HIGH){
   oppositeValue=LOW;
 }else if (val==LOW){
   oppositeValue=HIGH;
 }

  if (DEBUG_VERBOSE_MODE){
    Serial.print("revertDigitalSignalValue() received the value ");Serial.print(val);
    Serial.print(" and is going to return ");Serial.println(oppositeValue);
  }

 return oppositeValue;
}

// **** END of DATA HANDLING
// **********************
// **** MACHINE MOVEMENTS

// ** MACHINE MOVEMENTS -- NON-STOP FUNCTIONS

// Description: Moves the opposite cylinder during a short period of time. Used to release pressure.
unsigned long releasePressure(int cylinderPin, boolean &hpf){
  unsigned long auxT=VALUE_TIMER_NULL;
  if (DEBUG_MODE) Serial.println("Releasing pressure");
  auxT = moveCylinderUntilHighPressureBecomes( getOppositeSolenoid(cylinderPin),hpf,VALUE_HP_DISABLED, VALUE_MAX_TIME_RELEASE_PRESSURE);  // Release pressure
  if (DEBUG_MODE) Serial.print("Time measured to release pressure: ");Serial.println(auxT);
  return auxT;
}

// Moves the cylinder until high pressure sensor reaches the value secified and returns the time itgets to reach that place.
// cylinderPin: the pin assigned to the cylinder we want to move.
// &hpf: high pressure flag.
// hpv: the High Pressure Value we want to reach.
// maxTime: Maximum time we want to be moving the cylinder (safety mechanism).
// Return: time used to reach the desired high pressure value.
unsigned long moveCylinderUntilHighPressureBecomes(int cylinderPin, boolean &hpf, uint8_t hpv, unsigned long maxTime){

  unsigned long timestamp=millis();

  // Debug Mode
  if (DEBUG_MODE){
    Serial.print("moveCylinderUntilHighPressureBecomes: ");Serial.println(hpv);
    Serial.print("CylinderPin: "); Serial.println(cylinderPin);
  }

  while( (pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)!=hpv) && (timestamp + maxTime > millis()) ){
    digitalWrite(cylinderPin,VALUE_SOL_ENABLED);                // Cilinder movement.
  }
  if (pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)==VALUE_HP_ENABLED){hpf=true;}
  digitalWrite(cylinderPin,VALUE_SOL_DISABLED);

  return (millis()-timestamp);
}

// Description: Moves the cylinder during the time specified.
// As we are always releasing pressure after any cylinder movement we can check the high pressure sensor in this function, 
// which is good for the machine.
// This function will not stop the movement until high pressure or reaching the specified time. So it's recommeded for short periods of time.
// * Input - Outputs - return values *
// cylinderPin: the pin assigned to the cylinder.
// time: the time (milliseconds) that we want to move the cylinder.
// &hpf: high pressure flag.
void moveCylinderDuring(uint8_t cylinderPin,unsigned long time, boolean &hpf){

  unsigned long timestamp=millis();

  if (DEBUG_MODE){
    Serial.print("timestamp: ");Serial.println(timestamp);
    Serial.print("time: ");Serial.println(time);
    Serial.print("TIMESTAMP+TIME: ");Serial.println(timestamp+time);
    Serial.print("MILLIS: ");Serial.println(millis());
//      Serial.print("");Serial.println("");
  }

  while ( (pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)==VALUE_HP_DISABLED)  && (timestamp+time > millis()) ){
    digitalWrite(cylinderPin,VALUE_SOL_ENABLED);                // Cylinder movement.
  }
  digitalWrite(cylinderPin,VALUE_SOL_DISABLED);
  if (pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)) hpf=true;
}

// Moves both cylinders during the specified time or until HIGH PRESSURE.
void moveBothCylinderDuring(uint8_t cylinderPin1, uint8_t cylinderPin2, unsigned long timeMoving){

  unsigned long timestamp=millis();
  
  digitalWrite(cylinderPin1,VALUE_SOL_ENABLED);
  digitalWrite(cylinderPin2,VALUE_SOL_ENABLED);
  while ( (pinDigitalValueIs(PIN_PRESSURE,VALUE_INPUT_READ_DELAY)==VALUE_HP_DISABLED) && (timestamp+timeMoving > millis())){}
  digitalWrite(cylinderPin1,VALUE_SOL_DISABLED);
  digitalWrite(cylinderPin2,VALUE_SOL_DISABLED);

}

// Release the pressure from the solenoids using the data received from the panel.
// ONLY APPLIES FOR MANUAL MODE.
// digitalInputs[]: contains the information from the panel.
void releasePressureManualMode(uint8_t digitalInputs[]){

  if (pinDigitalValueIs(PIN_PRESSURE, VALUE_INPUT_READ_DELAY)==VALUE_HP_ENABLED){
    if (DEBUG_MODE){Serial.println("Applying reversal movement on solenoids..");}

    digitalWrite(PIN_SOLD,(digitalInputs[ID_BUTTON_UP]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLU,(digitalInputs[ID_BUTTON_DOWN]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLR,(digitalInputs[ID_BUTTON_LEFT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLL,(digitalInputs[ID_BUTTON_RIGHT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
//    digitalWrite(PIN_SOLS,(array[ID_BUTTON_SHAKER]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));  
    delay(VALUE_TIME_RELEASE_PRESSURE_STAGE);
    setSolenoids(VALUE_SOL_DISABLED);

  }
}

// ** MACHINE MOVEMENTS -- END of NON-STOP FUNCTIONS
// **
// ** MACHINE MOVEMENTS -- CHECK-AND-GO FUNCTIONS

// Moves the cylinder to the top position and returns the time itgets to reach that place.
// cylinderPin: the pin assigned to the cylinder we want to move.
// &hpf: high pressure flag.
void moveCylinderUntilHighPressure(int cylinderPin, boolean &hpf){

  if (DEBUG_MODE){
    Serial.println("moveCylinderUntilHighPressure: ");
    Serial.print("CylinderPin: "); Serial.println(cylinderPin);
  }

  digitalWrite(cylinderPin,VALUE_SOL_ENABLED);
  if(pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)==VALUE_HP_ENABLED){
    hpf=true;
    digitalWrite(cylinderPin,VALUE_SOL_DISABLED);
    releasePressure(cylinderPin,hpf);
  }
}

// ** MACHINE MOVEMENTS -- END OF CHECK-AND-GO FUNCTIONS

// ** MACHINE MOVEMENTS -- STATUS FUNCTIONS
// ** MACHINE MOVEMENTS -- END of STATUS FUNCTIONS

// **** END OF MACHINE MOVEMENTS

// **** MACHINE MODES

// Applies the actions expected in manual mode following the data stored in digitalInputs.
// digitalInputs[]: contains the information from the panel.
// &hpf: flag to track the high pressure sensor.
void applyManualMode(uint8_t digitalInputs[], boolean &hpf){

//// Uncomment the next "if" if you want to check the high pressure sensor beofre applying the manual mode.
//// NOTE: Consider to add a configurable constant to be able to configure this behaviour at the beginning of the code.
//  if (pinDigitalValueIs(PIN_PRESSURE, VALUE_INPUT_READ_DELAY)==VALUE_HP_DISABLED){

    if (DEBUG_MODE){Serial.println("Applying movement data to solenoids..");}
      digitalWrite(PIN_SOLU,(digitalInputs[ID_BUTTON_UP]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
      digitalWrite(PIN_SOLD,(digitalInputs[ID_BUTTON_DOWN]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
      digitalWrite(PIN_SOLL,(digitalInputs[ID_BUTTON_LEFT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
      digitalWrite(PIN_SOLR,(digitalInputs[ID_BUTTON_RIGHT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
      digitalWrite(PIN_SOLS,(digitalInputs[ID_BUTTON_SHAKER]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));  
//  }
//  else {
//    if (DEBUG_MODE){Serial.println("Warning: high pressure signal detected. Switching off solenoids.");}
//    hpf=true;
//    setSolenoids(VALUE_SOL_DISABLED);
//    // Uncomment the next line to enable the release pressure procedure for manual mode. This is NOT RECOMENDED because it would cause
//    // vibrations in the machine.
//    //releasePressureManualMode(digitalInputs);
//  }
}

// Applies the auto-mode.
// panel[]: the information readed from the machine.
// stage: which stage of the auto-mode do we want to run.
// &hpf: high pressure flag.
//void applyAutoMode(uint8_t panel[], unsigned long times[], short &stage, short &substage, boolean &hpf){
//}

// **** END OF MACHINE MODES
// **********************
// **** READ && SHOW FUNCTIONS

// Reads all the values of the panel, adding a check protection against rebounce, with a delay.
void readPanel(uint8_t digitalInputs[], int analogInputs[], const int d){  
  
  // read and delay for digital inputs
  uint8_t vU0 = digitalRead(PIN_BUTTON_UP);
  uint8_t vD0 = digitalRead(PIN_BUTTON_DOWN);
  uint8_t vL0 = digitalRead(PIN_BUTTON_LEFT);
  uint8_t vR0 = digitalRead(PIN_BUTTON_RIGHT);
  uint8_t vS0 = digitalRead(PIN_BUTTON_SHAKER);
  uint8_t vP0 = digitalRead(PIN_PRESSURE);
  uint8_t vSwOn0 = digitalRead(PIN_SWON);
  uint8_t vSwAuto0 = digitalRead(PIN_SWAUTO);
  
  delay(d);
  
  uint8_t vU1 = digitalRead(PIN_BUTTON_UP);
  uint8_t vD1 = digitalRead(PIN_BUTTON_DOWN);
  uint8_t vR1 = digitalRead(PIN_BUTTON_RIGHT);
  uint8_t vL1 = digitalRead(PIN_BUTTON_LEFT);
  uint8_t vS1 = digitalRead(PIN_BUTTON_SHAKER);
  uint8_t vP1 = digitalRead(PIN_PRESSURE);
  uint8_t vSwOn1 = digitalRead(PIN_SWON);
  uint8_t vSwAuto1 = digitalRead(PIN_SWAUTO);

  digitalInputs[ID_BUTTON_UP] = (vU0==vU1?vU0:VALUE_INPUT_DISABLED);  // If the analog inputs differ, we keep the old value.
  digitalInputs[ID_BUTTON_DOWN] = (vD0==vD1?vD0:VALUE_INPUT_DISABLED);
  digitalInputs[ID_BUTTON_LEFT] = (vL0==vL1?vL0:VALUE_INPUT_DISABLED);
  digitalInputs[ID_BUTTON_RIGHT] = (vR0==vR1?vR0:VALUE_INPUT_DISABLED);
  digitalInputs[ID_BUTTON_SHAKER] = (vS0==vS1?vS0:VALUE_INPUT_DISABLED);
  digitalInputs[ID_PRESSURE] = (vP0==vP1?vP0:VALUE_INPUT_DISABLED);
  digitalInputs[ID_SWON] = (vSwOn0==vSwOn1?vSwOn0:VALUE_INPUT_SW_DISABLED);
  digitalInputs[ID_SWAUTO] = (vSwAuto0==vSwAuto0?vSwAuto0:VALUE_INPUT_SW_DISABLED);

  analogInputs[ID_POTM]=analogRead(PIN_POTM);  // We do no extra checks for analog  inputs
  analogInputs[ID_POTD]=analogRead(PIN_POTD);  

}

// Prints the content of the digital and analog inputs.
void printPanel(uint8_t panel[], int analogInputs[]){

  Serial.println("********************************************");  
  Serial.print("Switch ON: "); Serial.println(panel[ID_SWON],DEC);
  Serial.print("Switch AUTO: "); Serial.println(panel[ID_SWAUTO],DEC);
  Serial.print("Button UP: "); Serial.println(panel[ID_BUTTON_UP],DEC);
  Serial.print("Button DOWN: "); Serial.println(panel[ID_BUTTON_DOWN],DEC);
  Serial.print("Button LEFT: "); Serial.println(panel[ID_BUTTON_LEFT],DEC);
  Serial.print("Button RIGHT: "); Serial.println(panel[ID_BUTTON_RIGHT],DEC);
  Serial.print("Button SHAKER: "); Serial.println(panel[ID_BUTTON_SHAKER],DEC);
  Serial.print("High PRESSURE: "); Serial.println(panel[ID_PRESSURE],DEC);
  Serial.print("Main Potentiometer: "); Serial.println(analogInputs[ID_POTM],DEC);
  Serial.print("Drawer potentiometer: "); Serial.println(analogInputs[ID_POTD],DEC);
  Serial.println("********************************************"); 

}

// Shows the content of the timesArray
void printTimesArray(unsigned long ta[]){

  Serial.println("********************************************");
  Serial.print("Time SOLU: "); Serial.println(ta[ID_TIME_SOLU],DEC);
  Serial.print("Time SOLD: "); Serial.println(ta[ID_TIME_SOLD],DEC);
  Serial.print("Time SOLL: "); Serial.println(ta[ID_TIME_SOLL],DEC);
  Serial.print("Time SOLR: "); Serial.println(ta[ID_TIME_SOLR],DEC);
  Serial.print("Time SOLS: "); Serial.println(ta[ID_TIME_SOLS],DEC);
  Serial.println("********************************************"); 

}

// **** END OF - READ && SHOW FUNCTIONS
// **** DOWN FROM HERE - FUNCTIONS UNDER REVIEW

// Updates the leds according to the information from the panel and the flags.
// panel[]: The array containing the information from the panel.
// &sbt: the status blinking timer. It contains the last time (millisecs) in which the led status changed it's status.
// &hpf: high pressure sensor flag. It is true if the high pressure sensor was activated.
// &hpt: high pressure timer. It tracks the last time the led was activated dure to the high presure sensor flag.
// Output:
// Led status: It blinks with different frequency depending on the mode we are running.
// Led high pressure: It is enabled if the high pressure flag in true. If so we always turn on the led and put the flag false.
//   Disabled if the flag is false and the time has passed.
void updateLeds(uint8_t panel[],unsigned long &sbt, boolean &hpf, unsigned long &hpt){

  // BLINKING PROCEDURE - STATUS LED
  // Note: Consider taking out this variable
  unsigned long timer=((panel[ID_SWAUTO]==VALUE_INPUT_SW_ENABLED)?VALUE_TIME_BLINKING_AUTO:VALUE_TIME_BLINKING_MANUAL);
  unsigned long presentTime=millis();
  uint8_t value=0;
  uint8_t value1=0;
  if (DEBUG_LED_MODE){
    Serial.print("UPDATING LED STATUS. Present time:");Serial.println(presentTime);
    Serial.print("Last Status Led udpate: ");Serial.println(sbt);
    Serial.print("timer: ");Serial.println(timer);
    delay(5000);
  }
  if (presentTime > sbt+timer){
      digitalWrite(PIN_LED_STATUS,revertDigitalSignalValue(pinDigitalValueIs(PIN_LED_STATUS, VALUE_INPUT_READ_DELAY)) );
      sbt=presentTime;
      if (DEBUG_LED_MODE){Serial.println("LED STATUS HAS BEEN UPDATED"); delay(1000);}
  }

  // BLINKING PROCEDURE - HIGH PRESSURE LED
  if (DEBUG_LED_MODE){
    Serial.print("hpt: ");Serial.println(hpt);
    Serial.print("VALUE_TIME_BLINKING_HIGH_PRESSURE: ");Serial.println(VALUE_TIME_BLINKING_HIGH_PRESSURE);
  }

  if (DEBUG_LED_MODE){Serial.print("Last high pressure led update:");Serial.println(hpt);}

  if (hpf){

    if (DEBUG_LED_MODE){
      Serial.println("HIGH PRESSURE FLAG ON.");
      Serial.println("HIGH PRESSURE LED IS GONNA BE UPDATED.");delay(5000);
    }
    digitalWrite(PIN_LED_HIGH_PRESSURE,VALUE_LED_HIGHPRESSURE_ENABLED);
    hpt=presentTime;
    // ******
    // This is the only place in the code where highPressureFlag becomes false.
    // Take care about this detail for the rest of the code to be consistent.
    // High pressure flag != highPressureSensor input. This flag is to track the high pressure sensor led behaviour.
    // To know the real state of the high pressure sensor input just check the input.
    // ******
    hpf=false;
    if (DEBUG_LED_MODE){Serial.println("HIGH PRESSURE LED HAS BEEN UPDATED.");delay(10000);}

  }else if(presentTime>hpt+VALUE_TIME_BLINKING_HIGH_PRESSURE){

    if (DEBUG_LED_MODE){Serial.println("HIGH PRESSURE FLAG OFF.");}
    digitalWrite(PIN_LED_HIGH_PRESSURE,VALUE_LED_HIGHPRESSURE_DISABLED);

  }
}

// *** UP FROM HERE - FUNCTIONS UNDER REVIEW

// *****************************

void setup() {
    // Define Inputs/Outputs
    pinMode(PIN_SOLU, OUTPUT);
    pinMode(PIN_SOLD, OUTPUT);
    pinMode(PIN_SOLL, OUTPUT);
    pinMode(PIN_SOLR, OUTPUT);
    pinMode(PIN_SOLS, OUTPUT);

    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_HIGH_PRESSURE, OUTPUT);

    pinMode(PIN_BUTTON_UP, INPUT);
    pinMode(PIN_BUTTON_DOWN, INPUT);
    pinMode(PIN_BUTTON_LEFT, INPUT);
    pinMode(PIN_BUTTON_RIGHT, INPUT);
    pinMode(PIN_BUTTON_SHAKER, INPUT);
    pinMode(PIN_PRESSURE, INPUT);
    pinMode(PIN_SWON, INPUT);
    pinMode(PIN_SWAUTO, INPUT);
    pinMode(PIN_POTM, INPUT);
    pinMode(PIN_POTD, INPUT);

    // Set initial status - OUTPUTS
    digitalWrite(PIN_BUTTON_UP, HIGH);
    digitalWrite(PIN_BUTTON_DOWN, HIGH);
    digitalWrite(PIN_BUTTON_LEFT, HIGH);
    digitalWrite(PIN_BUTTON_RIGHT, HIGH);
    digitalWrite(PIN_BUTTON_SHAKER, HIGH);
    digitalWrite(PIN_PRESSURE, HIGH);
    digitalWrite(PIN_SWON, HIGH);

    // Set initial status - Wake up INPUTS
    digitalWrite(PIN_BUTTON_UP, HIGH);
    digitalWrite(PIN_BUTTON_DOWN, HIGH);
    digitalWrite(PIN_BUTTON_LEFT, HIGH);
    digitalWrite(PIN_BUTTON_RIGHT, HIGH);
    digitalWrite(PIN_BUTTON_SHAKER, HIGH);
    digitalWrite(PIN_PRESSURE, HIGH);
    digitalWrite(PIN_SWON, HIGH);
    digitalWrite(PIN_SWAUTO, HIGH);
    digitalWrite(PIN_POTM, HIGH);
    digitalWrite(PIN_POTD, HIGH);

    // Initial setup for loop() variables
    stage=FAILSAFE_STAGE;
    substage=0;
    blinkingStatusTimer=millis();
    blinkingHighPressureTimer=VALUE_TIMER_NULL;
    flagHighPressure=false;
    timer=millis();
    timestamp=millis();
    chronoIsRunning=false;
    testModeCylinderPin=VALUE_PIN_NULL;
    movementTimer=VALUE_TIMER_NULL;
    auxTimer=VALUE_TIMER_NULL;

    startingPoint=VALUE_TIMER_NULL;
    variableTravelTime=VALUE_TIMER_NULL;


    Serial.begin(9600);
}

void loop() {

  readPanel(digitalInputs, analogInputs, VALUE_INPUT_READ_DELAY);
  updateLeds(digitalInputs, blinkingStatusTimer, flagHighPressure, blinkingHighPressureTimer);
  if (DEBUG_MODE){ printPanel(digitalInputs,analogInputs); printTimesArray(timesArray);};

  if (digitalInputs[ID_SWON]==VALUE_INPUT_SW_ENABLED){  // Power ON

    if (DEBUG_MODE) Serial.println("I'm ON!");

    // Set test-mode values to the default
    testModeCylinderPin=VALUE_PIN_NULL;
    
    if (digitalInputs[ID_SWAUTO]==VALUE_INPUT_SW_DISABLED){ // Manual mode
      if (DEBUG_MODE) Serial.println("I'm on MANUAL MODE!");

      // Set auto-mode values to the default
      stage=FAILSAFE_STAGE;
      substage=0;
      movementTimer=VALUE_TIMER_NULL;

      // Apply manual-mode.
      applyManualMode(digitalInputs,flagHighPressure);
      
    }else{                            // AUTO MODE

      if (DEBUG_MODE){
        Serial.println("I'm on AUTO MODE!"); Serial.print("Stage: ");Serial.println(stage); Serial.print(" SubStage: ");Serial.println(substage);
      }
      // Set the proper initial values
      // Checks, if needed.

//      applyAutoMode(digitalInputs, timesArray, stage, substage, flagHighPressure);

      // Being able to move the shaker at any time in auto-mode if the !chronoIsRunning
      if (digitalInputs[ID_BUTTON_SHAKER]==VALUE_INPUT_ENABLED && !chronoIsRunning){
        digitalWrite(PIN_SOLS,VALUE_SOL_ENABLED);
      }else digitalWrite(PIN_SOLS,VALUE_SOL_DISABLED);

      switch(stage){

        case FAILSAFE_STAGE:    // FAILSAFE_STAGE: Startup procedure
    
            if (substage==0 && !flagHighPressure){
              setSolenoids(VALUE_SOL_DISABLED);                                   // switch off the solenoids - as described in the documentation.
                // Release pressure from SOLU if neccesary.
              if (digitalInputs[ID_PRESSURE]==VALUE_HP_ENABLED) {
                auxTimer=releasePressure(PIN_SOLU,flagHighPressure); // TODO: Implement a releaseHighPressureOnAllSolenoids().
              }
              substage++;

            }else if (substage==1 && !flagHighPressure){
              moveCylinderUntilHighPressure(PIN_SOLL, flagHighPressure);          // Clean the platform and goes to the initial position.
              if (flagHighPressure) substage++;

            }else if (substage==2 && !flagHighPressure){
                moveCylinderUntilHighPressure(PIN_SOLU, flagHighPressure);
                if (flagHighPressure) substage++;

            }else if (substage==3 && !flagHighPressure){
              moveCylinderUntilHighPressure(PIN_SOLR, flagHighPressure);
              if (flagHighPressure) substage++;

            }else if (substage==4 && !flagHighPressure){
              stage=CALIBRATE_SOLENOIDS;
              substage=0;
            }
          break;

        case CALIBRATE_SOLENOIDS:       // Get the times we need
            if (substage==0){            // Note: Consider to encapsulate the next feature (chronometer).
              if (!chronoIsRunning){
                timer=millis();
                chronoIsRunning=true;
              }
              moveCylinderUntilHighPressure(PIN_SOLD,flagHighPressure);
              if (flagHighPressure){
                timesArray[ID_TIME_SOLD] = millis() - timer;
                if (DEBUG_MODE) {Serial.print("The for SOLD has been: ");Serial.println(timesArray[ID_TIME_SOLD]);}
                chronoIsRunning=false;
                timer=VALUE_TIMER_NULL;
                substage++;
              }
            }else if (substage==1){
              if (!chronoIsRunning){
                timer=millis();
                chronoIsRunning=true;
              }
              moveCylinderUntilHighPressure(PIN_SOLL,flagHighPressure);
              if (flagHighPressure){
                timesArray[ID_TIME_SOLL] = millis() - timer;
                if (DEBUG_MODE) {Serial.print("The for SOLL has been: ");Serial.println(timesArray[ID_TIME_SOLL]);}
                chronoIsRunning=false;
                timer=VALUE_TIMER_NULL;
                substage=0;
                stage=EJECT_BRICK;
              }
            }
          break;

        // BRICK SEQUENCE
        case EJECT_BRICK: // Open the chamber
            //timesArray[ID_TIME_SOLU] = moveCylinderUntilHighPressureBecomes(PIN_SOLU, flagHighPressure,VALUE_HP_ENABLED);  // This value is not needed right now
            moveCylinderUntilHighPressure(PIN_SOLU, flagHighPressure);
            if (flagHighPressure) stage=PUSH_BRICK;
          break;
        case PUSH_BRICK:
            //timesArray[ID_TIME_SOLR] = moveCylinderUntilHighPressureBecomes(PIN_SOLR, flagHighPressure,VALUE_HP_ENABLED);  // This value is not needed, right now.
            moveCylinderUntilHighPressure(PIN_SOLR, flagHighPressure);
            if (flagHighPressure) stage=LOAD_SOIL;
          break;
        case LOAD_SOIL: // Push down the main cilinder and load the room with soil.

            if (DEBUG_MODE){
              Serial.println("Starting LOAD SOIL stage.");
            }

            // MAIN CYLINDER - POTM behaviour: we want all the time travel
            // Two behaviours dependending on the potM value:
            // digitalInputs[ID_POTM]/VALUE_MAX_POTM < 0.95 ( close to the maximum) we want to move it until High pressure.
            // In other case: go into timed mode.

            // We can't use a coefficient with unsigned long because every result between 0 and 1 will be rounded to 0.
            //if ( (analogInputs[ID_POTM])/(VALUE_MAX_POTM) < 0.95){ // Go into timing mode

            if ( analogInputs[ID_POTM] < 950 ){ // Go into timing mode
  
              // We can't operate with numbers below 1 with unsigned long. So we write the operation in another way. The next two expression should be equal.
              // So what we do is group all the multiplications, group all the divisors together, and then we do the division.
              //  movementTimer = timesArray[ID_TIME_SOLD] * (analogInputs[ID_POTM]/VALUE_MAX_POTM);
              movementTimer = (timesArray[ID_TIME_SOLD] * analogInputs[ID_POTM])/VALUE_MAX_POTM;
              if (DEBUG_MODE){
                Serial.println("Timing mode.");
                Serial.print("Time that is gonna be applied to SOLD and SOLS: ");Serial.println(movementTimer) ;
                //delay(10000);
              }

              if (!chronoIsRunning){
                timer=millis();
                chronoIsRunning=true;
              }
              
            }else{    // Go into until-high-pressure mode
              if (DEBUG_MODE){
                Serial.print("Until-high-pressure-mode.");
              }
              movementTimer = timesArray[ID_TIME_SOLD] * 2; // We double the value of the timer for this solenoid to reach the high pressure point.
              chronoIsRunning=false;
            }
            
            moveCylinderUntilHighPressure(PIN_SOLD, flagHighPressure);
            moveCylinderUntilHighPressure(PIN_SOLS, flagHighPressure);

            if ( (flagHighPressure) || (millis()-timer > movementTimer) ){
              setSolenoids(VALUE_SOL_DISABLED);                
              timer=VALUE_TIMER_NULL;
              chronoIsRunning=false;
              movementTimer=VALUE_TIMER_NULL;
              stage=CLOSE_CHAMBER;
              if (DEBUG_MODE){Serial.println("LOAD_SOIL stage finished. Stop moving SOLS and SOLD.");};
            }

          break;

        case CLOSE_CHAMBER:  // Moves the drawer on the main cylinder

            // We can't operate with numbers below 1 with unsigned long. So we write the operation in another way. The next two expression should be equal.
            //startingPoint = ((1/4)*timesArray[ID_TIME_SOLL]);
            startingPoint = (timesArray[ID_TIME_SOLL]/4);
            //variableTravelTime = ( (1/2) * timesArray[ID_TIME_SOLL] * (analogInputs[ID_POTD] / VALUE_MAX_POTD * 2) );
            variableTravelTime = ( (timesArray[ID_TIME_SOLL] * analogInputs[ID_POTD]) / (VALUE_MAX_POTD * 2) );
            // Maximun value for drawer movementTimer = 3/4 * drawerTravelTime
            movementTimer =  ( startingPoint + variableTravelTime);

            if (!chronoIsRunning){
              if (DEBUG_MODE){
                Serial.print("Drawer travel time: ");Serial.println(timesArray[ID_TIME_SOLL]);
                Serial.print("Drawer starting time: ");Serial.println(startingPoint);
                Serial.print("Drawer variable time: ");Serial.println(variableTravelTime);
                Serial.print("Time that will be applied to SOLL: ");Serial.println(movementTimer) ;
              }
              timestamp=millis();
              chronoIsRunning=true;
            }
            //delay(20000);

            moveCylinderUntilHighPressure(PIN_SOLL, flagHighPressure);

            if ( (flagHighPressure) || (millis()-timestamp > movementTimer) ){
              digitalWrite(PIN_SOLL,VALUE_SOL_DISABLED);
              timestamp=VALUE_TIMER_NULL;
              chronoIsRunning=false;
              stage=COMPRESS_SOIL;
              if (DEBUG_MODE){Serial.println("Stage CLOSE_CHAMBER finished.");}
              //delay(10000);
            }
          break;

        case COMPRESS_SOIL: // Compression stage
            moveCylinderUntilHighPressure(PIN_SOLU, flagHighPressure);
            if (flagHighPressure) stage=OPEN_CHAMBER;
            break;

        case OPEN_CHAMBER: // Open the chamber
            moveCylinderUntilHighPressure(PIN_SOLL, flagHighPressure);
            if (flagHighPressure) stage=EJECT_BRICK;    // Going to stage 0 to get full calibration before each press.
          break;

        default:
            Serial.print("ERROR: Stage not defined. Value of stage = ");Serial.print(stage);
            Serial.print("Going into FAILSAFE_STAGE");
            delay(4000);
            stage=FAILSAFE_STAGE;

          break;
      }
    }

  }else{       // SWON is Disabled -> TEST MODE
    if (DEBUG_MODE) Serial.println("SWON is DISABLED -> I'm on TEST-MODE!");

    // Apply test-mode.
    if (testModeCylinderPin==VALUE_PIN_NULL){
      setSolenoids(VALUE_SOL_DISABLED);      // Init default values for the other modes
      stage=FAILSAFE_STAGE;
      substage=0;
      testModeCylinderPin = getEnabledCylinder(digitalInputs);
    }else{
      moveCylinderUntilHighPressure(testModeCylinderPin,flagHighPressure);
      if (flagHighPressure){
        auxTimer= releasePressure(testModeCylinderPin,flagHighPressure);
        testModeCylinderPin=VALUE_PIN_NULL;
        if (DEBUG_MODE) {Serial.print("It took ");Serial.print(auxTimer); Serial.print(" to relase th pressure from pin ");Serial.println(testModeCylinderPin);}
      }
    } 

  }

//  if (DEBUG_MODE) delay(1000);

  }
