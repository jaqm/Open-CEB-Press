

// loop() variables
short stage;       // Defines the stage for the auto-mode.
short substage;
unsigned long blinkingStatusTimer=millis();  // Timer to track the blinking procedure.
unsigned long blinkingHighPressureTimer=0;  // Timer to track the blinking procedure for pressure timer.
boolean flagHighPressure=false; // Flag to track if it was received a highPressure signal.
unsigned long timer=0;          // Timer to track times.
unsigned long movementTimer=0;          // Used to calculate the time that is being applied to move a cylinder.
boolean chronoIsRunning=false;

// Debug mode
const boolean DEBUG_MODE=true;
const boolean DEBUG_VERBOSE_MODE=false;
const boolean DEBUG_LED_MODE=false;

// STANDARD VALUES
// inputs
const uint8_t VALUE_INPUT_ENABLED = LOW;
const uint8_t VALUE_INPUT_DISABLED = HIGH;
// inputs - high pressure sensor
//// Real values
const uint8_t VALUE_HP_ENABLED = LOW;
const uint8_t VALUE_HP_DISABLED = HIGH;
//// For testing auto-mode purpose.
//const uint8_t VALUE_HP_ENABLED = HIGH;
//const uint8_t VALUE_HP_DISABLED = LOW;
// outputs - Solenoids
const uint8_t VALUE_SOL_ENABLED=LOW;
const uint8_t VALUE_SOL_DISABLED=HIGH;
// outputs - leds
const uint8_t VALUE_LED_ENABLED = LOW;
const uint8_t VALUE_LED_DISABLED = HIGH;
const uint8_t VALUE_LED_HIGHPRESSURE_ENABLED = HIGH;
const uint8_t VALUE_LED_HIGHPRESSURE_DISABLED = LOW;

// CONST - LED timing for blinking
const unsigned long VALUE_TIME_BLINKING_MANUAL=1000;
const unsigned long VALUE_TIME_BLINKING_AUTO=250;
const unsigned long VALUE_TIME_BLINKING_HIGH_PRESSURE=500;
//CONST
const unsigned long VALUE_INPUT_READ_DELAY = 5;  // Delay (milliseconds) used to consider a stable input read.
const unsigned long VALUE_HP_READ_DELAY = 3;
const unsigned long VALUE_TIME_RELEASE_PRESSURE_STAGE = 100;
// CONST - STATES OF THE AUTO-MODE
const short FAILSAFE_STAGE=0;
const short CALIBRATE_SOLENOIDS = 1;
const short EJECT_BRICK = 2;
const short PUSH_BRICK = 3;
const short LOAD_SOIL = 4;
const short CLOSE_CHAMBER = 5;
const short COMPRESS_SOIL = 6;
const short OPEN_CHAMBER = 7;


// VALUE_MAX_POTM=2^8 ; because a int is compound by 8 bits.
const int VALUE_MAX_POTM=255;
const int VALUE_MAX_POTD=VALUE_MAX_POTM;

// INPUTS
int PIN_SWON=PIN_C7;    //on/off switch
int PIN_SWAUTO=PIN_C6;    //auto/manual switch
int PIN_BUTTON_UP=PIN_C5;    //button for up
int PIN_BUTTON_DOWN=PIN_C4;    //button for down
int PIN_BUTTON_LEFT=PIN_C3;    //button for left
int PIN_BUTTON_RIGHT=PIN_C2;    //button for right
int PIN_BUTTON_SHAKER=PIN_C1;    //button for shaker
int PIN_POTD=PIN_F2;    //potentiometer for the drawer
int PIN_POTM=PIN_F1;    //potentiometer for the main cylinder
int PIN_PRESSURE=PIN_F0;

// OUTPUTS - Solenoids
int PIN_SOLU=PIN_B6;    //solenoid for cylinder up
int PIN_SOLD=PIN_B5;    //solenoid for cylinder down
int PIN_SOLL=PIN_B4;    //solenoid for drawer left
int PIN_SOLR=PIN_B3;    //solenoid for drawer right
int PIN_SOLS=PIN_B2;    //solenoid for shaker motor 

// OUTPUTS - leds
int PIN_LED_STATUS=PIN_E0;
int PIN_LED_HIGH_PRESSURE=PIN_E1;

// PANEL ARRAY - it contains all the input panel values.
const int ID_SWON=0;
const int ID_SWAUTO=1;
const int ID_BUTTON_UP=2;
const int ID_BUTTON_DOWN=3;
const int ID_BUTTON_LEFT=4;
const int ID_BUTTON_RIGHT=5;
const int ID_BUTTON_SHAKER=6;
const int ID_POTD=7;
const int ID_POTM=8;
const int ID_PRESSURE=9;
const int SENSORS_AMOUNT=10;
uint8_t panelArray[SENSORS_AMOUNT];  // The array which contains all the input panel variables.

// TIMESARRAY
unsigned long timesArray[]={0,0,0,0,0};
const int ID_TIME_SOLU=0;
const int ID_TIME_SOLD=1;
const int ID_TIME_SOLL=2;
const int ID_TIME_SOLR=3;
const int ID_TIME_SOLS=4;

//**********************************
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

    uint8_t value0 = HIGH;
    uint8_t value1 = LOW;

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

uint8_t getOppositeSolenoid(uint8_t pinSol){
  
  uint8_t opposite=-1;
  if (pinSol==PIN_SOLU){opposite=PIN_SOLD;}
  else if (pinSol==PIN_SOLD){opposite=PIN_SOLU;}
  else if (pinSol==PIN_SOLL){opposite=PIN_SOLR;}
  else if (pinSol==PIN_SOLR){opposite=PIN_SOLL;}
  else if (pinSol==PIN_SOLS){opposite=PIN_SOLS;}
  else Serial.println("ERROR: Unexpected pin value received by getOppositeSolenoid()");
  return opposite;
}

// **** END of GETTERS && SETTERS

// **** DATA HANDLING

// Receiving the values related to the potentiometer and the total time, returns the value that should be applied to the cylinder.
unsigned long getTimeFromPotentiometer(unsigned long totalTime, uint8_t potValue,uint8_t maxPotValue){
  unsigned long result=( (3/4) * totalTime * (potValue/maxPotValue) );
  if (DEBUG_MODE) {Serial.print("The time calculated is: ");Serial.println(result);}
  return result;
}


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
 return oppositeValue;
}

// **** END of DATA HANDLING
// **** MACHINE MOVEMENTS

//// Moves the cylinder until high pressure sensor reaches the value secified and returns the time itgets to reach that place.
//// cylinderPin: the pin assigned to the cylinder we want to move.
//// &hpf: high pressure flag.
//unsigned long moveCylinderUntilHPtakeTime(int cylinderPin, boolean &hpf){
//
//  // Variable used to compare 
//  unsigned long timestamp=millis();
//
//  // Debug Mode
//  if (DEBUG_MODE){
//    Serial.println("moveCylinderUntilHPtakeTime: ");
//    Serial.print("CylinderPin: "); Serial.println(cylinderPin);
//  }
//
//  digitalWrite(cylinderPin,VALUE_SOL_ENABLED);                // Cilinder movement.
//  while(pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)!=VALUE_HP_ENABLED){}
//  hpf=true;
//  digitalWrite(cylinderPin,VALUE_SOL_DISABLED);
//
//  return (millis()-timestamp);
//}

// Moves the cylinder to the top position and returns the time itgets to reach that place.
// cylinderPin: the pin assigned to the cylinder we want to move.
// &hpf: high pressure flag.
void moveCylinderUntilHighPressure(int cylinderPin, boolean &hpf){

  // Variable used to compare 
//  unsigned long timestamp=millis();
//  unsigned long result=0;
  
  // Debug Mode
  if (DEBUG_MODE){
    Serial.println("moveCylinderUntilHighPressure: ");
    Serial.print("CylinderPin: "); Serial.println(cylinderPin);
  }

  digitalWrite(cylinderPin,VALUE_SOL_ENABLED);                // Cilinder movement.
  if(pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)==VALUE_HP_ENABLED){
    hpf=true;
    digitalWrite(cylinderPin,VALUE_SOL_DISABLED);
//    result=(millis()-timestamp);
  
    moveCylinderUntilHighPressureBecomes( getOppositeSolenoid(cylinderPin),hpf,VALUE_HP_DISABLED);  // Release pressure
  
//    return result;
  }          //
}

// Moves the cylinder until high pressure sensor reaches the value secified and returns the time itgets to reach that place.
// cylinderPin: the pin assigned to the cylinder we want to move.
// &hpf: high pressure flag.
// hpv: the High Pressure Value we want to reach.
unsigned long moveCylinderUntilHighPressureBecomes(int cylinderPin, boolean &hpf, uint8_t hpv){

  // Variable used to compare 
  unsigned long timestamp=millis();
  uint8_t highPressure=revertDigitalSignalValue(hpv);

  // Debug Mode
  if (DEBUG_MODE){
    Serial.println("moveCylinderUntilHighPressure: ");
    Serial.print("CylinderPin: "); Serial.println(cylinderPin);
  }

  digitalWrite(cylinderPin,VALUE_SOL_ENABLED);                // Cilinder movement.
  while(pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)!=hpv){}
  if (pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)==VALUE_HP_ENABLED){hpf=true;}
  digitalWrite(cylinderPin,VALUE_SOL_DISABLED);

  return (millis()-timestamp);
}

// Moves the cylinder during the time specified.
// cylinderPin: the pin assigned to the cylinder.
// time: the time (milliseconds) that we want to move the cylinder.
// &hpf: high pressure flag.
void moveCylinderDuring(uint8_t cylinderPin,unsigned long time, boolean &hpf){

  unsigned long timestamp=millis();
  
  digitalWrite(cylinderPin,VALUE_SOL_ENABLED);                // Cylinder movement.
  while ( (pinDigitalValueIs(PIN_PRESSURE,VALUE_HP_READ_DELAY)==VALUE_HP_DISABLED)  && (timestamp+time > millis()) ){}
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

// Initial point is considered for both cylinders as near as possible to the high-pressure point of SOLU and SOLD.
// &hpf: high pressure flag
//void goToTheInitialPosition(boolean &hpf){
//
//  setSolenoids(VALUE_SOL_DISABLED);
//  moveCylinderUntilHighPressure(PIN_SOLR, hpf);
//  moveCylinderUntilHighPressure(PIN_SOLU, hpf);
//}

// Release the pressure from the solenoids using the data received from the panel.
// ONLY APPLIES FOR MANUAL MODE.
// array[]: contains the information from the panel.
void releasePressureManualMode(uint8_t array[]){

  if (pinDigitalValueIs(PIN_PRESSURE, VALUE_INPUT_READ_DELAY)==VALUE_HP_ENABLED){
    if (DEBUG_MODE){Serial.println("Applying reversal data movement to solenoids..");}

    digitalWrite(PIN_SOLD,(array[ID_BUTTON_UP]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLU,(array[ID_BUTTON_DOWN]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLR,(array[ID_BUTTON_LEFT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLL,(array[ID_BUTTON_RIGHT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
//    digitalWrite(PIN_SOLS,(array[ID_BUTTON_SHAKER]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));  
    delay(VALUE_TIME_RELEASE_PRESSURE_STAGE);
    setSolenoids(VALUE_SOL_DISABLED);

  }
}

// **** END OF MACHINE MOVEMENTS

// **** MACHINE MODES

// Applies the actions expected in manual mode following the data stored in array.
// array[]: contains the information from the panel.
// &hpf: flag to track the high pressure sensor.
void applyManualMode(uint8_t array[], boolean &hpf){

  if (pinDigitalValueIs(PIN_PRESSURE, VALUE_INPUT_READ_DELAY)==VALUE_HP_DISABLED){
    if (DEBUG_MODE){Serial.println("Applying movement data to solenoids..");}
    digitalWrite(PIN_SOLU,(array[ID_BUTTON_UP]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLD,(array[ID_BUTTON_DOWN]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLL,(array[ID_BUTTON_LEFT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLR,(array[ID_BUTTON_RIGHT]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));
    digitalWrite(PIN_SOLS,(array[ID_BUTTON_SHAKER]==VALUE_INPUT_ENABLED ? VALUE_SOL_ENABLED:VALUE_SOL_DISABLED));  
  }else {
    if (DEBUG_MODE){Serial.println("Warning: high pressure signal detected. Switching off solenoids.");}
    hpf=true;
    setSolenoids(VALUE_SOL_DISABLED);
  }
}

// Applies the auto-mode.
// panel[]: the information readed from the machine.
// stage: which stage of the auto-mode do we want to run.
// &hpf: high pressure flag.
//void applyAutoMode(uint8_t panel[], unsigned long times[], short &stage, short &substage, boolean &hpf){
//}

// **** END OF MACHINE MODES
// **** READ && SHOW FUNCTIONS

// Reads all the values of the panel, adding a check protection against rebounce, with a delay.
void readPanel(uint8_t panelArray[], const int d){  
  
  uint8_t vU0 = digitalRead(PIN_BUTTON_UP);
  uint8_t vD0 = digitalRead(PIN_BUTTON_DOWN);
  uint8_t vL0 = digitalRead(PIN_BUTTON_LEFT);
  uint8_t vR0 = digitalRead(PIN_BUTTON_RIGHT);
  uint8_t vS0 = digitalRead(PIN_BUTTON_SHAKER);
  uint8_t vP0 = digitalRead(PIN_PRESSURE);
  uint8_t vSwOn0 = digitalRead(PIN_SWON);
  uint8_t vSwAuto0 = digitalRead(PIN_SWAUTO);
  uint8_t vPotM0 = analogRead(PIN_POTM);
  uint8_t vPotD0 = analogRead(PIN_POTD);
  
  delay(d);
  
  uint8_t vU1 = digitalRead(PIN_BUTTON_UP);
  uint8_t vD1 = digitalRead(PIN_BUTTON_DOWN);
  uint8_t vR1 = digitalRead(PIN_BUTTON_RIGHT);
  uint8_t vL1 = digitalRead(PIN_BUTTON_LEFT);
  uint8_t vS1 = digitalRead(PIN_BUTTON_SHAKER);
  uint8_t vP1 = digitalRead(PIN_PRESSURE);
  uint8_t vSwOn1 = digitalRead(PIN_SWON);
  uint8_t vSwAuto1 = digitalRead(PIN_SWAUTO);
  uint8_t vPotM1 = analogRead(PIN_POTM);
  uint8_t vPotD1 = analogRead(PIN_POTD);

  panelArray[ID_BUTTON_UP] = (vU0==vU1?vU0:VALUE_INPUT_DISABLED);
  panelArray[ID_BUTTON_DOWN] = (vD0==vD1?vD0:VALUE_INPUT_DISABLED);
  panelArray[ID_BUTTON_LEFT] = (vL0==vL1?vL0:VALUE_INPUT_DISABLED);
  panelArray[ID_BUTTON_RIGHT] = (vR0==vR1?vR0:VALUE_INPUT_DISABLED);
  panelArray[ID_BUTTON_SHAKER] = (vS0==vS1?vS0:VALUE_INPUT_DISABLED);
  panelArray[ID_PRESSURE] = (vP0==vP1?vP0:VALUE_INPUT_DISABLED);
  panelArray[ID_SWON] = (vSwOn0==vSwOn1?vSwOn0:VALUE_INPUT_DISABLED);
  panelArray[ID_SWAUTO] = (vSwAuto0==vSwAuto0?vSwAuto0:VALUE_INPUT_DISABLED);
  panelArray[ID_POTM] = (vPotM0==vPotM1?vPotM0:VALUE_INPUT_DISABLED);
  panelArray[ID_POTD] = (vPotD0==vPotD1?vPotD0:VALUE_INPUT_DISABLED);

}

void printPanel(uint8_t panel[]){

  Serial.println("********************************************");  
  Serial.print("Switch ON: "); Serial.println(panel[ID_SWON],DEC);
  Serial.print("Switch AUTO: "); Serial.println(panel[ID_SWAUTO],DEC);
  Serial.print("Button UP: "); Serial.println(panel[ID_BUTTON_UP],DEC);
  Serial.print("Button DOWN: "); Serial.println(panel[ID_BUTTON_DOWN],DEC);
  Serial.print("Button LEFT: "); Serial.println(panel[ID_BUTTON_LEFT],DEC);
  Serial.print("Button RIGHT: "); Serial.println(panel[ID_BUTTON_RIGHT],DEC);
  Serial.print("Button SHAKER: "); Serial.println(panel[ID_BUTTON_SHAKER],DEC);
  Serial.print("High PRESSURE: "); Serial.println(panel[ID_PRESSURE],DEC);
  Serial.print("Main Potentiometer: "); Serial.println(panel[ID_POTM],DEC);
  Serial.print("Drawer potentiometer: "); Serial.println(panel[ID_POTD],DEC);
  Serial.println("********************************************"); 

}

void printTimesArray(unsigned long ta[]){

  Serial.println("********************************************");
  Serial.print("Time SOLU: "); Serial.println(ta[ID_TIME_SOLU],DEC);
  Serial.print("Time SOLD: "); Serial.println(ta[ID_TIME_SOLD],DEC);
  Serial.print("Time SOLL: "); Serial.println(ta[ID_TIME_SOLL],DEC);
  Serial.print("Time SOLR: "); Serial.println(ta[ID_TIME_SOLR],DEC);
  Serial.print("Time SOLS: "); Serial.println(ta[ID_TIME_SOLS],DEC);
  Serial.println("********************************************"); 

}

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

  // Blinking procedure - STATUS LED
  // Note: Consider taking out this variable
  unsigned long timer=((panel[ID_SWAUTO]==VALUE_INPUT_ENABLED)?VALUE_TIME_BLINKING_AUTO:VALUE_TIME_BLINKING_MANUAL);
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
//      if (DEBUG_LED_MODE){Serial.println("LED STATUS IS GONNA BE UPDATED");delay(1000);}
//      value=pinDigitalValueIs(PIN_LED_STATUS, VALUE_INPUT_READ_DELAY);
//      value1=revertDigitalSignalValue(value);
//      if(DEBUG_LED_MODE){
//        Serial.print("Digital value read from LED STATUS: ");Serial.println(value);
//        Serial.print("Digital value that will be written to LED STATUS: ");Serial.println(value1);
//      }
      digitalWrite(PIN_LED_STATUS,revertDigitalSignalValue(pinDigitalValueIs(PIN_LED_STATUS, VALUE_INPUT_READ_DELAY)) );
      sbt=presentTime;
      if (DEBUG_LED_MODE){Serial.println("LED STATUS HAS BEEN UPDATED"); delay(1000);}
  }

  // Blinking procedure - HIGH PRESSURE LED
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
    hpf=false;
    if (DEBUG_LED_MODE){Serial.println("HIGH PRESSURE LED HAS BEEN UPDATED.");delay(10000);}

  }else if(presentTime>hpt+VALUE_TIME_BLINKING_HIGH_PRESSURE){

    if (DEBUG_LED_MODE){Serial.println("HIGH PRESSURE FLAG OFF.");}
    digitalWrite(PIN_LED_HIGH_PRESSURE,VALUE_LED_HIGHPRESSURE_DISABLED);

  }
}

// **** END OF - READ && SHOW FUNCTIONS

// *** DOWN FROM HERE - FUNCTIONS UNDER REVIEW
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

    stage=FAILSAFE_STAGE;
    substage=0;
    blinkingStatusTimer=millis();
    blinkingHighPressureTimer=0;
    flagHighPressure=false;
    timer=millis();
    chronoIsRunning=false;

    Serial.begin(9600);
}

void loop() {

  readPanel(panelArray,VALUE_INPUT_READ_DELAY);
  updateLeds(panelArray, blinkingStatusTimer, flagHighPressure, blinkingHighPressureTimer);
  if (DEBUG_MODE){ printPanel(panelArray); printTimesArray(timesArray);};

  if (panelArray[ID_SWON]==VALUE_INPUT_ENABLED){  // Power ON
    if (DEBUG_MODE) Serial.println("I'm ON!");
    
    if (panelArray[ID_SWAUTO]==VALUE_INPUT_DISABLED){ // Manual mode
      if (DEBUG_MODE) Serial.println("I'm on MANUAL MODE!");

      // Set auto-mode values to the default
      stage=FAILSAFE_STAGE;
      substage=0;

      // Apply manual-mode.
      applyManualMode(panelArray,flagHighPressure);
      releasePressureManualMode(panelArray);
      
    }else{                            // Auto mode
      if (DEBUG_MODE){
        Serial.println("I'm on AUTO MODE!"); Serial.print("Stage: ");Serial.println(stage); Serial.print(" SubStage: ");Serial.println(substage);
      }
      // Set the proper initial values
      // Checks, if needed.

//      applyAutoMode(panelArray, timesArray, stage, substage, flagHighPressure);
      switch(stage){
        case FAILSAFE_STAGE:    // FAILSAFE_STAGE: Startup procedure
    
            setSolenoids(VALUE_SOL_DISABLED);                                   // switch off the solenoids - as described in the documentation.
            moveCylinderDuring(PIN_SOLD,VALUE_TIME_RELEASE_PRESSURE_STAGE, flagHighPressure);	  // Release pressure
            if (substage==0 && !flagHighPressure){
      
              moveCylinderUntilHighPressure(PIN_SOLL, flagHighPressure);          // Clean the platform and goes to the initial position.
      
              if (flagHighPressure) substage++;
            }else if (substage==1 && !flagHighPressure){
    
                moveCylinderUntilHighPressure(PIN_SOLU, flagHighPressure);
    
                if (flagHighPressure) substage++;
            }else if (substage==2 && !flagHighPressure){
    
              moveCylinderUntilHighPressure(PIN_SOLR, flagHighPressure);
              if (flagHighPressure) substage++;

            }else if (substage==3 && !flagHighPressure){
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
                timer=0;
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
                timer=0;
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

            //movementTimer=timesArray[ID_TIME_SOLD]*(panelArray[ID_POTM]/VALUE_MAX_POTM);
            movementTimer=getTimeFromPotentiometer(timesArray[ID_TIME_SOLD],panelArray[ID_POTD],VALUE_MAX_POTM);
            
            if (DEBUG_MODE){Serial.print("Time applied to SOLD and SOLS: ");Serial.println(movementTimer) ;}//delay(1000);
            
            //moveBothCylinderDuring(PIN_SOLD, PIN_SOLS, (timesArray[ID_TIME_SOLD])*(panelArray[ID_POTM]/VALUE_MAX_POTM));
            if (!chronoIsRunning){
              timer=millis();
              chronoIsRunning=true;
            }
            
            moveCylinderUntilHighPressure(PIN_SOLD, flagHighPressure);
            moveCylinderUntilHighPressure(PIN_SOLS, flagHighPressure);

            if ( (flagHighPressure) || (movementTimer<millis()-timer) ){
            
              timer=0;
              chronoIsRunning=false;
              stage=CLOSE_CHAMBER;

              if (DEBUG_MODE){Serial.println("Stage LOAD_SOIL finished. Stop moving SOLS and SOLD.");};
            }
          break;

        case CLOSE_CHAMBER:  // Moves the drawer on the main cylinder
//            moveCylinderDuring(PIN_SOLL, (timesArray[ID_TIME_SOLL])*(panelArray[ID_POTD]/VALUE_MAX_POTD), flagHighPressure);
            //movementTimer=timesArray[ID_TIME_SOLL]*(panelArray[ID_POTD]/VALUE_MAX_POTD);
            movementTimer=getTimeFromPotentiometer(timesArray[ID_TIME_SOLL],panelArray[ID_POTD],VALUE_MAX_POTD);

            if (!chronoIsRunning){
              if (DEBUG_MODE){Serial.print("Time applied to SOLL: ");Serial.println(movementTimer) ;}//delay(1000);
              timer=millis();
              chronoIsRunning=true;
            }

            moveCylinderUntilHighPressure(PIN_SOLL, flagHighPressure);

            if ( (flagHighPressure) || (movementTimer<millis()-timer) ){

              timer=0;
              chronoIsRunning=false;
              stage=COMPRESS_SOIL;
              if (DEBUG_MODE){Serial.println("Stage CLOSE_CHAMBER finished.");}
            }
          break;

        case COMPRESS_SOIL: // Compression stage
            moveCylinderUntilHighPressure(PIN_SOLU, flagHighPressure);
    //        moveCylinderDuring(PIN_SOLD,(timesArray[ID_TIME_SOLD])*(panelArray[ID_POTM]/VALUE_MAX_POTM), hpf);
            if (flagHighPressure) stage=OPEN_CHAMBER;
            break;
        case OPEN_CHAMBER: // Open the chamber
            moveCylinderUntilHighPressure(PIN_SOLL, flagHighPressure);
            if (flagHighPressure) stage=EJECT_BRICK;    // Going to stage 0 to get full calibration before each press.
          break;
        default:
            stage=FAILSAFE_STAGE;
          break;
      }

    }
  
  }else{                              // Power OFF
    if (DEBUG_MODE) Serial.println("I'm OFF!");
    setSolenoids(VALUE_SOL_DISABLED);
    stage=FAILSAFE_STAGE;
    substage=0;

  }
  if (DEBUG_MODE) delay(1000);

}

