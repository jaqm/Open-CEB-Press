
// 

// loop() variables
int stage=0;       // Defines the stage for the auto-mode
const unsigned long VALUE_INPUT_READ_DELAY = 5;  // Delay (milliseconds) used to consider a stable input read.
const unsigned long VALUE_TIME_RELEASE_PRESSURE_STAGE = 500;

// Debug mode
const boolean DEBUG_MODE=true;
const boolean DEBUG_VERBOSE_MODE=false;

// STANDARD VALUES
// for inputs
const uint8_t VALUE_INPUT_ENABLED = LOW;
const uint8_t VALUE_INPUT_DISABLED = HIGH;
// for Solenoids
const uint8_t VALUE_SOLENOIDS_ENABLED=LOW;
const uint8_t VALUE_SOLENOIDS_DISABLED=HIGH;
// for leds
const uint8_t VALUE_LED_ENABLED = LOW;
const uint8_t VALUE_LED_DISABLED = HIGH;

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
int PIN_LED_ON=PIN_E0;
int PIN_LED_AUTO=PIN_E1;
//int PIN_LED_GND=?
//int PIN_LED_BUTTON_UP=?
//int PIN_LED_BUTTON_DOWN=?
//int PIN_LED_BUTTON_LEFT=?
//int PIN_LED_BUTTON_RIGHT=?
//int PIN_LED_BUTTON_SHAKER=?

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
uint8_t inputIs(int pin, int d){

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

// **** END of GETTERS && SETTERS

// **** DATA HANDLING

// This function inverts a digital value read from a pin like HIGH or LOW.
// val: HIGH or LOW
// return: if val=HGH then return LOW, if val=LOW then return HIGH. Default: B00001111.
//uint8_t revertDigitalSignalValue(uint8_t val){
//
//  uint8_t oppositeValue=B00001111;
//  
//  if (val==HIGH){
//    oppositeValue=LOW;
//  }else if (val==LOW){
//    oppositeValue=HIGH;
//  }
//  return oppositeValue;
//}

// **** END of DATA HANDLING
// **** MACHINE MOVEMENTS

// Moves the cylinder to the top position and returns the time it gets to reach that place.
unsigned long moveCylinderUntilHighPressure(int cylinderPin){

  // Variable used to compare 
  unsigned long timestamp=millis();

  digitalWrite(cylinderPin,HIGH);                // Cilinder movement.
  while(inputIs(PIN_PRESSURE,1)==LOW){}          //
  digitalWrite(cylinderPin,LOW);

  return (millis()-timestamp);
}

void moveCylinderDuring(uint8_t cylinderPin,unsigned long time){

  unsigned long timestamp=millis();
  
  digitalWrite(cylinderPin,HIGH);                // Cylinder movement.
  while ( (inputIs(PIN_PRESSURE,1)==LOW)  && (timestamp+time > millis()) ){}          //
  digitalWrite(cylinderPin,LOW);
}

// Initial point is considered for both cylinders as near as possible to the high-pressure point of SOLU and SOLD.
void goToTheInitialPosition(){

  setSolenoids(VALUE_SOLENOIDS_DISABLED);
  moveCylinderUntilHighPressure(PIN_SOLR);
  moveCylinderUntilHighPressure(PIN_SOLU);
}

// **** END OF MACHINE MOVEMENTS

// **** MACHINE MODES

// Applies the actions expected in manual mode following the data stored in array.
void applyManualMode(uint8_t array[]){

  digitalWrite(PIN_SOLU,(array[ID_BUTTON_UP]==VALUE_INPUT_ENABLED ? VALUE_SOLENOIDS_ENABLED:VALUE_SOLENOIDS_DISABLED));
  digitalWrite(PIN_SOLD,(array[ID_BUTTON_DOWN]==VALUE_INPUT_ENABLED ? VALUE_SOLENOIDS_ENABLED:VALUE_SOLENOIDS_DISABLED));
  digitalWrite(PIN_SOLL,(array[ID_BUTTON_LEFT]==VALUE_INPUT_ENABLED ? VALUE_SOLENOIDS_ENABLED:VALUE_SOLENOIDS_DISABLED));
  digitalWrite(PIN_SOLR,(array[ID_BUTTON_RIGHT]==VALUE_INPUT_ENABLED ? VALUE_SOLENOIDS_ENABLED:VALUE_SOLENOIDS_DISABLED));
  digitalWrite(PIN_SOLS,(array[ID_BUTTON_SHAKER]==VALUE_INPUT_ENABLED ? VALUE_SOLENOIDS_ENABLED:VALUE_SOLENOIDS_DISABLED));

}

// Moves both cylinders during the specified time or until HIGH PRESSURE.
void moveBothCylinderDuring(uint8_t cylinderPin1, uint8_t cylinderPin2, unsigned long timeMoving){

  unsigned long timestamp=millis();
  
  digitalWrite(cylinderPin1,VALUE_SOLENOIDS_ENABLED);
  digitalWrite(cylinderPin2,VALUE_SOLENOIDS_ENABLED);
  while ( (inputIs(PIN_PRESSURE,VALUE_INPUT_READ_DELAY)==VALUE_INPUT_DISABLED) && (timestamp+timeMoving > millis())){}
  digitalWrite(cylinderPin1,VALUE_SOLENOIDS_DISABLED);
  digitalWrite(cylinderPin2,VALUE_SOLENOIDS_DISABLED);

}

// Applies the auto-mode.
// panel[]: the information readed from the machine.
// stage: which stage of the auto-mode do we want to run.
void applyAutoMode(uint8_t panel[], unsigned long times[], int &stage){

  switch(stage){
    case 0:    // INITIAL STAGE: Initial position
        goToTheInitialPosition();
        stage++;
      break;
    case 1:       // Fulfill the times array.
        times[ID_TIME_SOLL] = moveCylinderUntilHighPressure(PIN_SOLL);
        times[ID_TIME_SOLD] = moveCylinderUntilHighPressure(PIN_SOLD);
        times[ID_TIME_SOLU] = moveCylinderUntilHighPressure(PIN_SOLU);
        times[ID_TIME_SOLR] = moveCylinderUntilHighPressure(PIN_SOLR);
        stage++;
      break;
    // BRICK SEQUENCE
    case 2:    // Push down the main cilinder and fulfill the room with sand.
        if (DEBUG_MODE){Serial.print("Time applied to SOLD: ");Serial.println(timesArray[ID_TIME_SOLD]*(panelArray[ID_POTM]/VALUE_MAX_POTM) ) ;}
        //
        moveBothCylinderDuring(PIN_SOLD, PIN_SOLS, (timesArray[ID_TIME_SOLD])*(panelArray[ID_POTM]/VALUE_MAX_POTM));
        stage++;
      break;
    case 3: // Moves the drawer on the main cylinder
        moveCylinderDuring(PIN_SOLL, (timesArray[ID_TIME_SOLL])*(panelArray[ID_POTD]/VALUE_MAX_POTD));
        stage++;
      break;
    case 4:  // Compression stage
        moveCylinderUntilHighPressure(PIN_SOLU);
        stage++;
      break;
    case 5: // Release pressure stage. 
        moveCylinderDuring(PIN_SOLD,VALUE_TIME_RELEASE_PRESSURE_STAGE);
        stage++;
        break;
    case 6: // Take out the brick
        moveCylinderUntilHighPressure(PIN_SOLL);
        moveCylinderUntilHighPressure(PIN_SOLU);
        moveCylinderUntilHighPressure(PIN_SOLR);
        stage=2;
      break;
    default:
        stage=0;
      break;
  }
}

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
  uint8_t vPotM0 = digitalRead(PIN_POTM);
  uint8_t vPotD0 = digitalRead(PIN_POTD);
  
  delay(d);
  
  uint8_t vU1 = digitalRead(PIN_BUTTON_UP);
  uint8_t vD1 = digitalRead(PIN_BUTTON_DOWN);
  uint8_t vR1 = digitalRead(PIN_BUTTON_RIGHT);
  uint8_t vL1 = digitalRead(PIN_BUTTON_LEFT);
  uint8_t vS1 = digitalRead(PIN_BUTTON_SHAKER);
  uint8_t vP1 = digitalRead(PIN_PRESSURE);
  uint8_t vSwOn1 = digitalRead(PIN_SWON);
  uint8_t vSwAuto1 = digitalRead(PIN_SWAUTO);
  uint8_t vPotM1 = digitalRead(PIN_POTM);
  uint8_t vPotD1 = digitalRead(PIN_POTD);

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
  Serial.print("Main Potentiometer: "); Serial.println(panel[ID_POTM],BIN);
  Serial.print("Drawer potentiometer: "); Serial.println(panel[ID_POTD],BIN);
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

// Updates the leds according to the information from the panel.
// NOTE: we are directly reverting the digital signal as we are using the opposite value
// to activate outputs and inputs. !!
// CONSIDER: reimplement this function using constant values defined in the begining of this code. 
void updateLeds(uint8_t panel[]){

// Old implementation
//  digitalWrite(PIN_LED_ON,revertDigitalSignalValue(panel[ID_SWON]));
//  digitalWrite(PIN_LED_AUTO,revertDigitalSignalValue(panel[ID_SWAUTO]));

  // New implementation
  digitalWrite(PIN_LED_ON,(panel[ID_SWON]==VALUE_INPUT_ENABLED)?VALUE_LED_ENABLED:VALUE_LED_DISABLED);
  digitalWrite(PIN_LED_AUTO,(panel[ID_SWAUTO]==VALUE_INPUT_ENABLED)?VALUE_LED_ENABLED:VALUE_LED_DISABLED);
//  digitalWrite(PIN_LED_ON,revertDigitalSignalValue(panel[ID_SWON]));
//  digitalWrite(PIN_LED_AUTO,revertDigitalSignalValue(panel[ID_SWAUTO]));
//  digitalWrite(PIN_LED_ON,revertDigitalSignalValue(panel[ID_SWON]));
//  digitalWrite(PIN_LED_AUTO,revertDigitalSignalValue(panel[ID_SWAUTO]));

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

    pinMode(PIN_LED_ON, OUTPUT);
    pinMode(PIN_LED_AUTO, OUTPUT);

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

    stage=0;

    Serial.begin(9600);
}

void loop() {

  readPanel(panelArray,VALUE_INPUT_READ_DELAY);
  updateLeds(panelArray);
  if (DEBUG_MODE){ printPanel(panelArray); printTimesArray(timesArray);};

  if (panelArray[ID_SWON]==LOW){  // Power ON
    if (DEBUG_MODE) Serial.println("I'm ON!");
    digitalWrite(PIN_LED_ON,LOW);
    
    if (panelArray[ID_SWAUTO]==HIGH){ // Manual mode
      if (DEBUG_MODE) Serial.println("I'm on MANUAL MODE!");

      // Set auto-mode values to the default
      stage=0;

      // Apply manual-mode.
      applyManualMode(panelArray);

    }else{                            // Auto mode
      if (DEBUG_MODE) Serial.println("I'm on AUTO MODE!");
      if (DEBUG_MODE){ Serial.print("I'm on stage");Serial.println(stage,DEC);}      
      // Set the proper initial values
      // Checks, if needed.

      applyAutoMode(panelArray, timesArray, stage);
    }
  
  }else{                              // Power OFF
    if (DEBUG_MODE) Serial.println("I'm OFF!");
    setSolenoids(VALUE_SOLENOIDS_DISABLED);
    stage=0;

  }
  if (DEBUG_MODE) delay(1000);  

}

