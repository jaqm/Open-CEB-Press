#include "Arduino.h"

// OUTPUTS are considered active when LOW
// INPUTS are considered active when HIGH

// TODO: REVIEW ACTIVATION VALUES!!
// By the way using: LEDs <- active when LOW
// solenoids: active when HIGH
// 

// OUTPUTS - Solenoids
int PIN_SOLU=PIN_B6;    //solenoid for cylinder up
int PIN_SOLD=PIN_B5;    //solenoid for cylinder down
int PIN_SOLL=PIN_B4;    //solenoid for drawer left
int PIN_SOLR=PIN_B3;    //solenoid for drawer right
int PIN_SOLS=PIN_B2;    //solenoid for shaker motor 

// OUTPUTS - leds
int PIN_LED_ON=PIN_E0;
int PIN_LED_AUTO=PIN_E1;

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

const int panelDelay = 1;

// UP FROM HERE - VARUABLES ALREADY REVIEWED
// DOWN FROM HERE - VARIABLES REVIEW PENDING

unsigned long delaytime=500; //500ms - half second blink of pressure sensor indicator

int dRetractionTime = -1; //A vanlue of -1 means that we have not yet recorded a value for dRetractionTime
int mExtensionTime = -1;

unsigned long ledAStartTime=0;
unsigned long prestime=0;

boolean automode=false;        //automode starts at off
boolean on=false;

boolean ledPIsLit=false;        //ledPIsLit starts at off
boolean ledAIsLit=false;        //led starts at off

int hydraulicTestFreq = 20; //The number of miliseconds between 
                            //tests of hydraulic pressure sensors

//Used within autoExec
long int shakeBegin = 0;


//*******  GETTERS && SETTERS *****
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
//        d (delay): amount of seconds between the first and the 2nd digital read to confirm the value.
// Return: the status AS-IS. As al of our INPUTS: LOW==ACTIVE, and HIGH==DISABLED
uint8_t inputIs(int pin, int d){

    uint8_t value0 = HIGH;
    uint8_t value1 = LOW;

    do{
	value0 = digitalRead(pin);
	delay(d);
	value1 = digitalRead(pin);
    }while (value0!=value1);
 
    return value0;

}

void read4inARow(uint8_t &buttonUp, uint8_t &buttonDown, uint8_t &buttonLeft, uint8_t &buttonRight){

  const int d = 3;

  uint8_t vU0 = digitalRead(PIN_BUTTON_UP);
  uint8_t vD0 = digitalRead(PIN_BUTTON_DOWN);
  uint8_t vL0 = digitalRead(PIN_BUTTON_LEFT);
  uint8_t vR0 = digitalRead(PIN_BUTTON_RIGHT);
  
  delay(d);
  
  uint8_t vU1 = digitalRead(PIN_BUTTON_UP);
  uint8_t vD1 = digitalRead(PIN_BUTTON_DOWN);
  uint8_t vR1 = digitalRead(PIN_BUTTON_RIGHT);
  uint8_t vL1 = digitalRead(PIN_BUTTON_LEFT);

  buttonUp = checkIfEquals(vU0,vU1);
  buttonDown = checkIfEquals(vD0,vD1);
  buttonLeft = checkIfEquals(vL0,vL1);
  buttonRight = checkIfEquals(vR0,vR1);

}


// **** END of GETTERS && SETTERS

// **** DATA HANDLING

// This function is supposed to invert a digital value readed from a pin like HIGH or LOW.
// e: HIGH or LOW
// return: if c=HGH then return LOW, if c=LOW then return HIGH. DEFAULT:   
uint8_t revertDigitalSignalValue(uint8_t val){

  uint8_t oppositeValue=B00001111;
  
  if (val==HIGH){
    oppositeValue=LOW;
  }else if (val==LOW){
    oppositeValue=HIGH;
  }

  return oppositeValue;
  
}

// Checks if the values received are equal.
// If equal returns the value of the first value,
// if not equals returns LOW.
uint8_t checkIfEquals(uint8_t a, uint8_t b){
  uint8_t value=LOW;
  if (a==b){
    value=a;
  }
  return value;
}


// **** END of DATA HANDLING

// **** MACHINE MOVEMENTS

// Moves the cylinder to the top position and returns the time it gets to reach that place.
unsigned long moveCylinderToEnd(int cylinderPin){

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
  while ( (inputIs(PIN_PRESSURE,1)==LOW)  && (timestamp+time < millis()) ){}          //
  digitalWrite(cylinderPin,LOW);
  
  
}

void goToTheInitialPosition(){
  
  setSolenoids(HIGH);
  
  moveCylinderToEnd(PIN_SOLD);
  moveCylinderToEnd(PIN_SOLL);
  
}

// Activates the shaker during some time
// secs: the time we will be shaking in seconds.
void shakeTheSand(int secs){
  
  unsigned long thisTime=millis();
  while( (thisTime+(secs*1000) > millis())){
    digitalWrite(PIN_SOLS,LOW);
  }
  digitalWrite(PIN_SOLS,HIGH);
}



// **** END OF MACHINE MOVEMENTS

// **** MACHINE MODES

void applyManualMode(uint8_t array[]){

  digitalWrite(PIN_BUTTON_UP, revertDigitalSignalValue(array[ID_BUTTON_UP]));
  digitalWrite(PIN_BUTTON_DOWN, revertDigitalSignalValue(array[ID_BUTTON_DOWN]));
  digitalWrite(PIN_BUTTON_LEFT, revertDigitalSignalValue(array[ID_BUTTON_LEFT]));
  digitalWrite(PIN_BUTTON_RIGHT, revertDigitalSignalValue(array[ID_BUTTON_RIGHT]));
  digitalWrite(PIN_BUTTON_SHAKER, revertDigitalSignalValue(array[ID_BUTTON_SHAKER]));
  
}

// **** END OF MACHINE MODES


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

  panelArray[ID_BUTTON_UP] = checkIfEquals(vU0,vU1);
  panelArray[ID_BUTTON_DOWN] = checkIfEquals(vD0,vD1);
  panelArray[ID_BUTTON_LEFT] = (uint8_t) checkIfEquals(vL0,vL1);
  panelArray[ID_BUTTON_RIGHT] = checkIfEquals(vR0,vR1);
  panelArray[ID_BUTTON_SHAKER] = checkIfEquals(vS0,vS1);
  panelArray[ID_PRESSURE] = checkIfEquals(vP0,vP1);
  panelArray[ID_SWON] = checkIfEquals(vSwOn0,vSwOn1);
  panelArray[ID_SWAUTO] = checkIfEquals(vSwAuto0,vSwAuto1);
  panelArray[ID_POTM] = checkIfEquals(vPotM0,vPotM1);
  panelArray[ID_POTD] = checkIfEquals(vPotD0,vPotD1);

}

// ** UP FROM HERE -- ALREADY REVIEWED
// ** DOWN FROM HERE --  REVIWED but.. WAITTING


// ** DOWN FROM HERE -- PENDING.
// ---------------------------


void setAuto(){
    // This function is run every mainloop increment. It makes sure the correct LEDs are on
    // at evey given instant
    delay(100); //Universal delay for debouncing and smooth operation

    //-----------
    if(on){

        //This block makes sure that if we're reading high pressure, PIN_LED_ON is lit
      if(inputIs(PIN_PRESSURE,5)==HIGH && !ledPIsLit){   //If we read high pressure, but PIN_LED_ON is not lit
  	digitalWrite(PIN_LED_ON,HIGH);              //...then turn on PIN_LED_ON
  	ledAIsLit=true;     	              //set PIN_LED_ONIsLit flag so as not to run this block redundently
  	//Record the time we lit up PIN_LED_ON
  	prestime=millis();
  
      }else if(inputIs(PIN_PRESSURE,5)==LOW && ((millis()-prestime)>delaytime)){
  	    //If we're no longer reading high pressure and enough time has passed for the user to see the LED
  	  	    //Update flags and led accordingly
  	    ledAIsLit=false;
  	    digitalWrite(PIN_LED_ON,LOW);
      }
 
      if (inputIs(PIN_SWAUTO,3)==HIGH){
        //Set the automode global flag. Indicates to the rest of the program that automode is active

        //Because we're turning on auto, we need to make sure that we have values for dRetractionTime and mExtention time

        //If we don't have values for either, we need to call drawer bounce
        //This is because we need the shaft to be open so as not to cause pre-mature compression
        //INTERUPTS THE MAIN LOOP
        if(dRetractionTime==-1 || mExtensionTime==-1){
//           drawerBounce();
	}

          //If we don't have a value for mExtensionTime,l run mainbounce to derive one
	  //INTERUPTS THE MAIN LOOP
	if(mExtensionTime == -1){
	    //mainBounce();
	}

	automode=true;

	  //Blink Procedure
	  //If we have waited an entire blink cycle...
	if((millis()-ledAStartTime)>delaytime){

	    //Turn ledA on if it is off, off if it's on
	    if(ledAIsLit){
	      digitalWrite(PIN_LED_AUTO,LOW);
	      ledAIsLit=0;
	    }else{
              digitalWrite(PIN_LED_AUTO,HIGH);
	      ledAIsLit=1;
	      ledAStartTime=millis(); //Either way, reset the clock
	    }

	}
	    //If the autoswitch is not on, make sure that ledA and automode flag are off

      ////This block makes sure that if the auto switch is toggled, ledA is blinking
      }else if(digitalRead(PIN_SWAUTO)==HIGH){
  	delay(3);
  	if(automode && digitalRead(PIN_SWAUTO)==LOW){
  
  	    //Turn off automode and handle variables cleanly
  	    automode=false;
  	    terminateAutoExec();
  
  	    //Turn off the LED too
  	    digitalWrite(PIN_LED_AUTO,HIGH);
  	}
      }
    }
  
//  return;

}


/////////TEST FUNCTIONS/////////////

//TODO: Do we want to be able to change the vale of PIN_POTD during automatic operation?

int dHaltTime(){
//This function calculates the necessary halt time from the value read by our potentiometer PIN_POTD

    //read in the value of the drawer potentiometer
    int PIN_POTDValue = analogRead(PIN_POTD);

    //Calculate the fraction that the potentiometer has been turned as a number beween 0 and 1.0
    float fracTurn = PIN_POTDValue / 1023.0;

    //TODO: There better be a value for dRetractionTime... how to catch if there isnt?
    return (int)(fracTurn * dRetractionTime);
}

void drawerTiming(){
    // This test function sets the drawer cylinder to a position configured with a potentiometer
    // It is used to calibrate the appropriate halt time as follows
  
    if(dRetractionTime==-1){ //If we do not yet have a value for retracionTime, call drawerBounce to derive one
//	drawerBounce();
    }

    delay(300); //Debounce 

    // extend the cylinder until we reach threshold pressure
    digitalWrite(PIN_SOLR,HIGH);                //Begin fowards pressure
    while(inputIs(PIN_PRESSURE,5)==LOW){ //While we are not at threshold pressure... 
	delay(hydraulicTestFreq);                      //continue to push the drawer
    }
    digitalWrite(PIN_SOLR,LOW);                 //Cut forward pressure

    // calculate haltTime the amount of time it would take to retract perc percent of the way down the shaft
    float haltTime = dHaltTime();

    // start timer
    long int timerStart = millis();
    long int timeElapsed = 0; //This will represent the elapsed time since beginning of retraction

    //Debounce
    delay(300);

    // retract until timeElapsed has reached haltTime
    digitalWrite(PIN_SOLL,HIGH);    //Begin retraction pressure
    while(timeElapsed < haltTime && inputIs(PIN_PRESSURE,5)==LOW){ //Continue retracting until we hit our halting time or pressure threshold
	delay(hydraulicTestFreq);
	timeElapsed = millis() - timerStart;
	Serial.print(timeElapsed,DEC);
    }
    digitalWrite(PIN_SOLL,LOW); // The cylinder should now be in position, stop here
    return;
}

///MAIN CYLINDER TESTS

int mHaltTime(){
//This function calculates the necessary halt time from the value read by our potentiometer PIN_POTD
  
    //read in the value of the drawer potentiometer
    int PIN_POTMValue = analogRead(PIN_POTM);

    //Calculate the fraction that the potentiometer has been turned as a number beween 0 and 1.0
    float fracTurn = PIN_POTMValue / 1023.0;

    //TODO: There better be a value for dRetractionTime... how to catch if there isnt?
    return (int)(fracTurn * mExtensionTime);
}

void mainTiming(){
    // This test function sets the main cylinder to a position configured with a potentiometer
    // It is used to calibrate the appropriate halt time as follows
  
    if(mExtensionTime==-1){ //If we do not yet have a value for retracionTime, call drawerBounce to derive one
	//mainBounce();
    }

    delay(300); //Debounce, don't want to read pressureIsHigh twice in one strike!

    // retract the cylinder until we reach threshold pressure
    digitalWrite(PIN_SOLD,HIGH);                //Begin fowards pressure
    while(inputIs(PIN_PRESSURE,5)==LOW){ //While we are not at threshold pressure... 
	delay(hydraulicTestFreq);                      //continue to push the drawer
    }
    digitalWrite(PIN_SOLD,LOW);                 //Cut forward pressure

    // calculate haltTime the amount of time it would take to retract perc percent of the way down the shaft
    float haltTime = dHaltTime();

    // start timer
    long int timerStart = millis();
    long int timeElapsed = 0; //This will represent the elapsed time since beginning of retraction

    //Debounce
    delay(300);

    // retract until timeElapsed has reached haltTime
    digitalWrite(PIN_SOLU,HIGH);    //Begin retraction pressure
    while(timeElapsed < haltTime && inputIs(PIN_PRESSURE,5)==LOW){ //Continue retracting until we hit our halting time
	delay(hydraulicTestFreq);
	timeElapsed = millis() - timerStart;
	Serial.print(timeElapsed,DEC);
    }
    digitalWrite(PIN_SOLU,LOW); // The cylinder should now be in position, stop here
    return;
}

void testButtons(){    //this is the function for controlling the machine manually via buttons
    if(on) return;    //if we ended up here somehow when the on switch is low, go back to where we came from
    if(digitalRead(PIN_BUTTON_UP)==LOW){  //if we read up button on, wait 3ms for debounce
	delay(3);
	if(digitalRead(PIN_BUTTON_UP)==LOW){ //if we read it low still, then button is pressed, run drawer bounce routine
	    //drawerBounce();
	}
    }
    if(digitalRead(PIN_BUTTON_DOWN)==LOW){ 
	delay(3);
	if(digitalRead(PIN_BUTTON_DOWN)==LOW){ 
	    drawerTiming();
	}
    }
    if(digitalRead(PIN_BUTTON_LEFT)==LOW){  //if left button is low, debounce it
	delay(3);
	if(digitalRead(PIN_BUTTON_LEFT)==LOW){ //still low? turn on solenoid
	    //mainBounce();
	}
    }
    if(digitalRead(PIN_BUTTON_RIGHT)==LOW){
	delay(3);
	if(digitalRead(PIN_BUTTON_RIGHT)==LOW){
	    mainTiming();
	}
    }
}

////////AUTO STATE MACHINE////////
//The following is a state machine to automate the brick pressing process
//TODO: Consider revising to enumerated type 
const short PUSH_BRICK = 0;       //Extends the drawer to eject the brick.
const short DROP_PLATFORM = 1;    //Opens up the vertical shaft for dumping
const short DUMP_DIRT = 2;        //Shakes dirt into the shaft 
const short OBSTRUCT_PASSAGE = 3; //Moves the drawer to block shaft and allow a surface for compression
const short COMPRESS_BLOCK = 4;   //Extends the main cylinder to compress a brick
const short OPEN_PASSAGE = 5;     //Retract the drawer to open the passage between the hopper and compression chamber
const short RAISE_BRICK = 6;      //Brings the brick to the drawer level

//The state we track
short autoState = PUSH_BRICK;
int lastStateChange = 0; //Marks the last time state was changed
boolean stateIsSetup = false;

void changeAutoState(int nextState){
    autoState = nextState;
    stateIsSetup = false; //This is used by every state in the execAuto machine 
    //to indicate that the next state must run its setup procedure
    lastStateChange = millis(); //This marks the time at which states were changed
}

void terminateAutoExec(){
    //This function should be run when autoExec is abrubtly ended.
    //Because we have many state variables 
    autoState = PUSH_BRICK;
    lastStateChange = 0;
    setSolenoids(LOW);
}

void autoExec(){
//This function is essentially a state machine, performing a single incremental action
//every loop cycle depending on the state of autoState

    delay(10); //For debouncing and stablization throughout the loop

    const long int desiredShakeTime = 3000;

    //This state machine should not be operating unless the machine is active and on automode
    if(!on || !automode){return;}


    //Clear the drawer and open the chamber
    if(autoState==PUSH_BRICK){
	if(!stateIsSetup){
	    digitalWrite(PIN_SOLR,HIGH);
	    stateIsSetup = true;
	}

	else if(inputIs(PIN_PRESSURE,5)==HIGH){
	    digitalWrite(PIN_SOLR,LOW);
	    changeAutoState(DROP_PLATFORM);
	}

    } 

    if(autoState==DROP_PLATFORM){
	if(!stateIsSetup){
	    digitalWrite(PIN_SOLD,HIGH);
	    stateIsSetup = true;
	}

	else if(inputIs(PIN_PRESSURE,5)==HIGH){
	    digitalWrite(PIN_SOLD,LOW);
	    changeAutoState(DUMP_DIRT);
	}
    }


    else if(autoState==DUMP_DIRT){
	if(!stateIsSetup){
	    //Begin solonoid and trip flag
	    digitalWrite(PIN_SOLS,HIGH);
	    stateIsSetup = true;
	}

	//If we've shaken enough...
	else if(millis() - lastStateChange > desiredShakeTime){

	    //Terminate solonoid and correct states
	    changeAutoState(OBSTRUCT_PASSAGE);
	    digitalWrite(PIN_SOLS,LOW);
	}
    }

    //
    else if(autoState==OBSTRUCT_PASSAGE){

	//Begin retraction if it has not yet begun
	if(!stateIsSetup){
	    digitalWrite(PIN_SOLL,HIGH);
	    stateIsSetup=true;
	}

	//If we've reached threshold time...
	else if(millis() - lastStateChange > dHaltTime()){

	    //Stop retraction and change states after reaching threshold 
	    digitalWrite(PIN_SOLL,LOW);
	    changeAutoState(COMPRESS_BLOCK);
	}

    }


    //Pushes main cylinder to a compression state against the drawer
    else if(autoState==COMPRESS_BLOCK){
	if(!stateIsSetup){
	    digitalWrite(PIN_SOLU,HIGH);
	    stateIsSetup=true;
	}

	else if(millis() - lastStateChange > mHaltTime()){

	    //Turn off the main cylinder
	    digitalWrite(PIN_SOLU,LOW);

	    //And pull back slightly to relieve pressure
	    digitalWrite(PIN_SOLD, HIGH);
	    delay(500); //An arbitrary meter
	    digitalWrite(PIN_SOLD,LOW);
	    changeAutoState(OPEN_PASSAGE);
	}
    }


    else if(autoState==OPEN_PASSAGE){
	if(!stateIsSetup){
	    digitalWrite(PIN_SOLL,HIGH);
	    stateIsSetup = true;
	}

	else if(inputIs(PIN_PRESSURE,5)==HIGH){
	    digitalWrite(PIN_SOLL,LOW);
	    changeAutoState(RAISE_BRICK);
	}
    }


    else if(autoState==RAISE_BRICK){
	if(!stateIsSetup){
	    digitalWrite(PIN_SOLU,HIGH);
	    stateIsSetup = true;
	}

	else if(inputIs(PIN_PRESSURE,5)==HIGH){
	    digitalWrite(PIN_SOLU,LOW);
	    changeAutoState(PUSH_BRICK);
	}
    }

}
            


// the setup routine runs once when you press reset:
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

    // Set initial status
    digitalWrite(PIN_BUTTON_UP, HIGH);
    digitalWrite(PIN_BUTTON_DOWN, HIGH);
    digitalWrite(PIN_BUTTON_LEFT, HIGH);
    digitalWrite(PIN_BUTTON_RIGHT, HIGH);
    digitalWrite(PIN_BUTTON_SHAKER, HIGH);
    digitalWrite(PIN_PRESSURE, HIGH);

    Serial.begin(9600);
}
// the loop routine runs over and over again forever:
void loop() {

  readPanel(panelArray,panelDelay);

  if (panelArray[ID_SWON]==LOW){  // Power ON
    digitalWrite(PIN_LED_ON,LOW);
    
    if (panelArray[ID_SWAUTO]==HIGH){ // Manual mode
      digitalWrite(PIN_LED_AUTO,LOW);
      applyManualMode(panelArray);

    }else{                            // Auto mode
        // Set the proper initial values
      digitalWrite(PIN_LED_AUTO,HIGH);

      // Checks, if needed.
      int stage=0;
      unsigned long verticalAxisTime = 0;
      unsigned long horizontalAxisTime = 0;
      int timeShaking=3;     //   <----
      
      switch(stage){
      
        case 0:    // Initial case: Measure the time it takes to do a complete travel for the cylinders.
            goToTheInitialPosition();
            verticalAxisTime = moveCylinderToEnd(PIN_SOLU);
            horizontalAxisTime = moveCylinderToEnd(PIN_SOLD);
            stage++;
          break;

        case 1: 
            goToTheInitialPosition();
            stage++;
          break;

        case 2:    // Shakes the sand.
            shakeTheSand(timeShaking);
            stage++;
          break;

        case 3:
            moveCylinderDuring(PIN_SOLU,verticalAxisTime);
            moveCylinderDuring(PIN_SOLR,horizontalAxisTime);
            stage++;
          break;
          
        case 4:
            
          break;
        case 5:
          break;
        case 6:
          break;
        default:
          break;
          
      
      }
      // Go to initial position
      goToTheInitialPosition();


    }  
  
  
  }else{                              // Power OFF
    setSolenoids(HIGH);
    digitalWrite(PIN_LED_ON,HIGH);
    digitalWrite(PIN_LED_AUTO,HIGH);
    

  }
  
  

}

