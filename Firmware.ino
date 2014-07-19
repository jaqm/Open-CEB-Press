#include "Arduino.h"

//pin mapping:

int solU=PIN_B6;    //solenoid for cylinder up
int solD=PIN_B5;    //solenoid for cylinder down
int solL=PIN_B4;    //solenoid for drawer left
int solR=PIN_B3;    //solenoid for drawer right
int solS=PIN_B2;    //solenoid for shaker motor 

int switchON=PIN_C7;    //on/off switch
int switchAUTO=PIN_C6;    //auto/manual switch
int btnU=PIN_C5;    //button for up
int btnD=PIN_C4;    //button for down
int btnL=PIN_C3;    //button for left
int btnR=PIN_C2;    //button for right
int btnS=PIN_C1;    //button for shaker

int potD=PIN_F2;    //potentiometer for the drawer
int potM=PIN_F1;    //potentiometer for the main cylinder

int pressuresens=PIN_F0;


int ledP=PIN_E0;

int xledA=PIN_E1;//either xledA or xledM may be on at the same time

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

//*******  
// Description: Sets all solenoids into mode.
// mode: HIGH or LOW values expected.
void setSolenoids(uint8_t mode){
  digitalWrite(solU,mode);            //turn all solenoids into mode
  digitalWrite(solD,mode);
  digitalWrite(solL,mode);
  digitalWrite(solR,mode);
  digitalWrite(solS,mode);

}



//Encapsulates a debounced read of the pressure sensor
boolean pressureIsHigh(){
  if(digitalRead(pressuresens)==LOW){
    delay(5);
    if(digitalRead(pressuresens)==LOW){
      return true;
    }
  }
  return false;
}


void setOn(){
 if(!on){                //if the switch is off,
//  digitalWrite(solU,LOW);            //turn all solenoids off
//  digitalWrite(solD,LOW);
//  digitalWrite(solL,LOW);
//  digitalWrite(solR,LOW);
//  digitalWrite(solS,LOW);
  setSolenoids(LOW);
 }
 if(digitalRead(switchON)==LOW){    //if we are reading the ON switch to indeed be on
  delay(5);                // then delay for debounce
  if(digitalRead(switchON)==LOW){    //if it's still low, then we set ON to be true 
   on=true;
  }
 }else{                    //otherwise, if the on switch is set to off, 
   delay(5);
  if(digitalRead(switchON)==HIGH){
   on=false;
   digitalWrite(xledA,LOW);

  }
 }

}

void setAuto(){
  // This function is run every mainloop increment. It makes sure the correct LEDs are on
  // at evey given instant
  delay(100); //Universal delay for debouncing and smooth operation

 if(!on) return;            

 //This block makes sure that if we're reading high pressure, ledP is lit
  if(pressureIsHigh() && !ledPIsLit){    //If we read high pressure, but ledP is not lit

      //...then turn on ledP
      digitalWrite(ledP,HIGH);           

      //set ledPIsLit flag so as not to run this block redundently
      ledPIsLit=true;   

      //Record the time we lit up ledP
      prestime=millis(); 
    }
  else{
    //If we're no longer reading high pressure and enough time has passed for the user to see the LED
    if(!pressureIsHigh() && ((millis()-prestime)>delaytime)){  

      //Then change flag accordingly
      ledPIsLit=false;

      //And turn it off
      digitalWrite(ledP,LOW);
    }

  }

 ////This block makes sure that if the auto switch is toggled, ledA is blinking
 if(digitalRead(switchAUTO)==HIGH){
  delay(3); //debounce
  if(digitalRead(switchAUTO)==HIGH){

   //Set the automode global flag. Indicates to the rest of the program that automode is active

   //Because we're turning on auto, we need to make sure that we have values for dRetractionTime and mExtention time

   //If we don't have values for either, we need to call drawer bounce
   //This is because we need the shaft to be open so as not to cause pre-mature compression
   //INTERUPTS THE MAIN LOOP
   if(dRetractionTime==-1 || mExtensionTime==-1){
     drawerBounce();
   }

   //If we don't have a value for mExtensionTime,l run mainbounce to derive one
   //INTERUPTS THE MAIN LOOP
   if(mExtensionTime == -1){
     mainBounce();
   }

   automode=true;

   //Blink Procedure
   //If we have waited an entire blink cycle...
   if((millis()-ledAStartTime)>delaytime){

    //Turn ledA on if it is off, off if it's on
    if(ledAIsLit){
    digitalWrite(xledA,LOW);
    ledAIsLit=0;
    }else{
    digitalWrite(xledA,HIGH);
    ledAIsLit=1;
    ledAStartTime=millis(); //Either way, reset the clock
    }

  }

 //If the autoswitch is not on, make sure that ledA and automode flag are off
 }
 }else if(digitalRead(switchAUTO)==HIGH){
   delay(3);
  if(automode && digitalRead(switchAUTO)==LOW){

   //Turn off automode and handle variables cleanly
   automode=false;
   terminateAutoExec();

   //Turn off the LED too
   digitalWrite(xledA,HIGH);
  }
 }
}

void actButtons(){        //this is the function for controlling the machine manually via buttons
 if(!on) return;        //if we ended up here somehow when the on switch is low, go back to where we came from
 if(digitalRead(btnU)==LOW){    //if we read up button on, wait 3ms for debounce
  delay(3);
  if(digitalRead(btnU)==LOW){    //if we read it low still, go to switchsol case 0 and set it high
    digitalWrite(solU,HIGH);
  }
 }else{
    delay(3);
  if(digitalRead(btnU)==HIGH){    //if after debounce it is high, go to switch case 0 and set it low
    digitalWrite(solU,LOW);        
  }
 }
 if(digitalRead(btnD)==LOW){    //if down button is presssed, debounce dat ish
  delay(3);
  if(digitalRead(btnD)==LOW){    //still low? ok, drive dat puppy
    digitalWrite(solD,HIGH);
  }
 }else{                //otherwise turn off the solenoid and check the next button
    delay(3);
  if(digitalRead(btnD)==HIGH){
    digitalWrite(solD,LOW);
  }
 }
 if(digitalRead(btnL)==LOW){    //if left button is low, debounce it
  delay(3);
  if(digitalRead(btnL)==LOW){    //still low? turn on solenoid
    digitalWrite(solL,HIGH);
  }
 }else{                //otherwise turn off solenoid and move on to right button
    delay(3);
  if(digitalRead(btnL)==HIGH){
    digitalWrite(solL,LOW);
  }
 }
  if(digitalRead(btnR)==LOW){
  delay(3);
  if(digitalRead(btnR)==LOW){
    digitalWrite(solR,HIGH);
  }
 }else{
    delay(3);
  if(digitalRead(btnR)==HIGH){
    digitalWrite(solR,LOW);
  }
 }
 if(digitalRead(btnS)==LOW){    //is the shaker motor button pressed?
  delay(3);
  if(digitalRead(btnS)==LOW){    //
    digitalWrite(solS,HIGH);
  }
 }else{
    delay(3);
  if(digitalRead(btnS)==HIGH){
    digitalWrite(solS,LOW);
  }
 }
}

/////////TEST FUNCTIONS/////////////

//TODO: Don't do redundant work. Record our retraction time from drawerbounce and call drawerbounce
//from drawerTiming if the value does not already exist.

///DRAWER CYLINDER TESTS


void drawerBounce(){                
  // This function sends the drawer to its forward-most point, then returns to its back limit. 
  // We will change direction and halt whenever we reach a threshold of pressure indicated by our
  // 'pressuresens' register going LOW. 
  digitalWrite(solR,HIGH);                //Begin extension pressure
  while(!pressureIsHigh()){ //While we have not reached threshold pressure we have not reached 
    delay(50);                      //continue to push the drawer
  }
  digitalWrite(solR,LOW);                 //Cut extention pressure pressure
  delay(200);
  digitalWrite(solL,HIGH);                //Begin backwards pressure
  long int retractionStart = millis();   //Record the starttime of our retraction
  while(!pressureIsHigh()){
    delay(50);
  }
  dRetractionTime = millis() - retractionStart;
  digitalWrite(solL,LOW);
  return;
}

//TODO: Do we want to be able to change the vale of potD during automatic operation?

int dHaltTime(){
//This function calculates the necessary halt time from the value read by our potentiometer potD

  //read in the value of the drawer potentiometer
  int potDValue = analogRead(potD);

  //Calculate the fraction that the potentiometer has been turned as a number beween 0 and 1.0
  float fracTurn = potDValue / 1023.0;

   //TODO: There better be a value for dRetractionTime... how to catch if there isnt?
  return (int)(fracTurn * dRetractionTime);
}

void drawerTiming(){
  // This test function sets the drawer cylinder to a position configured with a potentiometer
  // It is used to calibrate the appropriate halt time as follows
  
  if(dRetractionTime==-1){ //If we do not yet have a value for retracionTime, call drawerBounce to derive one
    drawerBounce();
  }

  delay(300); //Debounce 

  // extend the cylinder until we reach threshold pressure
  digitalWrite(solR,HIGH);                //Begin fowards pressure
  while(!pressureIsHigh()){ //While we are not at threshold pressure... 
    delay(hydraulicTestFreq);                      //continue to push the drawer
  }
  digitalWrite(solR,LOW);                 //Cut forward pressure

  // calculate haltTime the amount of time it would take to retract perc percent of the way down the shaft
  float haltTime = dHaltTime();

  // start timer
  long int timerStart = millis();
  long int timeElapsed = 0; //This will represent the elapsed time since beginning of retraction

  //Debounce
  delay(300);

  // retract until timeElapsed has reached haltTime
  digitalWrite(solL,HIGH);    //Begin retraction pressure
  while(timeElapsed < haltTime && !pressureIsHigh()){ //Continue retracting until we hit our halting time or pressure threshold
    delay(hydraulicTestFreq);
    timeElapsed = millis() - timerStart;
    Serial.print(timeElapsed,DEC);
  }
  digitalWrite(solL,LOW); // The cylinder should now be in position, stop here
  return;
}

///MAIN CYLINDER TESTS

void mainBounce(){                
  // This function sends the drawer to its most retracted point, then returns to its upper limit. 
  // We will change direction and halt whenever we reach a threshold of pressure indicated by our
  // 'pressuresens' register going LOW.

  digitalWrite(solD,HIGH);                //Begin retraction pressure
  while(!pressureIsHigh()){ //While we have not reached threshold pressure we have not reached 
    delay(hydraulicTestFreq);                      //continue to push the drawer
  }
  digitalWrite(solD,LOW);                 //Cut retraction pressure
  delay(500);
  digitalWrite(solU,HIGH);                //Begin backwards pressure
  long int extensionStart = millis();   //Record the starttime of our extension
  while(!pressureIsHigh()){
    delay(hydraulicTestFreq);
  }
  mExtensionTime = millis() - extensionStart;
  digitalWrite(solU,LOW);
  return;
}

int mHaltTime(){
//This function calculates the necessary halt time from the value read by our potentiometer potD
  
  //read in the value of the drawer potentiometer
  int potMValue = analogRead(potM);

  //Calculate the fraction that the potentiometer has been turned as a number beween 0 and 1.0
  float fracTurn = potMValue / 1023.0;

   //TODO: There better be a value for dRetractionTime... how to catch if there isnt?
  return (int)(fracTurn * mExtensionTime);
}

void mainTiming(){
  // This test function sets the main cylinder to a position configured with a potentiometer
  // It is used to calibrate the appropriate halt time as follows
  
  if(mExtensionTime==-1){ //If we do not yet have a value for retracionTime, call drawerBounce to derive one
    mainBounce();
  }

  delay(300); //Debounce, don't want to read pressureIsHigh twice in one strike!

  // retract the cylinder until we reach threshold pressure
  digitalWrite(solD,HIGH);                //Begin fowards pressure
  while(!pressureIsHigh()){ //While we are not at threshold pressure... 
    delay(hydraulicTestFreq);                      //continue to push the drawer
  }
  digitalWrite(solD,LOW);                 //Cut forward pressure

  // calculate haltTime the amount of time it would take to retract perc percent of the way down the shaft
  float haltTime = dHaltTime();

  // start timer
  long int timerStart = millis();
  long int timeElapsed = 0; //This will represent the elapsed time since beginning of retraction

  //Debounce
  delay(300);

  // retract until timeElapsed has reached haltTime
  digitalWrite(solU,HIGH);    //Begin retraction pressure
  while(timeElapsed < haltTime && !pressureIsHigh()){ //Continue retracting until we hit our halting time
    delay(hydraulicTestFreq);
    timeElapsed = millis() - timerStart;
    Serial.print(timeElapsed,DEC);
  }
  digitalWrite(solU,LOW); // The cylinder should now be in position, stop here
  return;
}

void testButtons(){    //this is the function for controlling the machine manually via buttons
 if(on) return;    //if we ended up here somehow when the on switch is low, go back to where we came from
 if(digitalRead(btnU)==LOW){  //if we read up button on, wait 3ms for debounce
  delay(3);
  if(digitalRead(btnU)==LOW){ //if we read it low still, then button is pressed, run drawer bounce routine
    drawerBounce();
  }
 }
 if(digitalRead(btnD)==LOW){ 
  delay(3);
  if(digitalRead(btnD)==LOW){ 
    drawerTiming();
  }
 }
 if(digitalRead(btnL)==LOW){  //if left button is low, debounce it
  delay(3);
  if(digitalRead(btnL)==LOW){ //still low? turn on solenoid
    mainBounce();
  }
 }
  if(digitalRead(btnR)==LOW){
    delay(3);
    if(digitalRead(btnR)==LOW){
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
      digitalWrite(solR,HIGH);
      stateIsSetup = true;
    }

    else if(pressureIsHigh()){
      digitalWrite(solR,LOW);
      changeAutoState(DROP_PLATFORM);
    }

  } 

  if(autoState==DROP_PLATFORM){
    if(!stateIsSetup){
      digitalWrite(solD,HIGH);
      stateIsSetup = true;
    }

    else if(pressureIsHigh()){
      digitalWrite(solD,LOW);
      changeAutoState(DUMP_DIRT);
    }
  }


  else if(autoState==DUMP_DIRT){
    if(!stateIsSetup){
      //Begin solonoid and trip flag
      digitalWrite(solS,HIGH);
      stateIsSetup = true;
    }

    //If we've shaken enough...
    else if(millis() - lastStateChange > desiredShakeTime){

      //Terminate solonoid and correct states
      changeAutoState(OBSTRUCT_PASSAGE);
      digitalWrite(solS,LOW);
    }
  }

  //
  else if(autoState==OBSTRUCT_PASSAGE){

    //Begin retraction if it has not yet begun
    if(!stateIsSetup){
      digitalWrite(solL,HIGH);
      stateIsSetup=true;
    }

    //If we've reached threshold time...
    else if(millis() - lastStateChange > dHaltTime()){

      //Stop retraction and change states after reaching threshold 
      digitalWrite(solL,LOW);
      changeAutoState(COMPRESS_BLOCK);
    }

  }


  //Pushes main cylinder to a compression state against the drawer
  else if(autoState==COMPRESS_BLOCK){
    if(!stateIsSetup){
      digitalWrite(solU,HIGH);
      stateIsSetup=true;
    }

    else if(millis() - lastStateChange > mHaltTime()){

      //Turn off the main cylinder
      digitalWrite(solU,LOW);

      //And pull back slightly to relieve pressure
      digitalWrite(solD, HIGH);
      delay(500); //An arbitrary meter
      digitalWrite(solD,LOW);
      changeAutoState(OPEN_PASSAGE);
    }
  }


  else if(autoState==OPEN_PASSAGE){
    if(!stateIsSetup){
      digitalWrite(solL,HIGH);
      stateIsSetup = true;
    }

    else if(pressureIsHigh()){
      digitalWrite(solL,LOW);
      changeAutoState(RAISE_BRICK);
    }
  }


  else if(autoState==RAISE_BRICK){
    if(!stateIsSetup){
      digitalWrite(solU,HIGH);
      stateIsSetup = true;
    }

    else if(pressureIsHigh()){
      digitalWrite(solU,LOW);
      changeAutoState(PUSH_BRICK);
    }
  }

}


// the setup routine runs once when you press reset:
void setup() {
  // Define Inputs/Outputs
  pinMode(solU, OUTPUT);
  pinMode(solD, OUTPUT);
  pinMode(solL, OUTPUT);
  pinMode(solR, OUTPUT);
  pinMode(solS, OUTPUT);

  pinMode(ledP, OUTPUT);
  pinMode(xledA, OUTPUT);

  pinMode(btnU, INPUT);
  pinMode(btnD, INPUT);
  pinMode(btnL, INPUT);
  pinMode(btnR, INPUT);
  pinMode(btnS, INPUT);
  pinMode(pressuresens, INPUT);
  pinMode(switchON, INPUT);
  pinMode(switchAUTO, INPUT);
  pinMode(potM, INPUT);
  pinMode(potD, INPUT);

  // Set initial status
  digitalWrite(btnU, HIGH);
  digitalWrite(btnD, HIGH);
  digitalWrite(btnL, HIGH);
  digitalWrite(btnR, HIGH);
  digitalWrite(btnS, HIGH);
  digitalWrite(pressuresens, HIGH);

  if (digitalRead(switchON)==LOW){
    on = true; 
  }

  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  setOn();
  setAuto();
  if(automode){
   autoExec();
  }else{
    if(on){
      actButtons();
    }else{
      testButtons();
    }
  }
}
