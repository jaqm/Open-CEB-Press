//pin mapping:

int solU=PIN_B6;	//solenoid for cylinder up
int solD=PIN_B5;	//solenoid for cylinder down
int solL=PIN_B4;	//solenoid for drawer left
int solR=PIN_B3;	//solenoid for drawer right
int solS=PIN_B2;	//solenoid for shaker motor 

int switchON=PIN_C7;	//on/off switch
int switchAUTO=PIN_C6;	//auto/manual switch
int btnU=PIN_C5;	//button for up
int btnD=PIN_C4;	//button for down
int btnL=PIN_C3;	//button for left
int btnR=PIN_C2;	//button for right
int btnS=PIN_C1;	//button for shaker

int potD=PIN_F2;	//potentiometer for the drawer
int potM=PIN_F1;	//potentiometer for the main cylinder

int pressuresens=PIN_F0;


int ledP=PIN_E0;

int xledA=PIN_E1;//either xledA or xledM may be on at the same time

unsigned long delaytime=500; //500ms - half second blink of pressure sensor indicator


unsigned long ledAStartTime=0;
unsigned long prestime=0;

boolean automode=false;		//automode starts at off
boolean on=false;		//on starts at off

boolean ledPIsLit=false;		//ledPIsLit starts at off
boolean ledAIsLit=false;		//led starts at off

int hydraulicTestFreq = 20; //The number of miliseconds between 
                            //tests of hydraulic pressure sensors

void switchSol(int number, int state){		
 int sol=-1;					
 switch(number){				
  case 0: //UP					//case 0:  sol = solup
    sol=solU;					
    break;
  case 1: //DOWN				
    sol=solD;					
    break;
  case 2: //LEFT				
    sol=solL;					
    break;
  case 3: //RIGHT				
    sol=solR;
    break;
  case 4: //SHAKER
    sol=solS;
    break;
  default:
    break;
 }
 if(on && sol!=-1){				//if false(on was initialized as false earlier) AND (not)sol = -1 , 
  digitalWrite(sol,state);			//set sol equal to the state from the above void
 } else {
  digitalWrite(sol,LOW);			//otherwise, don't do anything with the solenoid
 }

}


//reduced on time to 50 instead of 100

void setOn(){
 if(!on){				//if the switch is off,
  switchSol(0,LOW);			//turn all solenoids off
  switchSol(1,LOW);
  switchSol(2,LOW);
  switchSol(3,LOW);
  switchSol(4,LOW);
 }
 if(digitalRead(switchON)==LOW){	//if we are reading the ON switch to indeed be on
  delay(5);				// then delay for debounce
  if(digitalRead(switchON)==LOW){	//if it's still low, then we set ON to be true 
   on=true;
  }
 }else{					//otherwise, if the on switch is set to off, 
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
 if(!on) return;			

 //This block makes sure that if we're reading high pressure, ledP is lit
  if(digitalRead(pressuresens)==LOW && !ledPIsLit){	//If we read high pressure, but ledP is not lit
    delay(3); //debounce
    if(digitalRead(pressuresens)==LOW){  

      //...then turn on ledP
      digitalWrite(ledP,HIGH);           

      //set ledPIsLit flag so as not to run this block redundently
      ledPIsLit=true;   

      //Record the time we lit up ledP
      prestime=millis(); 
    }
  }else{
      delay(3); //debounce

    //If we're no longer reading high pressure and enough time has passed for the user to see the LED
    if(digitalRead(pressuresens)==HIGH && ((millis()-prestime)>delaytime)){  

      //Then change flag accordingly
      ledPIsLit=false;

      //And turn it off
      digitalWrite(ledP,LOW);
    }

  }

 //This block makes sure that if the auto switch is toggled, ledA is blinking
 if(digitalRead(switchAUTO)==HIGH){
  delay(3); //debounce
  if(digitalRead(switchAUTO)==HIGH){

   //Set the automode global flag. Indicates to the rest of the program that automode is active
   automode=true;

   //Blink Procedure
   //If we have waited an entire blink cycle...
   if((millis()-ledAStartTime)>delaytime){

    //Turn ledA on if it is off, off if it's on
    if(ledAIsLit){
    digitalWrite(xledA,LOW);
    ledAIsLit=0;
    }else{
    digitalWrite(xledA,HIGH0u    ledAIsLit=1;
    }
    ledAStartTime=millis(); //Either way, reset the clock
    }

  }
 //If the autoswitch is not on, make sure that ledA and automode flag are off
 }else{
   delay(3);
  if(automode && digitalRead(switchAUTO)==LOW){
   automode=false;
   digitalWrite(xledA,HIGH);
  }
 }
}

void actButtons(){		//this is the function for controlling the machine manually via buttons
 if(!on) return;		//if we ended up here somehow when the on switch is low, go back to where we came from
 if(digitalRead(btnU)==LOW){	//if we read up button on, wait 3ms for debounce
  delay(3);
  if(digitalRead(btnU)==LOW){	//if we read it low still, go to switchsol case 0 and set it high
    switchSol(0,HIGH);
  }
 }else{
    delay(3);
  if(digitalRead(btnU)==HIGH){	//if after debounce it is high, go to switch case 0 and set it low
    switchSol(0,LOW);		
  }
 }
 if(digitalRead(btnD)==LOW){	//if down button is presssed, debounce dat ish
  delay(3);
  if(digitalRead(btnD)==LOW){	//still low? ok, drive dat puppy
    switchSol(1,HIGH);
  }
 }else{				//otherwise turn off the solenoid and check the next button
    delay(3);
  if(digitalRead(btnD)==HIGH){
    switchSol(1,LOW);
  }
 }
 if(digitalRead(btnL)==LOW){	//if left button is low, debounce it
  delay(3);
  if(digitalRead(btnL)==LOW){	//still low? turn on solenoid
    switchSol(2,HIGH);
  }
 }else{				//otherwise turn off solenoid and move on to right button
    delay(3);
  if(digitalRead(btnL)==HIGH){
    switchSol(2,LOW);
  }
 }
  if(digitalRead(btnR)==LOW){
  delay(3);
  if(digitalRead(btnR)==LOW){
    switchSol(3,HIGH);
  }
 }else{
    delay(3);
  if(digitalRead(btnR)==HIGH){
    switchSol(3,LOW);
  }
 }
 if(digitalRead(btnS)==LOW){	//is the shaker motor button pressed?
  delay(3);
  if(digitalRead(btnS)==LOW){	//
    switchSol(4,HIGH);
  }
 }else{
    delay(3);
  if(digitalRead(btnS)==HIGH){
    switchSol(4,LOW);
  }
 }
}

/////////TEST FUNCTIONS/////////////
//TODO: Don't do redundant work. Record our retraction time from drawerbounce and call drawerbounce
//from drawerTiming if the value does not already exist.

long int dRetractionTime = -1; //A vanlue of -1 means that we have not yet recorded a value for dRetractionTime

void drawerBounce(){                
  // This function sends the drawer to its forward-most point, then returns to its back limit. 
  // We will change direction and halt whenever we reach a threshold of pressure indicated by our
  // 'pressuresens' register going LOW. 
  switchSol(3,HIGH);                //Begin extension pressure
  while(digitalRead(pressuresens)){ //While we have not reached threshold pressure we have not reached 
    delay(50);                      //continue to push the drawer
  }
  switchSol(3,LOW);                 //Cut extention pressure pressure
  delay(5);
  switchSol(2,HIGH);                //Begin backwards pressure
  long int retractionStart = millis();   //Record the starttime of our retraction
  while(digitalRead(pressuresens)){
    delay(50);
  }
  long int dRetractionTime = millis() - retractionStart;
  switchSol(2,LOW);
  return;
}

// With regards to our drawerTiming test method:
// We want our cylinders to stop at increments down the shaft in order to coordinate movement between them
// As our machines only means of sense is testing pressure threshold, we need our press to autonomously generate an
// interval of time with which to push and pull the cylinders in order to reach the correct distance interval.
// As it appears to us, there are two ways to calibrate this 'halt time'

//TODO: Do we want to be able to change the vale of potD during automatic operation?

void drawerTiming(){
  // This test function sets the drawer cylinder to a position configured with a potentiometer
  // It is used to calibrate the appropriate halt time as follows
  
  // read in the value of the drawer potentiometer
  int potDValue = analogRead(potD);

  //Gives us the fraction that the potentiometer has been turned as a float between 0 and 1.0
  float fracTurn = potDValue / 1023.0;
  
  if(dRetractionTime==-1){ //If we do not yet have a value for retracionTime, call drawerBounce to derive one
    drawerBounce();
  }

  // extend the cylinder until we reach threshold pressure
  switchSol(3,HIGH);                //Begin fowards pressure
  while(digitalRead(pressuresens)){ //While we are not at threshold pressure... 
    delay(hydraulicTestFreq);                      //continue to push the drawer
  }
  switchSol(3,LOW);                 //Cut forward pressure

  // calculate haltTime the amount of time it would take to retract perc percent of the way down the shaft
  float haltTime = fracTurn * dRetractionTime;

  // start timer
  long int timerStart = millis();
  long int timeElapsed; //This will represent the elapsed time since beginning of retraction

  // retract until timeElapsed has reached haltTime
  switchSol(2,HIGH);    //Begin retraction pressure
  while(timeElapsed < haltTime){ //Continue retracting until we hit our halting time
    delay(hydraulicTestFreq);
    timeElapsed = millis() - timerStart;
  }
  switchSol(2,LOW); // The cylinder should now be in position, stop here
  return;
}

void mainBounce(){                
  // This function sends the drawer to its most retracted point, then returns to its upper limit. 
  // We will change direction and halt whenever we reach a threshold of pressure indicated by our
  // 'pressuresens' register going LOW. 
  switchSol(1,HIGH);                //Begin retraction pressure
  while(digitalRead(pressuresens)){ //While we have not reached threshold pressure we have not reached 
    delay(hydraulicTestFreq);                      //continue to push the drawer
  }
  switchSol(1,LOW);                 //Cut retraction pressure
  delay(5);
  switchSol(0,HIGH);                //Begin backwards pressure
  long int extensionStart = millis();   //Record the starttime of our extension
  while(digitalRead(pressuresens)){
    delay(hydraulicTestFreq);
  }
  long int mExtensionTime = millis() - extensionStart;
  switchSol(0,LOW);
  return;
}

void mainTiming(){
  // This test function sets the main cylinder to a position configured with a potentiometer
  // It is used to calibrate the appropriate halt time as follows
  
  // read in the value of the main potentiometer
  int potMValue = analogRead(potM);

  //Gives us the fraction that the potentiometer has been turned as a float between 0 and 1.0
  float fracTurn = potMValue / 1023.0;
  
  if(mExtensionTime==-1){ //If we do not yet have a value for retracionTime, call drawerBounce to derive one
    mainBounce();
  }

  // retract the cylinder until we reach threshold pressure
  switchSol(1,HIGH);                //Begin fowards pressure
  while(digitalRead(pressuresens)){ //While we are not at threshold pressure... 
    delay(hydraulicTestFreq);                      //continue to push the drawer
  }
  switchSol(1,LOW);                 //Cut forward pressure

  // calculate haltTime the amount of time it would take to retract perc percent of the way down the shaft
  float haltTime = fracTurn * mExtensionTime;

  // start timer
  long int timerStart = millis();
  long int timeElapsed; //This will represent the elapsed time since beginning of retraction

  // retract until timeElapsed has reached haltTime
  switchSol(0,HIGH);    //Begin retraction pressure
  while(timeElapsed < haltTime){ //Continue retracting until we hit our halting time
    delay(hydraulicTestFreq);
    timeElapsed = millis() - timerStart;
  }
  switchSol(0,LOW); // The cylinder should now be in position, stop here
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
//The following is a state machine to 
short int autoState = 0;
//Change to 'short int'? Does arduino accept const?
short DUMP_DIRT = 1;
short OBSTRUCT_PASSAGE = 2;
short COMPRESS_BLOCK = 3;
short OPEN_PASSAGE = 4;
short RAISE_BRICK = 5;
short PUSH_BRICK = 6; 

void autoExec(){
  if(!on || !automode)
  if(autoState==DUMP_DIRT)

}


// the setup routine runs once when you press reset:
void setup() {
  // initialize the digital pin as an output.
  pinMode(ledP, OUTPUT);
  pinMode(xledA, OUTPUT);
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
  digitalWrite(btnU, HIGH);
  digitalWrite(btnD, HIGH);
  digitalWrite(btnL, HIGH);
  digitalWrite(btnR, HIGH);
  digitalWrite(btnS, HIGH);
  digitalWrite(switchON, HIGH);
  digitalWrite(switchAUTO, HIGH);
  digitalWrite(pressuresens, HIGH);

  pinMode(potA, INPUT);
  pinMode(potB, INPUT);

}



// the loop routine runs over and over again forever:
void loop() {
  setOn();
  setAuto();
  if(automode){
   //put in auto press state machine here
  }else{
    if(on){
      actButtons();
    }else{
      drawerTiming();
    }
  }
}
