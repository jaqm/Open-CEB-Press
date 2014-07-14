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

int potA=PIN_F2;	//
int potB=PIN_F1;	//

int pressuresens=PIN_F0;


int ledP=PIN_E0;

int xledA=PIN_E1;//either xledA or xledM may be on at the same time

unsigned long delaytime=500; //500ms - half second blink of pressure sensor indicator


unsigned long lasttime=0;
unsigned long prestime=0;

boolean automode=false;		//automode starts at off
boolean on=false;		//on starts at off
boolean pressure=false;		//pressure starts at off


boolean ledblink=false;		//led starts at off

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
 if(!on) return;			//is the ON button set to off? if yes, get outta here
  if(digitalRead(pressuresens)==LOW && !pressure){	//
    delay(3);
    if(digitalRead(pressuresens)==LOW){
      digitalWrite(ledP,HIGH);
      pressure=1;
      prestime=millis();
    }
  }else{
      delay(3);

    if(digitalRead(pressuresens)==HIGH && ((millis()-prestime)>delaytime)){
      pressure=0;
      digitalWrite(ledP,LOW);
    }

  }
 if(digitalRead(switchAUTO)==HIGH){
  delay(3);
  if(digitalRead(switchAUTO)==HIGH){
   automode=true;
   if((millis()-lasttime)>delaytime){
    if(ledblink){
    digitalWrite(xledA,LOW);
    ledblink=0;
    }else{
    digitalWrite(xledA,HIGH);
    ledblink=1;
    }
    lasttime=millis();
    }

  }
 }else{
   delay(3);
  if(digitalRead(switchAUTO)==LOW){
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

/////////TEST FUNCTIONS/////////////
void drawerBounce(){                
  // This function sends the drawer to its forward most point, then returns to its back limit. 
  // We will change direction and halt whenever we reach a threshold of pressure indicated by our
  // 'pressuresens' register going LOW. 
  switchSol(3,HIGH);                //Begin fowards pressure
  while(digitalRead(pressuresens)){ //When our pressuresens pin is high, it means that we have not reached 
                                    //a threshhold of pressure
    delay(50);                      //so we continue to push the drawer
  }
  switchSol(3,LOW);                 //Cut forward pressure
  delay(5);
  switchSol(2,HIGH);                //Begin backwards pressure
  while(digitalRead(pressuresens)){
    delay(50);
  }
  switchSol(2,LOW);
  return;
}

void testButtons(){    //this is the function for controlling the machine manually via buttons
 if(on) return;    //if we ended up here somehow when the on switch is low, go back to where we came from
 if(digitalRead(btnU)==LOW){  //if we read up button on, wait 3ms for debounce
  delay(3);
  if(digitalRead(btnU)==LOW){ //if we read it low still, then button is pressed, run drawer bounce routine
    drawerBounce();
  }
 }else{
    delay(3);
  if(digitalRead(btnU)==HIGH){  //if after debounce it is high, go to switch case 0 and set it low
    switchSol(0,LOW);   
  }
 }
 if(digitalRead(btnD)==LOW){  //if down button is presssed, debounce dat ish
  delay(3);
  if(digitalRead(btnD)==LOW){ //still low? ok, drive dat puppy
    switchSol(1,HIGH);
  }
 }else{       //otherwise turn off the solenoid and check the next button
    delay(3);
  if(digitalRead(btnD)==HIGH){
    switchSol(1,LOW);
  }
 }
 if(digitalRead(btnL)==LOW){  //if left button is low, debounce it
  delay(3);
  if(digitalRead(btnL)==LOW){ //still low? turn on solenoid
    switchSol(2,HIGH);
  }
 }else{       //otherwise turn off solenoid and move on to right button
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
 if(digitalRead(btnS)==LOW){  //is the shaker motor button pressed?
  delay(3);
  if(digitalRead(btnS)==LOW){ //
    switchSol(4,HIGH);
  }
 }else{
    delay(3);
  if(digitalRead(btnS)==HIGH){
    switchSol(4,LOW);
  }
 }

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
      testButtons();
    }
  }
}
