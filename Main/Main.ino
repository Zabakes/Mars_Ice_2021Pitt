/*
* Pitt SOAR
* This is the comprehensive code for controlling the drill via MATLAB and Arduino
* Updated 08/26/2021
*/

/*THINGS THAT STILL NEED DONE:
*/

#include "CurrentMon.h"
#include<HX711.h> //Load cell library

//SERIAL COMMUNICATION VARIABLES
int sref=-1;

#define loadCellData 21
#define loadCellClk 53

struct dWrite{
    int port;
    int pin;
    int val;
};

Timer<> T;
HX711 forceSensor;
CurrentMon ISense(0, -22.6, 1, 250, 1, T);

#define UPPER_LIMIT 8.9 //If the current goes above this stop everything

#define MOTOR_STEPS 800
#define RPM 120 

//calculation for length of delay used to control vertical speed (microseconds)
unsigned long stepDelay = 50; 
int distance = 0; //step count

// All the wires needed for full functionality; motor 1 (vertical) and motor2 (tool change)
#define vertStepPin 2
#define vertDirPin 48
#define toolChangeStepPin 46
#define toolChangeDirPin 44
//limit switch digital pins
#define topLimit 24
#define botLimit 26
//actuator pins
#define act1 38
#define act2 36
// 2-wire basic config, microstepping is hardwired on the driver
//BasicStepperDriver stepper2(MOTOR_STEPS2, toolChangeDirPin, toolChangeStepPin);

//RELAY DEFINITIONS
#define DRILL  13
#define PROBE  12
#define PUMP  11

#define VALVE1  10
#define VALVE2  9
#define VALVE3  8
#define VALVE4  3

const static void (*funcMap[])() = {  &drillDown,   //Command 0 Turns Drill on and goes down
                                      &drillUp,     //Command 1 Turns Drill on and goes up
                                      &heaterOn,    //Command 2 Turns Heater on and waits
                                      &drillOn,     //Command 3 Turns the drill on and waits
                                      &pumpOn,      //Command 4 Turns the pump on and waits
                                      &toolChangeCW,//Command 5 Turns the tool changer Clockwise
                                      &toolChangeCC,//Command 6 Turns the tool changer CounterClockwise
                                      &toolRealease,//Command 7 Pulls the node cone up and releases the tool
                                      &toolGrab,    //Command 8 Puts the nose cone down to grab tool
                                      &heaterDown,  //Command 9 Turns the heater on and moves down
                                      &heaterUp,    //Command 10 Turns the heater on and moves up
                                      &goUp,        //Command 11 Move up with nothing on
                                      &goDown};     //Command 12 Move down with nothing on

void setup() {

    Serial.begin(115200);

    //initialize stepper motors
    pinMode(vertStepPin,OUTPUT);
    pinMode(vertDirPin,OUTPUT);
    pinMode(act1,OUTPUT);
    pinMode(act2,OUTPUT);

    //initializes limit switches
    pinMode(topLimit,INPUT);
    pinMode(botLimit,INPUT);

    //initializes relays
    pinMode(DRILL,OUTPUT);
    pinMode(PUMP,OUTPUT);
    pinMode(PROBE,OUTPUT);
    pinMode(VALVE1,OUTPUT);
    pinMode(VALVE2,OUTPUT);
    pinMode(VALVE3,OUTPUT);
    
    //Turns everything off
    stopAll();

    //This code initializes force sensor
    forceSensor.begin(loadCellData, loadCellClk);
    forceSensor.set_scale(420.0983); // loadcell factor 5 KG
    //forceSensor.tare(); //zeroes load cell

}//end setup

void loop()
{  
  T.tick();//Update the timer
  ISense.updateIrms();//Get the latest current reading
  checkIrms();//Check to make sure we're not drawing too much current
  updateState();//update what we're doing
} 

void updateState(){
    if(!Serial){
      stopAll();//If the computer is disconeted STOP EVERYTHING
      //digitalWrite(LED_BUILTIN, HIGH);
    }else{
      if(Serial.available() > 0){//If a new state is being commanded from matlab
        //digitalWrite(LED_BUILTIN, LOW);
        int srefTemp = Serial.parseInt();
        Serial.read();

        if(sref != srefTemp){//If the state is new

          sref = srefTemp; //Update the real sref
          stopAll(); // if the state changed go back to the safe state this makes it easy to go to any other state

        }
      }
    }

    for(size_t i; !T.empty(); i++){
        delay(1);
        T.tick();//Wait for the last task to finish before starting the next one
        if(i > 10000){
          stopAll();//If we're waiting more than a full second for a task to complete turn everything off
        }
    }

    if(sref <= 12 && sref >= 0){
      //Serial.println(sref);
      funcMap[sref](); //Call the function according to the state we're in based on the map at the top of the code
    }else{
      stopAll();//If the state is invalid turn everything off This shouldn't be nessary it should happen on the state change but it's here for redundancy
    }
}

void drillDown(){
  if(distance%200 >= 150 && forceSensor.is_ready()){
    printDigitalCore();
  }
  drillOn();
  goDown();
}

void heaterDown(){
  heaterOn();
  goDown();
}

void drillUp(){
  drillOn();
  goUp();
}

void heaterUp(){
  heaterOn();
  goUp();
}

void toolChangeCW(){
  digitalWrite(toolChangeDirPin, HIGH);//Low for down
  stepMotor(toolChangeStepPin);
}

void toolChangeCC(){
  digitalWrite(toolChangeDirPin, LOW);//Low for down
  stepMotor(toolChangeStepPin);
}

void checkIrms() {
    while(ISense.getLastIrms() > UPPER_LIMIT) {
      stopAll();
      ISense.updateIrms();
      Serial.print("CURRENT EXCEEDED UPPED LIMIT :");
      Serial.println(ISense.getLastIrms());
      sref = -1;
    }
}

void stopAll(){
    digitalWrite(DRILL,HIGH);//all relays should be HIGH to be off
    digitalWrite(PUMP,HIGH);
    digitalWrite(PROBE,HIGH);
    digitalWrite(VALVE1, HIGH);
    digitalWrite(VALVE2, HIGH);
    digitalWrite(VALVE3, HIGH);
}

void goDown(void){

  //continue until signal change or limit switch
  if(!digitalRead(botLimit)){
    stepDrillDown();
  }else{
    //TODO set dist to lower val
  }
}

void stepMotor(int motorPin){
  
  dWrite *dat = malloc(sizeof(dWrite));

  dat->pin = motorPin;
  dat->val = LOW;

	uint8_t bit = digitalPinToBitMask(motorPin);
	uint8_t port = digitalPinToPort(motorPin);
	volatile uint8_t *out;

	out = portOutputRegister(port);

	*out |= bit;

  T.in(stepDelay, &stepDown, (void*)dat);
  
}

bool stepDown(void *dat){

  dWrite args = *((dWrite *)dat);

  uint8_t bit = digitalPinToBitMask(args.pin);
	uint8_t port = digitalPinToPort(args.pin);
	volatile uint8_t *out;

	out = portOutputRegister(port);

	*out &= ~bit;
	
  T.in(stepDelay*5, [](dWrite * d){free(d);}, dat);//This saves the dat struct and makes sure the code blocks until both steps are complete before moving on to the next state

  return false;
}

void stepDrillDown(){

  digitalWrite(vertDirPin, HIGH);//High for descent
  stepMotor(vertStepPin);
  distance++;
}

void stepDrillUp(){

  digitalWrite(vertDirPin, LOW);//Low for down
  stepMotor(vertStepPin);
  distance--;

}

void goUp(void)
{
  if(!digitalRead(topLimit)){ //reverses unless stopped or limit switch is hit
    stepDrillUp();
  }else{
    distance = 0;
  }
}

void heaterOn(void){
  digitalWrite(PROBE,LOW); 
}

void pumpOn(void){
  digitalWrite(PUMP, LOW);
}

void drillOn(void){
  digitalWrite(DRILL,LOW);
}

void valveOpen(int valvePin){
  digitalWrite(valvePin, LOW);
}

void printDigitalCore(void)
{
    if(forceSensor.is_ready()){
        auto Irms = ISense.getLastIrms();// Calculate Irms only
        auto force = forceSensor.get_units(1); //averages 1 readings for output
        Serial.print(distance);
        Serial.print(" ");
        Serial.print(force);
        Serial.print(" ");
        Serial.println(Irms);
    }
}

void toolRealease(){
    digitalWrite(act1,LOW);
    digitalWrite(act2, HIGH);
}

void toolGrab(){
    digitalWrite(act1,HIGH);
    digitalWrite(act2,LOW);
}
