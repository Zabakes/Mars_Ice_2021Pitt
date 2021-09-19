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

#define dataPin 21
#define clockPin 53

struct dWrite{
    int pin;
    int val;
};

Timer<> T;
HX711 forceSensor;
CurrentMon ISense(0, 0, 0, 400, 1, T);

#define UPPER_LIMIT 8.9 //If the current goes above this stop everything

#define MOTOR_STEPS 800 
#define RPM 120 

//calculation for length of delay used to control vertical speed (microseconds)
unsigned long stepDelay = 60.0/(long(RPM)*long(MOTOR_STEPS)*2)*1000000; 
int distance = 0; //step count

// All the wires needed for full functionality; motor 1 (vertical) and motor2 (tool change)
#define stepPin 2
#define dirPin 3
#define stepPin2 4
#define dirPin2 5
//limit switch digital pins
#define topLimit 14
#define botLimit 15
//actuator pins
#define act1 34
#define act2 36
// 2-wire basic config, microstepping is hardwired on the driver
//BasicStepperDriver stepper2(MOTOR_STEPS2, dirPin2, stepPin2);

//RELAY DEFINITIONS
#define DRILL  8
#define PROBE  9
#define PUMP  10

/* #define VALVE1  11
#define VALVE2  12
#define VALVE3  13
#define VALVE4   */

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
    pinMode(stepPin,OUTPUT);
    pinMode(dirPin,OUTPUT);
    pinMode(act1,OUTPUT);
    pinMode(act2,OUTPUT);

    //initializes limit switches
    pinMode(topLimit,INPUT);
    pinMode(botLimit,INPUT);

    //initializes relays
    pinMode(DRILL,OUTPUT);
    pinMode(PUMP,OUTPUT);
    pinMode(PROBE,OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    /* pinMode(VALVE1,OUTPUT);
    pinMode(VALVE2,OUTPUT);
    pinMode(VALVE3,OUTPUT); */
    
    //Turns everything off
    stopAll();

    //This code initializes force sensor
    forceSensor.begin(dataPin, clockPin);
    forceSensor.set_scale(420.0983); // loadcell factor 5 KG
    //forceSensor.tare(); //zeroes load cell

    Serial.println("leaving setup");
}//end setup

void loop()
{  
  T.tick();//Update the timer
  ISense.updateIrms();//Get the latest current reading
  checkIrms();//Check to make sure we're not drawing too much current
  updateState();//update what we're doing
} 

void updateState(){
    Serial.println("Updating State");
    if(!Serial){
      stopAll();//If the computer is disconeted STOP EVERYTHING
      digitalWrite(LED_BUILTIN, HIGH);
    }else{
      digitalWrite(LED_BUILTIN, LOW);
      if(Serial.available()){//If a new state is being commanded from matlab

        int srefTemp = Serial.parseInt();

        if(sref != srefTemp){//If the state is new

          sref = srefTemp; //Update the real sref
          stopAll(); // if the state changed go back to the safe state this makes it easy to go to any other state
          Serial.println("New State");
        }
      }
    }

    for(size_t i; !T.empty(); i++){
        delay(1);
        T.tick();//Wait for the last task to finish before starting the next one
        if(i > 1000){
          stopAll();//If we're waiting more than a full second for a task to complete turn everything off
          if(!(i%100)){
            Serial.print("Waiting for tasks for a long time");
          }
        }
    }

    if(sref <= 12 && sref >= 0){
      funcMap[sref](); //Call the function according to the state we're in based on the map at the top of the code
    }else{
      stopAll();//If the state is invalid turn everything off This shouldn't be nessary it should happen on the state change but it's here for redundancy
    }
}

void drillDown(){
  Serial.println("Drill Down");
  if(distance%200 >= 150 && forceSensor.is_ready()){
    printDigitalCore();
  }
  drillOn();
  goDown();
}

void heaterDown(){
  Serial.println("Heater Down");
  heaterOn();
  goDown();
}

void drillUp(){
  Serial.println("Drill Up");
  drillOn();
  goUp();
}

void heaterUp(){
  Serial.println("Heater Up");
  heaterOn();
  goUp();
}

void toolChangeCW(){
  Serial.println("Toolchange CW");
}

void toolChangeCC(){
  Serial.println("Toolchange CC");
}

void checkIrms() {
    Serial.print("Checking Irms :");
    Serial.println(ISense.getLastIrms());
    while(ISense.getLastIrms() > UPPER_LIMIT) {
      stopAll();
      ISense.updateIrms();
      Serial.println("CURRENT EXCEEDED UPPED LIMIT");
      sref = -1;
    }
}

void stopAll(){
    Serial.println("Entering Stop");
}

void goDown(void){

  //continue until signal change or limit switch
  Serial.println("Going Down");

  if(!digitalRead(botLimit)){
    //stepDrillDown();
  }else{
    Serial.println("Hit Lower limit");
  }
}

void stepMotor(int motorPin){
  
  dWrite dat = {
    motorPin,
    LOW
  };

  digitalWrite(stepPin,HIGH);
  T.in(stepDelay, &digitalWrite, (void*)&dat);

}

bool digitalWrite(void *dat){
  dWrite args = *((dWrite *)dat);
  digitalWrite(args.pin, args.val);
  return false;
}

void stepDrillDown(){

  digitalWrite(dirPin, HIGH);//High for descent
  stepMotor(stepPin);
  distance++;
}

void stepDrillUp(){

  digitalWrite(dirPin, LOW);//Low for down
  stepMotor(stepPin);
  distance--;

}

void goUp(void)
{
  Serial.println("Going Up");
  if(!digitalRead(topLimit)){ //reverses unless stopped or limit switch is hit
    //stepDrillUp();
  }else{
    Serial.println("Hit Upper limit");
  }
}

void heaterOn(void){
  Serial.println("Heater On");
}

void pumpOn(void){
  Serial.println("Pump On");
}

void drillOn(void){
  Serial.println("drill On");
}

void valveOpen(int valvePin){
  digitalWrite(valvePin, LOW);
}

void printDigitalCore(void)
{
    Serial.println("Print DigitalCore");
}

void toolRealease(){
  Serial.println("Tool Realease");
}

void toolGrab(){
  Serial.println("Tool Grab");
}
