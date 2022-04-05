#include <PID_v2.h>
#include <ELMduino.h>
#include <Wire.h>
#include <EEPROM.h>

#define PA03
#define RPWM1   5
#define LPWM1   4
#define RPWM2   5
#define LPWM2   4
#define BRAKE   6

#ifndef PA03  //hall effect feedback flag undefined; use potetiometer feedback
#define POS1    A0 //position feedback for motor1
#define POS2    A1 //position feedback for motor2
#else          //hall effect feedback flag set; define inputs for signal 1 and signal 2 for both actuators
#define POS1_1    2   //has to be interrupt pin
#define POS1_2    14  //digital equivalent of A0
#define POS2_1    3   //has to be interrupt pin
#define POS2_2    15  //digital equivalent of A1
#endif //PA03

#define CS1     A2 //current sensor for first actuator
#define CS2     A3 //current sensor for second sensor

#define sgn(x) ((x)/abs(x))

float kp=2.; //tune it! the more it is, the faster motors responds
float ki=.05;
float kd=0;

const unsigned int kpAddress=0; //addresses in eeprom
const unsigned int kiAddress=4;
const unsigned int kdAddress=8;

double syncRatio=1.0; //ratio between output to first actuator and output to second actuator

double digitalZeroMm; //0mm offset
double digitalSpan; //span between 0mm and maximum length in digital value
const double mmSpan = 100.; //span of actuator in mm

PID_v2 cont(kp,ki,kd,PID::Direct); //PID controller object

unsigned int START_MILLIS; //to record start time since boot, in ms

//Utility functions
//translates extension in mm to wing angle
double mmToDeg(double mm){
  return 90+mm/mmSpan*80;
}
//reverse of prev function
double degToMm(double deg){
  return (deg-90)/80*mmSpan;
}
//get desired wing angle given vehicle speed (kph) and target effective wind speed (kph)
double calcWingAngle(double vehicleSpd,double targetSpd){
  if (vehicleSpd<targetSpd) //too slow
    return 170;
  return (180-degrees(acos(targetSpd/vehicleSpd)));
}
//get digital setpoint given distance setpoint in mm
int calcSetpointFromMm(double mm){
  return digitalZeroMm+mm/mmSpan*digitalSpan;

}
//write to actuator
void writeToActuator(int u, int rpin, int lpin){
  if (u>0){
    analogWrite(rpin,0);
    analogWrite(lpin,u);
  }
  else{
    analogWrite(lpin,0);
    analogWrite(rpin,-u);
  }
}

//now lets define macros for position of the actuators
#ifdef PA03
int pos1=0;
int pos2=0;
bool tmp=LOW;
void firstActuatorISR(){
  if (digitalRead(POS1_2)){
    pos1++;
  }
  else {
    pos1--;
  }
  tmp=!tmp;
  digitalWrite(LED_BUILTIN,tmp);
}
void secondActuatorISR(){
  if (digitalRead(POS2_2)){
    pos2++;
  }
  else {
    pos2--;
  }
}
#else
#define pos1 (analogRead(POS1))
#define pos2 (analogRead(POS2))
#endif

ELM327 elm327;

struct WingData {
  uint16_t pos;
  float cs1;
  float cs2;
}wingData; //has to total less than or equal to 32 bytes long

struct VehicleData{
  uint8_t kph;
}vehicleData; //the 32 byte limit also holds here

void updateWingData(){
  
  #ifdef PA03
  wingData.pos=0;// todo: get data from quadrature counter
  #else
  wingData.pos=(analogRead(POS1)+analogRead(POS2))/2;
  #endif
  
  wingData.cs1=(analogRead(CS1)-512)/20.5;
  wingData.cs2=(analogRead(CS2)-512)/20.5;
}

void updateVehicleData(){  // a function to get vehicle speed
  static uint8_t kph=0;
  uint8_t tempKph=elm327.kph();
  if (elm327.nb_rx_state==ELM_GETTING_MSG) vehicleData.kph=kph;
  else if (elm327.nb_rx_state==ELM_SUCCESS) vehicleData.kph=kph=tempKph;
  else elm327.printError();
}


//alpha filter for setpoint and feedback
double setpoint=0.0;
double alpha=.95;

void setup()
{
  //turn serial on
  //this port will be used for debugging
  Serial.begin( 115200 );
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  Serial.println("Start");

  //this port is to communicate with the HC05 module paired to the ELM327 dongle
  Serial1.begin(9600);
  Serial1.println("ATZ"); //to reset the hc05 module
  delay(1000);
  while(Serial1.available() > 0)
  {
    Serial1.read();
  }
  elm327.begin(Serial1,true,2000); //init elm327 to Serial1 port (pins d18,d19)

  //initialize i2c for communication with server
  Wire.begin(8);
  Wire.onRequest(handleWireRq);
  Wire.onReceive(handleWireRcv);

  //get saved gains from eeprom
  EEPROM.get(kpAddress,kp);
  EEPROM.get(kiAddress,ki);
  EEPROM.get(kdAddress,kd);
  
  //initialize IOs
  pinMode(RPWM1,OUTPUT);
  pinMode(LPWM1,OUTPUT);
  pinMode(RPWM2,OUTPUT);
  pinMode(LPWM2,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BRAKE,INPUT_PULLUP);
  analogWrite(RPWM1,0);
  analogWrite(LPWM1,0);
  analogWrite(RPWM2,0);
  analogWrite(LPWM2,0);

#ifdef PA03
  pinMode(POS1_1,INPUT_PULLUP);
  pinMode(POS1_2,INPUT_PULLUP);
  pinMode(POS2_1,INPUT_PULLUP);
  pinMode(POS2_2,INPUT_PULLUP);
#endif //PA03
  int del=5000;
  //retract
  analogWrite(RPWM1,1023);
  analogWrite(RPWM2,1023);
  delay(del);
  analogWrite(RPWM1,0);
  analogWrite(RPWM2,0);
  
#ifdef PA03
  attachInterrupt(digitalPinToInterrupt(POS1_1),firstActuatorISR,RISING);
  attachInterrupt(digitalPinToInterrupt(POS2_1),secondActuatorISR,RISING);
#endif //PA03
  Serial.println((pos1+pos2)/2);
  //extend and record maximum value
  analogWrite(LPWM1,1023);
  analogWrite(LPWM2,1023);
  delay(del);
  analogWrite(LPWM1,0);
  analogWrite(LPWM2,0);
  digitalSpan=(pos1+pos2)/2;
  Serial.println(digitalSpan);
  //retract and record offset
  analogWrite(RPWM1,1023);
  analogWrite(RPWM2,1023);
  delay(del);
  analogWrite(RPWM1,0);
  analogWrite(RPWM2,0);
  digitalZeroMm=(pos1+pos2)/2;
  digitalSpan-=digitalZeroMm;
  Serial.println(digitalZeroMm);

  //initialize controller
  cont.SetOutputLimits(-1023,1023);
  cont.Start((pos1+pos2)/2,0,0);
  
  //record start time
  START_MILLIS=millis();
  
  //initialize filters
  wingData.pos=(pos1+pos2)/2;
  setpoint=wingData.pos;
}

void handleWireRq(){ //handle server request
  static uint8_t frameId=0;
  if (frameId==0) Wire.write((char *)&wingData,sizeof(WingData));
  else if (frameId==1) Wire.write((char *)&vehicleData,sizeof(VehicleData));
  frameId=(frameId+1)%2;
}
void handleWireRcv(int sz){ //handle server write; to update one of the gains
  if (byte(Wire.read())==byte('p')) //update kp
  {
    while (!Wire.available()); //wait for data
    for (int i = 0; i < sizeof(float); i ++){ //update kp byte by byte
      ((byte *)&kp)[i]=byte(Wire.read());
    }
    EEPROM.put(kpAddress,kp);
  }
  else if (byte(Wire.read())==byte('i')) //update ki
  {
    while (!Wire.available()); //wait for data
    for (int i = 0; i < sizeof(float); i ++){ //update ki byte by byte
      ((byte *)&ki)[i]=byte(Wire.read());
    }
    EEPROM.put(kiAddress,ki);
  }
  else if (byte(Wire.read())==byte('d')) //update kd
  {
    while (!Wire.available()); //wait for data
    for (int i = 0; i < sizeof(float); i ++){ //update kd byte by byte
      ((byte *)&kd)[i]=byte(Wire.read());
    }
    EEPROM.put(kdAddress,kd);
  }
}

void loop()
{
  //always update data
  updateWingData();
  updateVehicleData();
  if (digitalRead(BRAKE)){ //break the loop if brake not hit
    //but first retract the wing
    writeToActuator(-1023,RPWM1,LPWM1);
    writeToActuator(-1023,RPWM2,LPWM2);
    return;
  }
  
  //read and filter current position
  double angle=calcWingAngle(vehicleData.kph,33);
  double mm=degToMm(angle);
  
  //calculate and filter setpoint
  setpoint = setpoint*alpha + calcSetpointFromMm(mm)*(1-alpha);  

  //todo : calculate sync ratio to optimize current usage on both actuators
  //calculate sync ratio
  syncRatio=1.0;
  //update cotroller output
  cont.Setpoint(setpoint);
  double u1=cont.Run(wingData.pos);
  double u2=u1/syncRatio;
    
  //to work around nonlinearity effect
  const int tol=50;
  const int hst=50;
  
  //write to first actuator
  if (abs(u1)>tol){
    writeToActuator(u1,RPWM1,LPWM1);
  }
  else if (abs(u1)>tol-hst){
    double x=(tol-(tol-abs(u1))/hst*tol)*sgn(u1);
    writeToActuator(x,RPWM1,LPWM1);
  }
  else{
    writeToActuator(0,RPWM1,LPWM1);
  }

  //write to second actuator
  if (abs(u2)>tol){
    writeToActuator(u2,RPWM2,LPWM2);
  }
  else if (abs(u2)>tol-hst){
    double x=(tol-(tol-abs(u2))/hst*tol)*sgn(u2);
    writeToActuator(x,RPWM2,LPWM2);
  }
  else{
    writeToActuator(0,RPWM2,LPWM2);
  }
}
