// Libraries
// These must be downloaded in the Arduino IDE. Use the libraries tab to search for the name of the library and the author.

#include <Wire.h> //Default library (uses pinMode, etc.)
#include <string.h> //Default library (for printing strings)
#include <Servo.h> //PWMServo by Jim Studt et al.
#include <PID_v1.h> //Arduino-PID-Library by Brett Beauregard
#include <BasicLinearAlgebra.h> //BasicLinearAlgebra by Tom Stewart

// Pin setup
#define VOLTAGE_SENSOR A1
#define PITOT A2
#define PITCH_CONTROL 5
#define POWER_SWITCH_RELAY 28
#define POWER_METER_RELAY 40
#define E_BUTTON 22
//#define DUMP_LOAD 27


//Other variables
#define DISCONNECT_VOLTAGE 0.1
#define SMALL_READER_RESISTOR 47.0
#define LARGE_READER_RESISTOR 470.0

//Encoder index
#define ENCODER_PIN_INDEX 2  

//Servos
Servo pitchActuator;  //defines as linear actuator

//define pitch angles
//For the PQ12-R linear actuator, 1000 is fully extended and 2000 is fully retracted. Going beyond 1550 strains the actuator (measured in microseconds).
//Use the range 1000-1500 for best results.
#define INITIAL_PITCH 1025 // 1025 seemed to produce higher power than 1000 4/22
#define MIN_PITCH 1450
#define MAX_PITCH 1000
#define DURABILITY_PITCH 1450

// Arduino Mega constants
#define OPERATING_VOLTAGE 5.0
#define OPERATING_PRESSURE 4000     //Pitot anemometer has a range of -2kPa to 2kPa
#define ANALOG_RANGE 1023.0

// defining external and internal power
#define setExternal 0
#define setInternal 1

//defining brake posistions
#define BRAKE_ENGAGED 1
#define BRAKE_DISENGAGED 0
#define BRAKE_RELAY1 43
#define BRAKE_RELAY2 44

//defining constants for reading RPMs
volatile unsigned long previousIndexTime = 0;
volatile unsigned long currentIndexTime = 0;
volatile bool newRevolution = false;

//pitot calibration constants
float A = 1.34;// = 3.30
float B = 3.51; //= -25.11
float C = -5.88; // = 60.12
float pitotZero;

//defining constants for PID loop, precision is key
double Setpoint, Input, Output;
double Kp = 0.40;    //proportion constant, CHANGE
double Ki = 0.012;    //integral constant, CHANGE
double Kd = 0.2;    //derivative constant, CHANGE
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Resistor and rpm lists
float rpm_list[] = {5.0,5.0,5.0,5.0, 100.0, 1116.0, 1164.0, 1185.0, 1250.0, 1275.0, 1375.0, 1450.0, 1525.0, 1600.0, 1675.0};
float resistor_list[] = {0.0,0.0,0.0,0.0,0.0,195.0,103.0,75.0,55.0,37.0,35.0};    //correlates resistor with wind speed
// State Machine setup
typedef enum operatingStateMachine {
  START,
  BRAKE,
  POWER,
  CONTROL,
  DURABILITY,
  TEST
};

// Test State Machine setup
typedef enum testStateMachine {
  test_select,
  pwrsource,
  brake,
  pitch,
  setload,
  readrpm,
  readpsr,
  calpitot,
  windspeed,
  tunepid,
  designfair,
  endtest
};

// Defining current state
operatingStateMachine operatingState = START;
testStateMachine testState = test_select;

void setup() {
  //safety check!! dumps load immediately and sets all relays to OFF
  // pinMode(DUMP_LOAD, OUTPUT);
  // digitalWrite(DUMP_LOAD, LOW);
  for (int i = 23; i <= 33; i+=2){
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
  }

  delay(1500);  //let system catch up

  Serial.begin(9600);  // Set baud rate to 9600 bit/s
  Serial.println("Setting up...");
  analogReference(DEFAULT);

  //set up load
  // setLoad(5);
  // Serial.println("Load set up.");

  //set up pins
  setUpPins();
  Serial.println("Pins set up.");

  setPowerMode(setExternal);
  

  //set up pitch servo
  setUpServo();
  Serial.println("Servo set up.");

  //PID setup
  Input = readRPM();
  Setpoint = 1000;
  Output = INITIAL_PITCH;
  myPID.SetMode(AUTOMATIC);
  myPID.SetControllerDirection(REVERSE);
  myPID.SetOutputLimits(INITIAL_PITCH, MIN_PITCH);
  myPID.SetSampleTime(500); // 500ms between calculations


  delay(10000);
  //Pitot tube anemometer setup
  zeroPitot();
  Serial.print("Pitot zero point: ");
  Serial.println(pitotZero);
  Serial.println("Pitot anemometer zeroed. Wind tunnel may now start up.");

  //initialize
  setPitch(INITIAL_PITCH);
  setBrake(BRAKE_DISENGAGED);

  //Encoder setup
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_INDEX), indexISR, RISING);

  delay(3000);  //allow pitch servo to set up
}

void loop() {
  static bool testing = false;  //set variable for switching into testing state machine

  //display input
  if(Serial.available()){
    String input = Serial.readStringUntil('\n');
    while(Serial.available()){
      Serial.read();
    }
    Serial.println(input);

    //if input is "test", enter test state
    if(input.equals("test")){
      Serial.println("Entering test state...");
      operatingState = TEST;
    }
  }

  // State machine CONDITION switching
  switch (operatingState){

    case START:
    {
      //If e-stop pressed or load disconnected, emergency stop.
      if(isEStopPressed() || isLoadDisconnected()){
        operatingState = BRAKE;
      }
      //If wind speed > 13 m/s, switch to DURABILITY
      else if(readWindSpeed() >= 14.0){
        operatingState = DURABILITY;
      }
      //If wind speed > 10 m/s, switch to CONTROL
      else if(readWindSpeed() >= 11.0 ){
        operatingState = CONTROL;
      }
      //If rpms > 100, switch to POWER
      else if(readRPM() >= 1000){
        operatingState = POWER;
      }
      break;
    }

    case BRAKE:
    {
      //If button unpressed and load connected, switch to start
      if(!isEStopPressed() && !isLoadDisconnected()){
        operatingState = START;
      }
      break;
    }

    case POWER:
    {
      //Code for wind speeds < 10.0 m/s
     
      //If e-stop pressed or load disconnected, emergency stop.
      if(isEStopPressed() || isLoadDisconnected()){
        operatingState = BRAKE;
      }
      //If wind speed < 4 m/s, switch to START
      else if(readWindSpeed() < 4.0){
        operatingState = START;
      }
      //If wind speed > 10 m/s, switch to CONTROL
      else if(readWindSpeed() >= 10.7){
        //Setpoint = makeSetpoint();      //PID loop can now target setpoint made at measured rpms at 10 m/s
        operatingState = CONTROL;
      }
      break;

          
    }

    case CONTROL:
    {
      //Code for wind speeds > 10.5 m/s and <13.5 m/s
      //Should use pitch control to limit power

      //If e-stop pressed or load disconnected, emergency stop.
      if(isEStopPressed() || isLoadDisconnected()){
        operatingState = BRAKE;
      }
      //If wind speed > 13 m/s, switch to DURABILITY
      else if(readWindSpeed() >= 13.0){
        operatingState = DURABILITY;
      }
      //If wind speed < 9 m/s, switch to POWER
      else if(readWindSpeed() < 9.0){
        operatingState = POWER;
      }
      break;
    }

    case DURABILITY:
    {
      //Code for wind speeds > 13.5 m/s 
      // Should use pitch control to slow down rpms but still produce power
      
      //If e-stop pressed or load disconnected, emergency stop.
      if(isEStopPressed() || isLoadDisconnected()){
        operatingState = BRAKE;
      }
      //If wind speed < 12 m/s, switch to CONTROL
      else if(readWindSpeed() < 12.0){
        operatingState = CONTROL;
      }
      break;
    }

    case TEST:
    {
      // switch boolean "testing" = true to enter test state machine
      testing = true;
      break;
    }
    
  }


  // State machine ACTION switching
  switch (operatingState){

    case START:
    {
      Serial.print("START!!  ");
      Serial.print("  RPMs: ");
      Serial.print(readRPM());
      Serial.print("  Pressure: ");
      Serial.print(readPitot());
      Serial.print("  E-stop: ");
      Serial.print(digitalRead(E_BUTTON));
      Serial.print("  Voltage Sensor: ");
      Serial.print(readVoltage());
      Serial.print("  IsLoadDisconnected: ");
      Serial.println(isLoadDisconnected());

      //Code for starting up
      //should do nothing until cut in speed is detected
      setLoad(0);
      setBrake(BRAKE_DISENGAGED);   //disengage brake
      setPitch(INITIAL_PITCH);
      setPowerMode(setExternal);    //set power to External
      delay(500);
      break;
    }

    case BRAKE:
    {
      Serial.print("BRAKE!!  ");
      Serial.print("   RPMs: ");
      Serial.print(readRPM());
      Serial.print("   Voltage Reading: ");
      Serial.print(readVoltage());
      Serial.print("   E-stop: ");
      Serial.print(digitalRead(E_BUTTON));
      Serial.print("   Load connection: ");
      Serial.println(readVoltage());
      //Code for e-stop pressed or load disconnected
      //Should engage braking system and return to start after recconection
      setPitch(INITIAL_PITCH);
      //delay(1000);
      setBrake(BRAKE_ENGAGED);

      break;
    }

    case POWER:
    {
      Serial.print("POWER!!  ");
      Serial.print("Power: ");
      Serial.print(calcPower());
      Serial.print("  Pressure: ");
      Serial.print(readPitot());
      Serial.print("  Wind Speed: ");
      Serial.print(readWindSpeed());
      Serial.print("  RPMs: ");
      Serial.println(readRPM());
      //Code for wind speeds < 10.0 m/s
      //Should use torque feedforward loop to control rpms to maintain best tip speed ratio
      setPowerMode(setExternal);  //set power to internal
      setPitch(INITIAL_PITCH);        //set pitch to optimal angle to extract power out of the wind (TSR)
      if (readWindSpeed() >= 4.85  && readWindSpeed() < 5.85 ){
        setLoad(5);
      }
      else if (readWindSpeed() >= 5.85  && readWindSpeed() < 6.85 ){
        setLoad(6);
      }
      else if (readWindSpeed() >= 6.85  && readWindSpeed() < 7.85 ){
        setLoad(7);
      }
      else if (readWindSpeed() >= 7.85  && readWindSpeed() < 8.85){
        setLoad(8);
      }
      else if (readWindSpeed() >= 8.85  && readWindSpeed() < 9.85){
        setLoad(9);
      }
      else if (readWindSpeed() >= 9.85){
        setLoad(10);
        Setpoint = makeSetpoint();      //PID loop can now target setpoint made at measured rpms at 10 m/s
      }
      break;

    }

    case CONTROL:
    {
      if(Setpoint == 0.0){
        Setpoint = 2100;
      }
      Serial.print("CONTROL!!  ");
      Serial.print("Power: ");
      Serial.print(calcPower());
      Serial.print("  Setpoint: ");
      Serial.print(Setpoint);
      Serial.print("  RPMs: ");
      Serial.print(readRPM());
      Serial.print("  Output: ");
      Serial.print(Output);
      Serial.print("  Wind Speed: ");
      Serial.println(readWindSpeed());
      //Code for wind speeds > 10.5 m/s and <13.5 m/s
      //Should use pitch control to limit power
      setPowerMode(setExternal);

      // PID Loop
      Input = readRPM();
      double error = Setpoint - Input;

      // Deadband of ±50 RPM (adjust as needed)
      if (abs(error) < 50) {
          error = 0;
          // Feed modified error into PID
          Input = Setpoint - error;
      }
      myPID.Compute();

      // deadband and slow the servo's movement on the output too
      double deadbandOut = 3;
      double maxStep = 10;
      static double lastOutput = 1045;
      double delta = Output - lastOutput;
      if (abs(Output - lastOutput) < 3) {
          Output = lastOutput;  // freeze small changes
      }
      // Slew rate limiting
      if (delta > maxStep)      Output = lastOutput + maxStep;
      else if (delta < -maxStep) Output = lastOutput - maxStep;
      lastOutput = Output;
      setPitch(Output); 
      //Serial.println(Input);   
      break;
    }

    case DURABILITY:
    {
      Serial.print("DURABILITY!!");
      Serial.print("Power: ");
      Serial.print(calcPower());
      Serial.print("  RPMs: ");
      Serial.print(readRPM());
      Serial.print("  Wind Speed: ");
      Serial.println(readWindSpeed());
      //Code for wind speeds > 13 m/s 
      // Should use pitch control to slow down rpms but still produce power
      setPowerMode(setExternal);
      setPitch(DURABILITY_PITCH);
      break;
    }

    case TEST:
    {
      // actions happen in testing state machine
      break;
    }
  }


  //Testing state machine
  if (testing){
    switch(testState){

      case test_select:
      {
        selectTest();
        break;
      }
      case pwrsource:
      {
        testState = test_select;
        break;
      }

      case brake:
      {
        testState = test_select;
        break;
      }

      case pitch:
      {
        testState = test_select;
        break;
      }    

      case setload:
      {
        testState = test_select;
        break;
      }

      case readrpm:
      {
        testState = test_select;
        break;
      }

      case readpsr:
      {
        testState = test_select;
        break;
      }

      case calpitot:
      {
        testState = test_select;
        break;
      }

      case windspeed:
      {
        testState = test_select;
        break;
      }
      
      case tunepid:
      {
        testState = test_select;
        break;
      }

      case designfair:
      {
        testState = test_select;
        break;
      }
      case endtest:
      {
        Serial.println("Leaving test state");
        operatingState = START;
        testing = false;
        testState = test_select;
        break;
      }              
    }

    // Testing state machine ACTIONS
    switch(testState){

      case test_select:   //return to state transitions
      {
        break;
      }

      case pwrsource:
      {
        //external power if turbine is not running, internal power if turbine is running
        //get input, power externally or internally
        Serial.print("Type e to power externally or type i to power internally. \n");
        while(!Serial.available()){
        }
        String input = Serial.readStringUntil('\n');
        while(Serial.available()){
          Serial.read();
        }
        if(input.equals("i")){
          setPowerMode(setInternal);
          Serial.println("Internal power.");
        }
        else if(input.equals("e")){
          setPowerMode(setExternal);
          Serial.println("External power.");
        }
        else{
          Serial.println("Invalid entry. Returning to test selection. \n");
        }
        break;
      }

      case brake:
      {
        //manually engage or disengage brake
        Serial.print("Type e to engage brake or type d to disengage brake. \n");
        while(!Serial.available()) {
        }
        String input = Serial.readStringUntil('\n');
        while(Serial.available()){
          Serial.read();
        }
        if(input.equals("e")){
          setBrake(BRAKE_ENGAGED);
          Serial.println("Brake engaged.");
        }
        else if(input.equals("d")){
          setBrake(BRAKE_DISENGAGED);
          Serial.println("Brake disengaged.");
        }
        else{
          Serial.println("Invalid entry. Returning to test selection. \n");
        }
        break;
      }

      case pitch:
      {
        //manually change pitch
        Serial.print("Select a pitch value between 1000 and 1450, inclusive. \n");
        while(!Serial.available()){
        }
        String input = Serial.readStringUntil('\n');
        while(Serial.available()){
          Serial.read();
        }
        int inputInt = input.toInt();
        if(inputInt >= MAX_PITCH && inputInt <= MIN_PITCH){
        setPitch(inputInt);
        Serial.print("Pitch set to ");
        Serial.println(inputInt);
        }
        else{
          Serial.println("Invalid entry. Returning to test selection. \n");
        }
        break;
      }

      case setload:
      {
        //manually set load
        Serial.print("Enter the wind speed to set the corresponding resistance (5-10 m/s), or type 0 to release all loads. \n");
        while(!Serial.available()){
        }
        String input = Serial.readStringUntil('\n');
        while(Serial.available()){
          Serial.read();
        }
        int inputInt = input.toInt();
        if(inputInt >= 5 && inputInt <= 11 || inputInt == 0){
        setLoad(inputInt);
        Serial.print("Load set to ");
        Serial.println(resistor_list[inputInt]);
        }
        else{
          Serial.println("Invalid entry. Returning to test selection. \n");
        }
        break;
      }

      case readrpm:
      {
        //keep same pitch, record rpms
        //setPitch(INITIAL_PITCH);
        int samples = 50;
        float sumRPM = 0.0;
        for (int i=0; i<samples; i++){
          sumRPM += readRPM();
        }
        float avgRPM = sumRPM / samples;
        Serial.print("Avg RPM: ");
        Serial.println(avgRPM);
        break;
      }

      case readpsr:
      {
        Serial.println(readPitot());
        break;
      }

      case calpitot:
      {
        // we should set a load or a pitch so that it can't spin to fast and destroy itself
        setPitch(INITIAL_PITCH); // set pitch so that airflow in calibration mostly matches conditions (blades affect)
        calibratePitot();
        Serial.print("A:");
        Serial.println(A);
        Serial.print("B:");
        Serial.println(B);      
        Serial.print("C:");
        Serial.println(C);
        Serial.print("Zero:");
        Serial.println(pitotZero);
        break;
      }

      case windspeed:
      {
        Serial.print("Wind Speed: ");
        Serial.print(readWindSpeed());
        Serial.println("m/s");
        break;
      }

      case tunepid:
      {
        Serial.print("Choose which constant to tune. Type p, i, or d. \n");
        while(!Serial.available()){
        }
        String input = Serial.readStringUntil('\n');
        while(Serial.available()){
          Serial.read();
        }
        if(input=="p"){
          Serial.println("Enter a new value for Kp.");
          while(!Serial.available()){
          }
          String input = Serial.readStringUntil('\n');
          while(Serial.available()){
            Serial.read();
          }
          double inputDouble = input.toDouble();
          Kp = inputDouble;
          Serial.print("Kp: ");
          Serial.print(Kp);
          Serial.print("  Ki: ");
          Serial.print(Ki);
          Serial.print("  Kd: ");
          Serial.println(Kd);
        }
        else if(input=="i"){
          Serial.println("Enter a new value for Ki.");
          while(!Serial.available()){
          }
          String input = Serial.readStringUntil('\n');
          while(Serial.available()){
            Serial.read();
          }
          double inputDouble = input.toDouble();
          Ki = inputDouble;
          Serial.print("Kp: ");
          Serial.print(Kp);
          Serial.print("  Ki: ");
          Serial.print(Ki);
          Serial.print("  Kd: ");
          Serial.println(Kd);
        }
        else if(input=="d"){
          Serial.println("Enter a new value for Kd.");
          while(!Serial.available()){
          }
          String input = Serial.readStringUntil('\n');
          while(Serial.available()){
            Serial.read();
          }
          double inputDouble = input.toDouble();
          Kd = inputDouble;
          Serial.print("Kp: ");
          Serial.print(Kp);
          Serial.print("  Ki: ");
          Serial.print(Ki);
          Serial.print("  Kd: ");
          Serial.println(Kd);
        }
        else{
          Serial.println("Invalid entry. Returning to test state.");
          break;
        }
        Input = readRPM();
        myPID.Compute();
        setPitch(Output); 
        Serial.print("Setpoint: ");
        Serial.print(Setpoint);
        Serial.print("  Input: ");
        Serial.print(Input);
        Serial.print("  Output: ");
        Serial.println(Output);
        break;
      }

      case designfair:
      {
        //code to show off at the design fair because the full turbine isn't complete yet.
        //output power readings, cycle through pitches to show power
        setPowerMode(setExternal);
        float previous_wind = 0.0;
        while(!Serial.available()){
          for(int i=0; i<=4; i++){
            int pitch = 1150+i*70;
            setPitch(pitch);
            Serial.print("Pitch:");
            Serial.print(pitch);
            Serial.print("  RPMS:");
            Serial.print(readRPM());
            Serial.print("  Volts:");
            Serial.print(readVoltage());
            Serial.print("  Wind Speed:");
            float wind = readWindSpeed();

            if (!isnan(wind)) {
              Serial.println(wind);
              previous_wind = wind;   // update existing variable
            }
            else {
              Serial.println(previous_wind);
            }

            delay(5000);
          }
        }

        String input = Serial.readStringUntil('\n');
        while(Serial.available()){
          Serial.read();
        }
        Serial.println(input);
        if(input.equals("end")){
          break;
        }
        break;
      }

      case endtest:
      {
        //no state actions
        break;
      }

      
    }
  }
}

void setUpPins(){
  pinMode(E_BUTTON, INPUT_PULLUP);
  pinMode(VOLTAGE_SENSOR, INPUT); 
  pinMode(POWER_SWITCH_RELAY, OUTPUT);
  pinMode(POWER_METER_RELAY, OUTPUT);
  digitalWrite(POWER_METER_RELAY, HIGH);
  pinMode(BRAKE_RELAY1, OUTPUT);
  pinMode(BRAKE_RELAY2, OUTPUT);
  pinMode(ENCODER_PIN_INDEX, INPUT);
  pinMode(37, OUTPUT);      //pins 37 and 35 are the unused relays
  digitalWrite(37, HIGH);   //but have to be set to HIGH (off)
  pinMode(35, OUTPUT);
  digitalWrite(35, HIGH);
  pinMode(33, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(23, OUTPUT);

}

void setUpServo(){
  //initialize pitch control actuator
  pitchActuator.attach(PITCH_CONTROL);
  setPitch(INITIAL_PITCH);
}


bool isEStopPressed(){
  return !digitalRead(E_BUTTON);
}

bool isLoadDisconnected() {
    static float lastVoltage = 0;
    static bool loadDisconnected = false;

    float voltage = readVoltage();

    // Thresholds
    const float UPPER_CONNECT_THRESHOLD    = 11.75;  // rising above this = connected
    const float UPPER_DISCONNECT_THRESHOLD = 11.45;
    const float LOWER_DISCONNECT_THRESHOLD = 11.24;
    const float LOWER_CONNECT_THRESHOLD = 11.12;  // falling below this = probably just external power

    // Detect falling edge (normal → low)
    if (!loadDisconnected && voltage < UPPER_DISCONNECT_THRESHOLD && voltage > LOWER_DISCONNECT_THRESHOLD && lastVoltage >= UPPER_CONNECT_THRESHOLD) {
        loadDisconnected = true;
    }
    // Detect jumping into disconnect range from external power
    if (!loadDisconnected && voltage > LOWER_DISCONNECT_THRESHOLD && voltage < UPPER_DISCONNECT_THRESHOLD && lastVoltage < LOWER_CONNECT_THRESHOLD) {
        loadDisconnected = true;
    }

    // Detect rising edge (low → normal)
    if (loadDisconnected && voltage > UPPER_CONNECT_THRESHOLD) {
        loadDisconnected = false;
    }
    // Detect on external power but without turbine power
    if (loadDisconnected && voltage < LOWER_CONNECT_THRESHOLD) {
        loadDisconnected = false;
    }

    lastVoltage = voltage;
    return loadDisconnected;
}

void setPowerMode(int mode){
  if(mode == setExternal){
    digitalWrite(POWER_SWITCH_RELAY, HIGH);
  }
  else{
    digitalWrite(POWER_SWITCH_RELAY, LOW);

  }
}

void setBrake(bool brakePosition){    //brake uses transistor switch to change circuits, just needs signal from arduino

  if(brakePosition == BRAKE_ENGAGED){
    digitalWrite(BRAKE_RELAY1, LOW);
    digitalWrite(BRAKE_RELAY2, LOW);
  }
  else{
    digitalWrite(BRAKE_RELAY1, HIGH);
    digitalWrite(BRAKE_RELAY2, HIGH);
  }
}

void setPitch(int pitchAngle){
  //set servo to pitch angle
  pitchActuator.writeMicroseconds(pitchAngle);
}

void setLoad(int wind_speed) {
  if (wind_speed == 0){
    for (int i = 23; i <= 33; i += 2) {
      digitalWrite(i, HIGH);
    }
  }

  else if (wind_speed < 5 || wind_speed > 10) {
    Serial.println("Invalid wind speed to load conversion.");
    return;
  }

  else{
    Serial.println("Successful wind speed to load conversion.");
    int newRelay = -2 * wind_speed + 43;    //converts wind speed to pin number
                                            // 5 m/s = 33
                                            // 6 m/s = 31
                                            // 7 m/s = 29
                                            // 8 m/s = 27
                                            // 9 m/s = 25
                                            // 10 m/s = 23
    digitalWrite(newRelay, LOW);

    for (int i = 23; i <= 33; i += 2) {
      if (i != newRelay) {
        digitalWrite(i, HIGH);
      }
    }
  }
}

void indexISR(){      //runs everytime a revolution is completed, used for readRPM
  previousIndexTime = currentIndexTime;
  currentIndexTime = micros();  //get new time
  newRevolution = true;
}

float readRPM() {
  static float rpm = 0;
  static float filteredRPM = 0;   // holds the smoothed value
  const float alpha = 0.6;        // smoothing factor

  if (newRevolution == true) {
    newRevolution = false;
    unsigned long timeInterval = currentIndexTime - previousIndexTime;
    rpm = (60.0 * 1000000.0) / timeInterval;
  }

  // Reject impossible spikes
  if (rpm > 5000) rpm = 0.0;

  // Apply exponential smoothing
  filteredRPM = filteredRPM + alpha * (rpm - filteredRPM);

  return filteredRPM;
}

float makeSetpoint(){
  float rpmSetpoint = 0.0;
  if (readWindSpeed() > 10.3 && readWindSpeed() < 11.2){
      rpmSetpoint = readRPM();
  }
  else {
      Serial.println("Unable to make RPM setpoint for 10 m/s");
  }
  return rpmSetpoint;
}

float readLoad(){
  //read resistance by which relay is on
  for (int i=23; i<=33; i+=2){
    if (digitalRead(i)==LOW){
      return resistor_list[-(i-43)/2];   //converts pin number to resistor list value
    }
  }
}

void zeroPitot(){
  float sum = 0;
  int n = 500;
  for (int i = 0; i < n; i++){
    float voltage = analogRead(PITOT)*(OPERATING_VOLTAGE / ANALOG_RANGE);
    //Serial.print("Pitot Reading: ");
    //Serial.print(analogRead(PITOT));
    //Serial.print("Voltage: ");
    //Serial.println(voltage);
    float pressure = (voltage - 2.5)*1000.0;
    sum += pressure;
  }
  pitotZero = sum/n;   //converts pressure reading to actual pressure over bit range
}

float inputWindSpeed(){   // read manually entered wind speed for calibration
  Serial.println("Enter wind speed between 5 and 10 m/s:");
  while(Serial.available() == 0){
    delay(100);
  }
  String input = Serial.readStringUntil('\n');
  while(Serial.available()){
    Serial.read();
  }
  float speed = input.toFloat();
  setLoad(speed); // try setting the load you want because the calibration is sensitive to turbine RPM
  Serial.print("Wind speed of ");
  Serial.print(speed, 1);   //prints float to 1 decimal place
  Serial.println("m/s entered.");
  return speed;
}

void calibratePitot(){    //calibrates pitot anemometer based on 3 known wind speeds
  float windSpeedArray[3];
  float pressureArray[3];
  for(int i = 0; i<3; i++){
    windSpeedArray[i] = inputWindSpeed();
    float pressureSum = 0.0;
    int n = 200;
    for(int j = 0; j<n; j++){
      float pressureRead = readPitot();
      pressureSum += pressureRead;
      Serial.print("Pressure Reading: ");
      Serial.println(pressureRead);
    }
    float pressureAvg = pressureSum / n;
    pressureArray[i] = pressureAvg;
    Serial.println(pressureAvg);
  }
  Serial.println("Calibrating...");

  // Matrices to hold the pressures and quadratic fit data. These arrays will be used to solve the matrix 
  // problem Ax = B, where A is the quadratic fit matrix based on the wind speeds, x is the values a, b, and c 
  // in the quadratic equation y = ax^2 + bx + c, and B is the output pressure values. We will be solving for 
  // a, b, and c

  BLA::Matrix<3, 1> pressures;
  pressures = {pressureArray[0], pressureArray[1], pressureArray[2]};
  BLA::Matrix<3, 3> quadFit;
  quadFit = {pow(windSpeedArray[0], 2), windSpeedArray[0], 1, 
             pow(windSpeedArray[1], 2), windSpeedArray[1], 1, 
             pow(windSpeedArray[2], 2), windSpeedArray[2], 1};

  //solve for a, b, and c
  Invert(quadFit);
  BLA::Matrix<3, 1> abc = quadFit * pressures;
  A = abc(0,0);
  B = abc(1,0);
  C = abc(2,0);
  Serial.println("Calibration complete.");
}

float readPitot(){
  float sum = 0.0;
  int n = 500;
  for (int i=0; i<n; i++) {
    float voltage = analogRead(PITOT)*(OPERATING_VOLTAGE/ANALOG_RANGE);
    float pressureTrue = (voltage-2.5)*1000.0;   //converts pressure reading to actual pressure over bit range
    sum += pressureTrue;
  }
  float pressureAvg = sum/n;
  return pressureAvg - pitotZero;
}

float readWindSpeed(){
  float pressureAvg = readPitot();
  
  if(pow(B, 2) - 4*A*(C - pressureAvg)<0){
    //Serial.println("Error reading wind speed, negative number in sqrt");
  }
  else if (A == 0){
    //Serial.println("Error reading wind speed, divide by zero, A = 0");
  }
  else{
    //Serial.println("Calculating wind speed...");
  }
  float windSpeed = (-B + sqrt(pow(B, 2) - 4*A*(C - pressureAvg))) / (2*A);
  return windSpeed;
}

float readVoltage() {   //reads in voltage measured across voltage divider
  float sum = 0.0;
  int samples = 5;
  for (int i=0; i<samples; i++) {
    float voltsIn = analogRead(VOLTAGE_SENSOR);
    float voltsDivided = (voltsIn / ANALOG_RANGE) * OPERATING_VOLTAGE; //converts volts to actual volts over the range of the microcontroller and the bit count
    float voltsTrue = voltsDivided / (SMALL_READER_RESISTOR/(SMALL_READER_RESISTOR + LARGE_READER_RESISTOR));   //from old resistor bank
    sum += voltsTrue;
  }
  float voltsTrueAvg = sum/samples;
  return voltsTrueAvg;
}

float calcPower(){
  float voltage = readVoltage()-11.08; // account for external power being there, that isn't real power in the load
  float resistance = readLoad();
  float power = voltage*voltage/resistance;
  return power;
}

void selectTest(){
  Serial.print("Enter a test. Options: pwrsource, brake, pitch, setload, readrpm, readpsr, calpitot, windspeed, tunepid, designfair, endtest \n");
  while(!Serial.available()){
  }
  String input = Serial.readStringUntil('\n');
  while(Serial.available()){
    Serial.read();
  }  

  if (input.equals("pwrsource")){
    testState = pwrsource;
  }
  else if (input.equals("brake")){
    testState = brake;
  }
  else if (input.equals("pitch")){
    testState = pitch;
  }
  else if (input.equals("setload")){
    testState = setload;
  }
  else if (input.equals("readrpm")){
    testState = readrpm;
  }
  else if (input.equals("readpsr")){
    testState = readpsr;
  }
  else if (input.equals("calpitot")){
    testState = calpitot;
  }
  else if (input.equals("windspeed")){
    testState = windspeed;
  }
  else if (input.equals("tunepid")){
    testState = tunepid;
  }
  else if (input.equals("designfair")){
    testState = designfair;
  }
  else if (input.equals("endtest")){
    testState = endtest;
  }
  else{
    Serial.println("Invalid entry.");
  }
}









