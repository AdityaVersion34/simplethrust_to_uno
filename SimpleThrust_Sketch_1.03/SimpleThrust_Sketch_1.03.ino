/*
  Sketch for use with SimpleThrust - Damian Cunningham, 2018/2019
  
  Developed on Arduino Mega 2560 Platform and can be modified to work with an UNO

  To customize, look for the block of code highlighted "ENTER YOUR CUSTOM SETUP HERE"

  Note:  The SimpleThrust software is oblivious to the microcontroller being used to send signals through the serial, or COM, port.
         It simply requires the microcontroller to send the correct Message in terms of length and format.
         Therefore feel free to modify the code to suit your specific needs understanding the the final Message sent to the 
         serial port must match the expected length and format.  
         The message structure is as follows:
  
    1 x Session timestamp (Position 0 in Message)
    1 x Interrupt timestamp and 1 x time interval since last interrupt for Pin2   which is the primary RPM input (WORKS FOR UNO AND MEGA) (Positions 1 and 2 in Message)
    1 x Interrupt timestamp and 1 x time interval since last interrupt for Pin3   which is the ESC RPM input     (WORKS FOR UNO AND MEGA) (Positions 3 and 4 in Message)
    1 x Interrupt timestamp and 1 x time interval since last interrupt for Pin 18 which is the Fan based Anemometer RPM input (MEGA ONLY) (Positions 5 and 6 in Message)
    
    Analog Inputs 
        Cell voltages 1 thru 12.                                                  (Positions 7 thru 18 in Message)
        Current output from hall effect current sensor                            (Position 19 in Message)
        Thrust output from load cell and amplifier                                (Position 20 in Message)
        Torque output from load cell(s) and amplifier. Two load cells can be used (Position 21 in Message)
        Motor, ESC, Battery, and Ambient temperatures                             (Positions 22 thru 25 in Message)
        Pitot and Hot Wire airspeed indication                                    (Positions 26 and 27 in Message)
        Throttle position                                                         (Position 28 in Message)
        */

#include <Servo.h> //Using the servo library to control the ESC
//The following two lines are needed for remote temp sensing using the MLX90614 IR Temp Sensor. This functionality is DISABLED in this sketch
#include <Wire.h> // I2C library, required for MLX90614 - Needed for thermal sensing
#include <SparkFunMLX90614.h> // SparkFunMLX90614 Arduino library - Needed for IR thermal sensing

  Servo myESC;  // create servo object to control the ESC
  IRTherm myMotorTherm; // Create an IRTherm object  - Needed if the MLX90614 IR temp sensor is used

  int Cell_Pins[13];       // Array to hold the analog pin numbers from which to read each cell voltage
  int Current_Pin;         // Analog pin from which to read the current signal
  
  int Thrust_Uses_HX711;    // Flag to determine if thrust is analog or digital via HX711
  int Thrust_Analog_Pin;          // Analog pin from which to read the thrust signal
  int Thrust_HX711_Clock_Pin;  // General purpose pin for HX711 clock if used
  int Thrust_HX711_Data_Pin;   // General purpose pin for HX711 data if used
  
  int Torque_Uses_HX711;   // Flag to determine if torque is analog or digital via HX711
  int Torque_Analog_Pin_1;        // First of two possible analog pin inputs for torque
  int Torque_Analog_Pin_2;        // Second input for torque
  int Torque_HX711_Clock_Pin_1;   //General purpose pins for HX711 clock and data for up to two torque load cells
  int Torque_HX711_Data_Pin_1;
  int Torque_HX711_Clock_Pin_2;
  int Torque_HX711_Data_Pin_2;
  
  
  int Motor_Temp_Uses_MLX90614;  // Contactless sensor, can report both ambient and object (Motor) temperatures
  int Motor_Temp_Pin;           // If not using MLX90614 contactless sensor then assumed that analog sensor being used
   
  
  
  int ESC_Temp_Pin;  // Three additional temperatures can be monitored (motor, ESC, battery, and ambient air). These should be analog sensors
  int Battery_Temp_Pin;
  int Ambient_Temp_Pin;
  
  int Pitot_Pin;           // Analog pin to read signal from a Pitot airspeed sensor
  int Hot_Wire_Pin;        // Analog pin to read signal from a hot wire airspeed sensor
  int Throttle_Pin;        // Analog pin to read a potentiometer voltage divider to control the ESC
  
  int ESC_Control_Pin;     // Digital output pin to control ESC

  int RPM_Pin;        // Interrupt pin for primary RPM measurement
  int RPM_ESC_Pin;    // Interrupt pin for secondary RPM measurement using one of the ESC motor wires
  int RPM_Fan_Pin;    // Interrupt pin for RPM measurement of a fan based anemomoeter

  int BaudRate;           // The rate at which data is sent through the serial port to the computer
                     
  volatile unsigned long RPM_Interval = 0;
  volatile unsigned long RPM_ESC_Interval = 0;
  volatile unsigned long RPM_Fan_Interval = 0;
  
  volatile unsigned long RPM_Old_Timestamp = 0;
  volatile unsigned long RPM_ESC_Old_Timestamp = 0;
  volatile unsigned long RPM_Fan_Old_Timestamp = 0;
    
  volatile unsigned long RPM_New_Timestamp = 0;
  volatile unsigned long RPM_ESC_New_Timestamp = 0;
  volatile unsigned long RPM_Fan_New_Timestamp = 0;
  
  String Message = "";
  String inString = "";

void setup() {
// *******************************
// ENTER YOUR CUSTOM SETUP HERE
// *******************************  
// In the following code, assign the analog pin number to each of the 12 battery cells.
// Use the analog pin names (A0, A1, A2, etc)
// If you are not going to use a specific cell, set the value to -1 
// In this version, we are only using 6 cells  on pins A0 to A5 so the remaining Cell_Pins are set to -1

//Serial.begin(9600);


//not using any battery cells
  Cell_Pins[1] =  -1;
  Cell_Pins[2] =  -1;
  Cell_Pins[3] =  -1;
  Cell_Pins[4] =  -1;
  Cell_Pins[5] =  -1;
  Cell_Pins[6] =  -1;
  Cell_Pins[7] =  -1; // A6;
  Cell_Pins[8] =  -1; // A7;
  Cell_Pins[9] =  -1; // A8;
  Cell_Pins[10] = -1; // A9;
  Cell_Pins[11] = -1; // A10;
  Cell_Pins[12] = -1; // A11;

// Assign the analog pin numbers to each of the measurements below. 
// Use the analog pin names (A0, A1, A2, etc.)
// For Thrust, set the flag for Thrust_Uses_HX711 to 1 if you are using an HX711 chip or 0 if not
// Then set the data and clock pins for the HX711, and the Thrust_Analog_Pin to -1
// If you are not using an HX711, set the data and clock pins to -1
// For Torque, you may use inputs from 1 or 2 load cells.  
// Use the same approach as Thrust if using an HX711
// If you are not using a specific measurement, set the pin value to -1

  Current_Pin = -1; //A6;  
  Thrust_Uses_HX711 = 1;
  Thrust_Analog_Pin = -1;  
  Thrust_HX711_Clock_Pin = 5;    //POF: just integer, or DX?
  Thrust_HX711_Data_Pin = 4;
    
  Torque_Uses_HX711 =-1;   
  Torque_Analog_Pin_1 = -1;    
  Torque_Analog_Pin_2 = -1;       
  Torque_HX711_Clock_Pin_1 = -1;    
  Torque_HX711_Data_Pin_1 = -1;
  Torque_HX711_Clock_Pin_2 = -1;
  Torque_HX711_Data_Pin_2 = -1;

  //If you are using MLX90614 then both motor and ambient temps can be acquired, set the usage flag to 1 and the pins for motor and ambient to -1
  Motor_Temp_Uses_MLX90614 = -1;
  Motor_Temp_Pin = -1; 
  Ambient_Temp_Pin = -1;
    
  ESC_Temp_Pin = -1;
  Battery_Temp_Pin = -1;
  
  Pitot_Pin = -1; //A8;     
  Hot_Wire_Pin = -1;
  
  Throttle_Pin = -1; //A13;  

// Assign the interrupt pins for the three available RPM measurements.  
// If you are not going to use a specific RPM signal set the value to -1
// Use standard pin numbers (2, 3, etc)
  RPM_Pin = -1; //2;      // For UNO and MEGA boards
  RPM_ESC_Pin = -1; // Would typically be 3 for UNO and MEGA boards
  RPM_Fan_Pin = -1; // Would typically be 18 for MEGA boards.  A third interrupt is not available on UNO

// Assign the ESC control pin.  This is the pin to which the signal wire from your ESC servo connector is connected
  ESC_Control_Pin = -1; //7;

// Set the Baud rate for communication over the serial port.
// Ensure to select the matching Baud rate in SimpleThrust when initiating the serial communications through the COM port
// Availailable Baud rates in SimpleThrust are: 9600, 14400, 19200, 28800, 38400, 57600, or 115200

  BaudRate = 9600;

// *******************************
// END OF CUSTOM SETUP
// ******************************* 

  //analogReference(EXTERNAL); DC03MAR19 - Removed use of the external reference
  
  // Setting the appropriate analog pins as inputs.
  for (int PinCount = 1; PinCount < 13 ; PinCount++){
    if (Cell_Pins[PinCount] != -1){
      pinMode(Cell_Pins[PinCount], INPUT);
    }
  }
  if (Current_Pin != -1)      {pinMode(Current_Pin,INPUT);}
  
  if (Thrust_Analog_Pin != -1){pinMode(Thrust_Analog_Pin,INPUT);}
  if (Thrust_Uses_HX711 == 1){
    if(Thrust_HX711_Data_Pin != -1)  {pinMode(Thrust_HX711_Data_Pin,INPUT);}
    if(Thrust_HX711_Clock_Pin != -1) {pinMode(Thrust_HX711_Clock_Pin, OUTPUT);}
  }
  
  if (Torque_Analog_Pin_1 != -1)     {pinMode(Torque_Analog_Pin_1,INPUT);}
  if (Torque_Analog_Pin_2 != -1)     {pinMode(Torque_Analog_Pin_2,INPUT);}
  if (Torque_Uses_HX711 == 1){
    if(Torque_HX711_Data_Pin_1 != -1)  {pinMode(Torque_HX711_Data_Pin_1,INPUT);}
    if(Torque_HX711_Clock_Pin_1 != -1) {pinMode(Torque_HX711_Clock_Pin_1, OUTPUT);}
    if(Torque_HX711_Data_Pin_2 != -1)  {pinMode(Torque_HX711_Data_Pin_2,INPUT);}
    if(Torque_HX711_Clock_Pin_2 != -1) {pinMode(Torque_HX711_Clock_Pin_2, OUTPUT);}
  }
 
  
  
  if (Motor_Temp_Pin != -1)   {pinMode(Motor_Temp_Pin,INPUT);}
  if (Motor_Temp_Uses_MLX90614 != -1){
     myMotorTherm.begin(); // Initialize thermal IR sensor
     myMotorTherm.setUnit(TEMP_C); // Set the library's units to Celcius for SimpleThrust
  }
  
  if (ESC_Temp_Pin != -1)     {pinMode(ESC_Temp_Pin,INPUT);}
  if (Battery_Temp_Pin != -1) {pinMode(Battery_Temp_Pin,INPUT);}
  if (Ambient_Temp_Pin != -1) {pinMode(Ambient_Temp_Pin,INPUT);}
  if (Pitot_Pin != -1)        {pinMode(Pitot_Pin,INPUT);}
  if (Hot_Wire_Pin != -1)     {pinMode(Hot_Wire_Pin,INPUT);}
  if (Throttle_Pin != -1)     {pinMode(Throttle_Pin,INPUT);}
 
  myESC.attach(ESC_Control_Pin);  //Sets the pin used for ESC control as output for PWM
  myESC.write (0);        //Sends a 0 signal to the ESC to try to avoid motor starting accidentally

  // Initialize interupts by assigning to interrupt handling routines
  // You can experiment with RISING or FALLING to determine which trigger mode works best for the RPM sensor being used
  if (RPM_Pin != -1)    {attachInterrupt(digitalPinToInterrupt(RPM_Pin),RPM_Interrupt_Single,FALLING);}     
  if (RPM_ESC_Pin != -1){attachInterrupt(digitalPinToInterrupt(RPM_ESC_Pin),RPM_ESC_Interrupt_Single,FALLING);} 
  if (RPM_Fan_Pin != -1){attachInterrupt(digitalPinToInterrupt(RPM_Fan_Pin),RPM_Fan_Interrupt_Single,FALLING);}

  // Initialize serial communication
    Serial.begin(BaudRate);
}

void loop() {
  Message = "";                      // Clear the string in preparation for adding new data 
  Message += micros();               // Timestamp for message (will be a few microseconds early)
  Message += ",";                    // Field separator
  Message += RPM_New_Timestamp;      // Time at which latest RPM signal was detected
  Message += ",";
  Message += RPM_Interval;           // Time between RPM signals
  Message += ",";        
  Message += RPM_ESC_New_Timestamp;  // Time at which latest RPM_ESC signal was detected
  Message += ",";
  Message += RPM_ESC_Interval;       // Time between RPM_ESC signals
  Message += ",";
  Message += RPM_Fan_New_Timestamp;  // Time at which latest RPM_Fan signal was detected
  Message += ",";
  Message += RPM_Fan_Interval;       // Time between RPM_Fan signals, 7th digit
  Message += ",";    
 
  // Read the analog inputs and add to the string
  // First the 12 cells...
  /*
  for (int CellCount = 1; CellCount < 13; CellCount++){
    if (Cell_Pins[CellCount] != -1){
      Message += analogRead(Cell_Pins[CellCount]);

      Message += ",";
    }
    else{
      Message += 0;
      Message += ",";
    }
  }
  */

 for(int i=0; i<12; i++){
  Message += 0;
  Message += ",";
 }

 //19th digit that was input
  //... then the remaining values
 
  if (Current_Pin != -1)     {Message += analogRead(Current_Pin);     Message += ",";} else {Message += 0; Message += ",";}
  
  if (Thrust_Uses_HX711 == 1){
    if((Thrust_HX711_Clock_Pin != -1) && (Thrust_HX711_Data_Pin != -1)){ // In case HX711 use was flagged as true but data and clock pins were not set
       Message += GetThrustValue(Thrust_HX711_Clock_Pin,Thrust_HX711_Data_Pin);
       Message += ",";
    }
    else{
    Message += 0; Message += ",";
  }
  }
  else{
  if (Thrust_Analog_Pin != -1)      {Message += analogRead(Thrust_Analog_Pin);      Message += ",";} else {Message += 0; Message += ",";}
  }

  if (Torque_Uses_HX711 == 1){
     int TempTorqueRead = 0;
     if((Torque_HX711_Clock_Pin_1 != -1) && (Torque_HX711_Data_Pin_1 != -1)){ // In case HX711 use was flagged as true but data and clock pins were not set
       TempTorqueRead += GetThrustValue(Torque_HX711_Clock_Pin_1,Torque_HX711_Data_Pin_1);
     }
     if((Torque_HX711_Clock_Pin_2 != -1) && (Torque_HX711_Data_Pin_2 != -1)){ // In case HX711 use was flagged as true but data and clock pins were not set
       TempTorqueRead += GetThrustValue(Torque_HX711_Clock_Pin_2,Torque_HX711_Data_Pin_2);
     }
     Message += TempTorqueRead; Message += ",";
  }
  else{
    int TempTorqueRead = 0;  
    if (Torque_Analog_Pin_1 != -1)    {TempTorqueRead = analogRead(Torque_Analog_Pin_1);} else {TempTorqueRead = 0;}
    if (Torque_Analog_Pin_2 != -1)    {Message += (TempTorqueRead + analogRead(Torque_Analog_Pin_2)); Message += ",";} else {Message += TempTorqueRead; Message += ",";}
  }

  if (Motor_Temp_Uses_MLX90614 == 1){
   if (myMotorTherm.read()) // On success, read() will return 1, on fail 0.
  {
    // Use the object() and ambient() functions to grab the object and ambient temperature
    // They'll be floats, calculated out to the unit you set with setUnit().
    //Serial.print("Object: " + String(myMotorTherm.object(), 2));
    //.print("Ambient: " + String(myMotorTherm.ambient(), 2));
    //Serial.println();
    Message += String(myMotorTherm.object(), 2); Message += ",";
    Message += String(myMotorTherm.ambient(), 2);  Message += ",";
  }
  else
  { Message += 0; Message += ",";
    Message += 0;  Message += ",";
  }
  }
  else{
  if (Motor_Temp_Pin != -1)  {Message += analogRead(Motor_Temp_Pin);  Message += ",";} else {Message += 0; Message += ",";}
  if (Ambient_Temp_Pin != -1){Message += analogRead(Ambient_Temp_Pin);Message += ",";} else {Message += 0; Message += ",";}
  }
  
  if (ESC_Temp_Pin != -1)    {Message += analogRead(ESC_Temp_Pin);    Message += ",";} else {Message += 0; Message += ",";}
  if (Battery_Temp_Pin != -1){Message += analogRead(Battery_Temp_Pin);Message += ",";} else {Message += 0; Message += ",";}
  if (Pitot_Pin != -1)       {Message += analogRead(Pitot_Pin);       Message += ",";} else {Message += 0; Message += ",";}
  if (Hot_Wire_Pin != -1)    {Message += analogRead(Hot_Wire_Pin);    Message += ",";} else {Message += 0; Message += ",";}
  if (Throttle_Pin != -1)    {Message += analogRead(Throttle_Pin);                   } else {Message += 0;}

  // Send the message
  Serial.println (Message);
  // Clear the serial buffer
  Serial.flush();
  // Wait a bit...
  delay(1);
}

//Capture messages sent through the serial port - used for throttle position and output to ESC
void serialEvent(){
   while (Serial.available() > 0) {
    int inChar = Serial.read();
    inString += (char)inChar;
    // If the char is a newline, send the new value to the ESC
    if (inChar == '\n') {
      myESC.write(map(inString.toInt(), 0, 100, 45, 135));  // Maps the received 0-100% throttle value to the appropriate "servo" signal for the ESC 
      // Note:  45 to 135 represents a range of 500 to 1500 ms for the ESC.  You can adjust these ranges to provide a dead band at either end.
      inString = "";
    }
  }
}

//Interrupt routine for primary RPM - Single sends only the most recent RPM detected
void RPM_Interrupt_Single(){
  RPM_New_Timestamp = micros();                             // Record time that the RPM event occurred
  RPM_Interval = RPM_New_Timestamp - RPM_Old_Timestamp; // Record the time between this event and the last event
  RPM_Old_Timestamp = RPM_New_Timestamp;                  // Remember the time for the next event
}

//Interrupt routine for ESC RPM - Single sends only the most recent RPM detected
void RPM_ESC_Interrupt_Single(){
   RPM_ESC_New_Timestamp = micros();
   RPM_ESC_Interval = RPM_ESC_New_Timestamp - RPM_ESC_Old_Timestamp;
   RPM_ESC_Old_Timestamp = RPM_ESC_New_Timestamp;
}

//Interrupt routine for Fan based anemometer RPM - Single sends only the most recent RPM detected
void RPM_Fan_Interrupt_Single(){
   RPM_Fan_New_Timestamp = micros();
   RPM_Fan_Interval = RPM_Fan_New_Timestamp - RPM_Fan_Old_Timestamp;
   RPM_Fan_Old_Timestamp = RPM_Fan_New_Timestamp;
}

//Interrupt routine for RPM1 - Multiple tracks all RPM events between calls to serial port
//void RPM1_Interrupt_Multiple(){
//  TempTime1 = micros();       // Record time that the RPM1 event occurred
//  Time1 = TempTime1-Oldtime1; // Record the time between this event and the last event
//  Oldtime1 = TempTime1;       // Remember the time for the next event
//}

//Interrupt routine for RPM2 - Multiple tracks all RPM events between calls to serial port
//void RPM2_Interrupt_Multiple(){
//    TempTime2 = micros();
//    Time2 = TempTime2-Oldtime2;
//    Oldtime2 = TempTime2;
//}

unsigned long GetThrustValue(int Sent_Clock_Pin, int Sent_Data_Pin)
{
  byte data[3];
  while (digitalRead(Sent_Data_Pin));
  for (byte j = 3; j--;)
  {
    for (char i = 8; i--;)
    {
      digitalWrite(Sent_Clock_Pin, HIGH);
      bitWrite(data[j], i, digitalRead(Sent_Data_Pin));
      digitalWrite(Sent_Clock_Pin, LOW);
    }
  }

  digitalWrite(Sent_Clock_Pin, HIGH);
  digitalWrite(Sent_Clock_Pin, LOW);

  data[2] ^= 0x80;
  /*
  return ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
      | (uint32_t) data[0];
  */
   return (((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8) | (uint32_t) data[0]) - 8470300;
}
