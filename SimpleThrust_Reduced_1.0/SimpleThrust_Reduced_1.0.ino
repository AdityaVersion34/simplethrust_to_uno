/*
    A reduced version of the SimpleThrust Arduino Sketch with the following modifications:

    - The only obtained reading will be the Static thrust, obtained from a connected Load Cell (With a HX711_ADC chip)
    - A lot of other readings and other variables will be stripped away. Refer to the original SimpleThrust files at:
        https://forum.flitetest.com/index.php?threads/simplethrust-open-source-software-for-your-thrust-stand-project.56111/
    
    Source of this sketch: https://github.com/AdityaVersion34/simplethrust_to_uno
*/
/*
    Note copied from the original SimpleThrust sketch:

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

    I will preserve the above message format.
*/

#include <Wire.h>
#include <HX711_ADC.h>
#include <LiquidCrystal_I2C.h>

//stores the message that will be sent to the serial monitor
String Message = "";

HX711_ADC LoadCell(4,5);    //setting up load cell configuration - Data, Sck at D4, D5
LiquidCrystal_I2C lcd(0x3F, 16, 2); // LCD HEX address 0x3F
int taree = 6;              //Pin location of tare button. press to tare the load cell. At D6 pin
float currentThrust;    //will store the thrust value obtained from the arduino
float thrustToPrint;    //version of thrust to print to lcd

#define CALIBRATION_FACTOR 207.5        //a calibration factor used to obtain correct LCD display readings. SimpleThrust should
                                        //be able to calibrate from this value for the software.

int baudRate;   //the baud rate of transmission through serial port to the serial monitor

//arduino setup function
void setup(){

    pinMode(taree, INPUT_PULLUP);       //initialize the tare button pin
    LoadCell.begin();       //start connection to HX711
    LoadCell.start(1000);   //1000 milliseconds for load cell to stabilize
    LoadCell.setCalFactor(CALIBRATION_FACTOR);     //calibration value obtained from previous scale code on the Arduino. MAY NEED TO RECALIBRATE

    //taree = 6;              //tare button is at D6 pin        For some reason, when taree was assigned in setup(),
                                //load cell was continuously taring
    currentThrust = 0;  
    baudRate = 9600;

    //initialize serial communication
    Serial.begin(baudRate);

    //lcd display initialization
	lcd.init();
	lcd.backlight();             // turns on the backlight

	lcd.setCursor(1, 0);         // set cursor to first row
	lcd.print("Digital Scale "); // print out to LCD

	delay(3000);
	lcd.clear();
}

//arduino loop function
void loop(){
    Message = "";           //clearning the string
    Message += micros();    //0th value - message timestamp
    Message += ",";

    //adding 19 filler 0s - as per the message format
    for(uint8_t i=0; i<19; i++){
        Message += 0;
        Message += ",";
    }

    //20th position - the thrust value
    // updating load cell reading and obtaining reading from it
    LoadCell.update();
    currentThrust = LoadCell.getData();
  
    // negative thrust seems to be alright

    lcd.setCursor(1, 0);          // set cursor to first row
	lcd.print("Digital Scale ");  // print out to LCD
    if (currentThrust < 0)
	{
		thrustToPrint = currentThrust * (-1);
		lcd.setCursor(0, 1);
		lcd.print("-");
		lcd.setCursor(8, 1);
		lcd.print("-");
	}
	else
	{
		thrustToPrint = currentThrust;
        lcd.setCursor(0, 1);
		lcd.print(" ");
		lcd.setCursor(8, 1);
		lcd.print(" ");
	}

	lcd.setCursor(1, 1); // set cursor to secon row
	lcd.print(thrustToPrint, 1);     // print out the retrieved value to the second row
	lcd.print("g ");
	float z = thrustToPrint / 28.3495;
	lcd.setCursor(9, 1);
	lcd.print(z, 2);
	lcd.print("oz ");

    //appending to message
    Message += (currentThrust*CALIBRATION_FACTOR); // scaling by a factor before sending to software. not sure about calibration
    Message += ",";

    //adding 8 more filler 0s - as per the message format
    for(uint8_t i=0; i<7; i++){
        Message += 0;
        Message += ",";
    }

    Message += 0;

      // Send the message
    Serial.println(Message);

    // taring load cell
    if (digitalRead(taree) == LOW)
    {
        //Serial.println("   Taring...    ");
        lcd.setCursor(1, 0);
        lcd.print("    Taring...   ");
        LoadCell.start(1000);
    }

    // Clear the serial buffer
    Serial.flush();

    // Wait a bit...
    delay(1);
}