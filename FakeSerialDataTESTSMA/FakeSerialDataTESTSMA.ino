#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_WIRE1.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)




int8_t rightTemp;
int8_t leftTemp;
int8_t smaStatus;

unsigned long left_last_send_time;
unsigned long right_last_send_time;

unsigned long startTime;
unsigned long endTime;
unsigned long duration = 10; // 10 ms

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool SMAflag = false;
bool SMAstopflag = false;
int smaPin = 3;
int sma_ms_open_full = 1800;  //number of milliseconds to send max power to sma (max 1000)
int sma_ms_open_partial = 900;
int current_activation = 0;
int smaMS_TOTAL = 20000; //number of milliseconds total
int maxPower = 200; //out of 255
int maintainPower = 20; //out of 255

//toggle these true or false depending what you want to test
bool LEFT = true;
bool RIGHT = true;
bool SMA = true;
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  
  Serial.begin(115200); //Serial COM baud rate
   
  Serial.println("Orientation Sensor Set-up"); Serial.println("");
  digitalWrite(2, LOW);
  delay(1000);
  digitalWrite(2, HIGH);
  delay(1000);
  inputString.reserve(200);

  /* Initialise the sensor */
  if(LEFT){
    Serial.print("Fake-Connected Left BNO055");
  } /* Initialise the sensor */
  if(RIGHT){
    Serial.print("Fake-Connected Right BNO055");
  }
  delay(200);
  
  
  if(SMA){
    smaStatus = false;
    pinMode(smaPin, OUTPUT);
    digitalWrite(smaPin, LOW);
  }
  Serial.println("Ready");
  
  left_last_send_time = millis();
  right_last_send_time = millis();
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/

// Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
void loop(void)
{
  //startTime = millis();

  if(LEFT){
    
  }

  if(RIGHT){

  }

  if(LEFT){
  }

  if(RIGHT){

  }

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  SMAstopflag = false;

  if(SMA){
    if (stringComplete) {
      smaStatus = inputString.toInt();
      Serial.print("Received data:");
      Serial.println(inputString);  
      // clear the string:
      inputString = "";
      stringComplete = false;
      if(smaStatus == 1)
      { 
        Serial.println("TRYING TO ACTIVATE SMA");
        if (current_activation == 0){
          Serial.println("SETTING FLAG TO ACTIVATE");
          SMAflag = true;
          current_activation = sma_ms_open_full;
        }
        else
        {
          Serial.println("Not taking additional activation, because gripper is already active.");
        }
      }
      if(smaStatus == 2)
      {
        Serial.println("TRYING TO ACTIVATE SMA222");
        if (current_activation == 0){
          Serial.println("SETTING FLAG TO ACTIVATE222");
          SMAflag = true;
          current_activation = sma_ms_open_partial;
        }
        else
        {
          Serial.println("Not taking additional activation, because gripper is already active.");
        }
      }
      if(smaStatus == 0)
      {
        Serial.println("SETTING DEACTIVATION CHART!!!");
        SMAstopflag = true;
      }
    }

    if(SMAflag)
    {
      Serial.println("SETTING SMAPIN TO MAX POWER");
      startTime = millis();
      analogWrite(smaPin, maxPower);
      //Serial.println("SMA started");
      SMAflag = false;
    }
    endTime = millis();
    if(current_activation > 0 && (endTime-startTime)>=current_activation)
    {
      Serial.println("SETTING SMAPING TO MAINTAIN POWER");
      analogWrite(smaPin, maintainPower);
      //Serial.println("maintaining SMA");
    } 
    if(current_activation > 0 && ((endTime-startTime)>=smaMS_TOTAL || SMAstopflag == true))
    {
      Serial.println("SETTING SMAPIN TO OFF!!!!!!!");
      Serial.print(current_activation);
      Serial.print("\t");
      Serial.print(endTime-startTime);
      Serial.print("\t");
      Serial.print(smaMS_TOTAL);
      Serial.print("\t");
      Serial.println(SMAstopflag);
      digitalWrite(smaPin, LOW);
      //Serial.print("SMA ended. Duration: ");
      //Serial.println(endTime-startTime);
      SMAflag = false;
      if (SMAstopflag == true)
      {
        SMAstopflag = false;
      }
      current_activation = 0;
      smaStatus = 0;
    }
  }
}
