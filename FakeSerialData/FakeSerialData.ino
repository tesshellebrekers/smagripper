#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_WIRE1.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)M

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
int smaPin = 3;
int smaMS = 1000;  //number of milliseconds to send max power to sma (max 1000)
int smaMS_TOTAL = 20000; //number of milliseconds total

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
  delay(100);
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
    imu::Vector<3> leftACC;
    leftACC[0] = 1.23;
    leftACC[1] = 0.0;
    leftACC[2] = 8.02;
    
    imu::Vector<3> leftMAG;
    leftMAG[0] = 3.45;
    leftMAG[1] = 3.67;
    leftMAG[2] = 3.89;
    
    imu::Vector<3> leftGYR;
    leftGYR[0] = 6.78;
    leftGYR[1] = 6.90;
    leftGYR[2] = 6.12;
    
    int8_t leftTemp = 30.02;
    
  Serial.print("L\t"); //identify which sensor skin this is 
   /* Display the floating point data for acceleration*/
  Serial.print(leftACC.x()); //float
  Serial.print("\t");
  Serial.print(leftACC.y()); //float
  Serial.print("\t");
  Serial.print(leftACC.z()); //float
  Serial.print("\t");

  /* Display the floating point data for magnetometer*/
  Serial.print(leftMAG.x()); //float
  Serial.print("\t");
  Serial.print(leftMAG.x()); //float
  Serial.print("\t");
  Serial.print(leftMAG.x()); //float
  Serial.print("\t");

  /* Display the floating point data for gyroscope*/
  Serial.print(leftGYR.x()); //float
  Serial.print("\t");
  Serial.print(leftGYR.y()); //float
  Serial.print("\t"); 
  Serial.print(leftGYR.z()); //float
  Serial.print("\t");

  /* Display the floating point data for temperature and analog values*/
  Serial.print(leftTemp); //float
  Serial.print("\t");
  Serial.print(456); //integer
  Serial.print("\t");
  Serial.print(722); //integer
  Serial.print("\t");
  Serial.print(811); //integer
  Serial.print("\t");
  Serial.print(355); //integer
  Serial.print("\t");
  Serial.print(711); //integer
  Serial.print("\t");
  Serial.print(smaStatus != 0); //bool
  Serial.print("\t");
  unsigned int current_time = millis(); //integer
  Serial.println(current_time - left_last_send_time);
  left_last_send_time = current_time;
  delay(10);
  }

  if(RIGHT){
    imu::Vector<3> rightACC;
    rightACC[0] = 0.0;
    rightACC[1] = 1.23;
    rightACC[2] = 8.02;
    
    imu::Vector<3> rightMAG;
    rightMAG[0] = 5.45;
    rightMAG[1] = 0.67;
    rightMAG[2] = 0.89;
    
    imu::Vector<3> rightGYR;
    rightGYR[0] = 4.78;
    rightGYR[1] = 3.90;
    rightGYR[2] = 2.12;
    
    int8_t rightTemp = 32.02;
    
  Serial.print("R\t"); //identify which sensor skin this is 
   /* Display the floating point data for acceleration*/
  Serial.print(rightACC.x()); //float
  Serial.print("\t");
  Serial.print(rightACC.y()); //float
  Serial.print("\t");
  Serial.print(rightACC.z()); //float
  Serial.print("\t");

  /* Display the floating point data for magnetometer*/
  Serial.print(rightMAG.x()); //float
  Serial.print("\t");
  Serial.print(rightMAG.x()); //float
  Serial.print("\t");
  Serial.print(rightMAG.x()); //float
  Serial.print("\t");

  /* Display the floating point data for gyroscope*/
  Serial.print(rightGYR.x()); //float
  Serial.print("\t");
  Serial.print(rightGYR.y()); //float
  Serial.print("\t"); 
  Serial.print(rightGYR.z()); //float
  Serial.print("\t");

  /* Display the floating point data for temperature and analog values*/
  Serial.print(rightTemp); //float
  Serial.print("\t");
  Serial.print(512); //integer
  Serial.print("\t");
  Serial.print(155); //integer
  Serial.print("\t");
  Serial.print(983); //integer
  Serial.print("\t");
  Serial.print(1002); //integer
  Serial.print("\t");
  Serial.print(677); //integer
  Serial.print("\t");
  Serial.print(smaStatus != 0); //bool
  Serial.print("\t");
  unsigned int current_time = millis();
  Serial.println(current_time - right_last_send_time);
  right_last_send_time = current_time; //integer
  delay(10);
  }

  if(SMA){
    if (stringComplete) {
      smaStatus = inputString.toInt();
      // clear the string:
      inputString = "";
      stringComplete = false;
      if(smaStatus == 1){SMAflag = true;}
    }

    if(SMAflag)
    {
      startTime = millis();
      analogWrite(smaPin, 120);
      SMAflag = false;
    }
    endTime = millis();
    if((endTime-startTime)>=smaMS & smaStatus == 1)
    {
      analogWrite(smaPin, 15);
    } 
    if((endTime-startTime)>=smaMS_TOTAL & smaStatus == 1)
    {
      digitalWrite(smaPin, LOW);
      SMAflag = false;
      smaStatus = 0;
    }
  }
  
  //delay(BNO055_SAMPLERATE_DELAY_MS);
  
}

void serialEvent() {
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
}
