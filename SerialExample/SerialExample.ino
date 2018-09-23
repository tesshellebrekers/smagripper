#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055_WIRE1.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)M

Adafruit_BNO055 rightBNO = Adafruit_BNO055();
Adafruit_BNO055_WIRE1 leftBNO = Adafruit_BNO055_WIRE1();

int8_t rightTemp;
int8_t leftTemp;
int8_t smaStatus;

unsigned long startTime;
unsigned long endTime;
unsigned long duration = 10; // 10 ms

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
bool SMAflag = false;
int smaPin = 3;
int smaMS = 700;  //number of milliseconds to send max power to sma

//toggle these true or false depending what you want to test
bool LEFT = false;
bool RIGHT = false;
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
    if(!leftBNO.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no left BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    else
    {
      leftBNO.setExtCrystalUse(false);
    }
  } /* Initialise the sensor */
  if(RIGHT){
    if(!rightBNO.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no right BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    else
    {
      rightBNO.setExtCrystalUse(false); 
    }
  }
  delay(200);
  
  
  if(SMA){
    smaStatus = false;
    pinMode(smaPin, OUTPUT);
    digitalWrite(smaPin, LOW);
  }
  Serial.println("Ready");
  
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
    imu::Vector<3> leftACC = leftBNO.getVector(Adafruit_BNO055_WIRE1::VECTOR_ACCELEROMETER);
    imu::Vector<3> leftMAG = leftBNO.getVector(Adafruit_BNO055_WIRE1::VECTOR_MAGNETOMETER);
    imu::Vector<3> leftGYR = leftBNO.getVector(Adafruit_BNO055_WIRE1::VECTOR_GYROSCOPE);
    int8_t leftTemp = leftBNO.getTemp();
    
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
  Serial.print(analogRead(A9)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A8)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A7)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A6)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A5)); //integer
  Serial.print("\t");
  Serial.print(smaStatus); //integer
  Serial.println("\t");
  }

  if(RIGHT){

    imu::Vector<3> rightACC = rightBNO.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> rightMAG = rightBNO.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Vector<3> rightGYR = rightBNO.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    int8_t rightTemp = rightBNO.getTemp();
    
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
  Serial.print(analogRead(A4)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A3)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A2)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A1)); //integer
  Serial.print("\t");
  Serial.print(analogRead(A0)); //integer
  Serial.print("\t");
  Serial.print(smaStatus); //integer
  Serial.println("\t");
  delay(5);
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
      digitalWrite(smaPin, HIGH);
      Serial.println("SMA started");
      SMAflag = false;
    }
    endTime = millis();
    if((endTime-startTime)>=smaMS & smaStatus == 1)
    {
      digitalWrite(smaPin, LOW);
      Serial.print("SMA ended. Duration: ");
      Serial.println(endTime-startTime);
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
