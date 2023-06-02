/* Telemetry Project:
   Renato and Michael.*/

#include <ArduinoJson.h>
#include <Wire.h>
#include <Zumo32U4.h>

#define JASON_SIZE 128
#define BUFFER_SIZE 200
#define PERIOD_INIT 500
#define ENABLE_INIT true
#define LOOP_DELAY_MS 10
#define INTERNAL_DATA_PERIOD 2000
#define BITRATE 115200

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

// Sampling period in millisec
uint32_t BuzzerSamplePeriod = PERIOD_INIT;
uint32_t IMUSamplePeriod = PERIOD_INIT;
uint32_t EncodersSamplePeriod = PERIOD_INIT;
uint32_t LineSensorsSamplePeriod = PERIOD_INIT;
uint32_t ProximitySensorsSamplePeriod = PERIOD_INIT;

// Enable reading from sensors
bool BuzzerEnableRead = ENABLE_INIT;
bool IMUEnableRead = ENABLE_INIT;
bool EncodersEnableRead = ENABLE_INIT;
bool LineSensorsEnableRead = ENABLE_INIT;
bool ProximitySensorsEnableRead = ENABLE_INIT;

Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;

bool imu_init_error = false;

void setup()       // run once, when the sketch starts
{
  // initialize serial:
  Serial.begin(BITRATE);
  Wire.begin();  
  // reserve 200 bytes for the inputString:
  inputString.reserve(BUFFER_SIZE);
  
  // imu initialization
  if (!imu.init())
  {    
    imu_init_error = true;
  }
  imu.enableDefault();

  // Line Sensors initialization  
  lineSensors.initThreeSensors();

  // Proximity Sensors initialization
  proxSensors.initThreeSensors();

  // Initialize speed
  motors.setSpeeds(0, 0);
}

void loop()        // run over and over again
{
    getSerial();
    parseInput();
    if(BuzzerEnableRead) readBuzzer();
    if(IMUEnableRead) readIMU();
    if(EncodersEnableRead) readEncoders();
    if(LineSensorsEnableRead) readLineSensors();
    if(ProximitySensorsEnableRead) readProximitySensors();
    sendInternalData();
    delay(LOOP_DELAY_MS);
}

// put message from serial into string
void getSerial()
{
  // Parse incoming string
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    if (inChar == '\n') 
    {
      stringComplete = true;
      inputString.replace("\r\n","");
      break;
    }
  }
  return;
}

// parse input from serial and execute command accordingly
void parseInput()
{
  // No message yet
  if (stringComplete == false) return;

  StaticJsonDocument<JASON_SIZE> JsonInput;

  DeserializationError error = deserializeJson(JsonInput, inputString);
  if (error)
  {
    Serial.print("Arduino: Got something that was not Json type from serial: ");
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
    return;
  }

  // Process incoming message
  // Buzzer
  if (JsonInput.containsKey("period_buzzer")) 
  {
    BuzzerSamplePeriod = JsonInput["period_buzzer"];
  } 
  else if (JsonInput.containsKey("enable_read_buzzer"))
  {
    BuzzerEnableRead = JsonInput["enable_read_buzzer"];
  }
  else if (JsonInput.containsKey("play_buzzer"))
  {
    if (JsonInput["play_buzzer"])
    {
      buzzer.playFrequency(440, 5000, 15);
    }
    else
    {
      buzzer.stopPlaying();
    }
  }
  // IMU
  else if (JsonInput.containsKey("period_imu"))
  {
    IMUSamplePeriod = JsonInput["period_imu"];
  } 
  else if (JsonInput.containsKey("enable_read_imu"))
  {
    IMUEnableRead = JsonInput["enable_read_imu"];
  }
  // Encoders
  else if (JsonInput.containsKey("period_encoders"))
  {
    EncodersSamplePeriod = JsonInput["period_encoders"];
  } 
  else if (JsonInput.containsKey("enable_read_encoders"))
  {
    EncodersEnableRead = JsonInput["enable_read_encoders"];
  }
  // Line Sensors
  else if (JsonInput.containsKey("period_line_sensors"))
  {
    LineSensorsSamplePeriod = JsonInput["period_line_sensors"];
  } 
  else if (JsonInput.containsKey("enable_read_line_sensors"))
  {
    LineSensorsEnableRead = JsonInput["enable_read_line_sensors"];
  }
  // Proximity Sensors
  else if (JsonInput.containsKey("period_proximity_sensors"))
  {
    ProximitySensorsSamplePeriod = JsonInput["period_proximity_sensors"];
  } 
  else if (JsonInput.containsKey("enable_read_proximity_sensors"))
  {
    ProximitySensorsEnableRead = JsonInput["enable_read_proximity_sensors"];
  }
  else if (JsonInput.containsKey("motors"))
  {
    motors.setSpeeds(JsonInput["motors"][0], JsonInput["motors"][1]);
  }
  else
  {
    Serial.print("Arduino: recieved a not valid dictionary from Pi: ");
    Serial.println(inputString);
  }
  // clear the string:
  inputString = "";
  stringComplete = false;
}


void readBuzzer()
{
  static uint32_t BuzzerlastSampleTime = 0;
  
  // Enter here if enough time has passed
  if ((uint32_t)(millis() - BuzzerlastSampleTime) >= BuzzerSamplePeriod)
  {
    BuzzerlastSampleTime = millis();
    // Create JSON message
    StaticJsonDocument<JASON_SIZE> JsonOutput;
    JsonOutput["sensor"] = "buzzer";
    JsonOutput["is_playing"] = buzzer.isPlaying();
    serializeJson(JsonOutput, Serial);
    Serial.println();
  }
}


void readIMU()
{
  static uint32_t IMUlastSampleTime = 0;
  
  // Enter here if enough time has passed
  if ((uint32_t)(millis() - IMUlastSampleTime) >= IMUSamplePeriod)
  {
    IMUlastSampleTime = millis();

    // Create JSON message
    StaticJsonDocument<JASON_SIZE> JsonOutput;
    JsonOutput["sensor"] = "imu";
    JsonOutput["error_init"] = 0;
    JsonOutput["error_read"] = 0;

    if(imu_init_error)
    {
      JsonOutput["error_init"] = 1;
      serializeJson(JsonOutput, Serial);
      Serial.println();
      return;
    }

    imu.read();

    if (imu.getLastError() != 0)
    {
      JsonOutput["error_read"] = 1;
      serializeJson(JsonOutput, Serial);
      Serial.println();
      return;
    }

    JsonArray accelerometer = JsonOutput.createNestedArray("accelerometer");
    accelerometer.add(imu.a.x);
    accelerometer.add(imu.a.y);
    accelerometer.add(imu.a.z);

    JsonArray magnetometer = JsonOutput.createNestedArray("magnetometer");
    magnetometer.add(imu.m.x);
    magnetometer.add(imu.m.y);
    magnetometer.add(imu.m.z);

    JsonArray gyro = JsonOutput.createNestedArray("gyro");
    gyro.add(imu.g.x);
    gyro.add(imu.g.y);
    gyro.add(imu.g.z);
    
    serializeJson(JsonOutput, Serial);
    Serial.println();
  }
}


void readEncoders()
{
  static uint32_t EncoderslastSampleTime = 0;
  
  // Enter here if enough time has passed
  if ((uint32_t)(millis() - EncoderslastSampleTime) >= EncodersSamplePeriod)
  {
    EncoderslastSampleTime = millis();

    // Create JSON message
    StaticJsonDocument<JASON_SIZE> JsonOutput;
    JsonOutput["sensor"] = "encoders";
    JsonOutput["error_right"] = 0;
    JsonOutput["right"] = -1;
    JsonOutput["error_left"] = 0;
    JsonOutput["left"] = -1;

    int countsRight = encoders.getCountsAndResetRight();
    int countsLeft = encoders.getCountsAndResetLeft();

    if (encoders.checkErrorRight())
    {
      JsonOutput["error_right"] = 1;
    }
    else
    {
      JsonOutput["right"] = countsRight;
    }
      
    if (encoders.checkErrorLeft())
    {
      JsonOutput["error_left"] = 1;
    }
    else
    {
      JsonOutput["left"] = countsLeft;
    }  

    serializeJson(JsonOutput, Serial);
    Serial.println();
  }
}


void readLineSensors()
{
  static uint32_t LineSensorslastSampleTime = 0;
  
  // Enter here if enough time has passed
  if ((uint32_t)(millis() - LineSensorslastSampleTime) >= LineSensorsSamplePeriod)
  {
    LineSensorslastSampleTime = millis();

    // Create JSON message
    StaticJsonDocument<JASON_SIZE> JsonOutput;
    JsonOutput["sensor"] = "line_sensors";

    uint16_t lineSensorValues[3] = { 0, 0, 0 };
    lineSensors.read(lineSensorValues);
  
    JsonArray data = JsonOutput.createNestedArray("data");
    data.add(lineSensorValues[0]);
    data.add(lineSensorValues[1]);
    data.add(lineSensorValues[2]);
    
    serializeJson(JsonOutput, Serial);
    Serial.println();
  }
}

void readProximitySensors()
{
  static uint32_t ProximitySensorslastSampleTime = 0;
  
  // Enter here if enough time has passed
  if ((uint32_t)(millis() - ProximitySensorslastSampleTime) >= ProximitySensorsSamplePeriod)
  {
    ProximitySensorslastSampleTime = millis();

    // Create JSON message
    StaticJsonDocument<JASON_SIZE> JsonOutput;
    JsonOutput["sensor"] = "proximity_sensors";

    proxSensors.read();
    // countsXWithYLeds - Y is Tx and X is Rx
    // readBasicX - Reads with sensor X (Rx) without LEDS. (Basic)
    JsonArray left_sensor = JsonOutput.createNestedArray("left_sensor");
    left_sensor.add(proxSensors.countsLeftWithLeftLeds());
    left_sensor.add(proxSensors.countsLeftWithRightLeds());

    JsonArray front_sensor = JsonOutput.createNestedArray("front_sensor");
    front_sensor.add(proxSensors.countsFrontWithLeftLeds());
    front_sensor.add(proxSensors.countsFrontWithRightLeds());

    JsonArray right_sensor = JsonOutput.createNestedArray("right_sensor");
    right_sensor.add(proxSensors.countsRightWithLeftLeds());
    right_sensor.add(proxSensors.countsRightWithRightLeds());

    JsonArray read_basic = JsonOutput.createNestedArray("read_basic");
    read_basic.add(proxSensors.readBasicLeft());
    read_basic.add(proxSensors.readBasicFront());
    read_basic.add(proxSensors.readBasicRight());
    
    serializeJson(JsonOutput, Serial);
    Serial.println();  
  }
}


void sendInternalData()
{
  static uint32_t InternalDatalastSampleTime = 0;
  
  // Enter here if enough time has passed
  if ((uint32_t)(millis() - InternalDatalastSampleTime) >= INTERNAL_DATA_PERIOD)
  {
    InternalDatalastSampleTime = millis();
    // Create JSON message
    StaticJsonDocument<JASON_SIZE> JsonOutput;
    JsonOutput["sensor"] = "zumo";

    JsonArray buzzer = JsonOutput.createNestedArray("buzzer");
    buzzer.add(BuzzerSamplePeriod);
    buzzer.add((int)BuzzerEnableRead);

    JsonArray imu = JsonOutput.createNestedArray("imu");
    imu.add(IMUSamplePeriod);
    imu.add((int)IMUEnableRead);

    JsonArray encoders = JsonOutput.createNestedArray("encoders");
    encoders.add(EncodersSamplePeriod);
    encoders.add((int)EncodersEnableRead);

    JsonArray line_sensors = JsonOutput.createNestedArray("line_sensors");
    line_sensors.add(LineSensorsSamplePeriod);
    line_sensors.add((int)LineSensorsEnableRead);

    JsonArray proximity_sensors = JsonOutput.createNestedArray("proximity_sensors");
    proximity_sensors.add(ProximitySensorsSamplePeriod);
    proximity_sensors.add((int)ProximitySensorsEnableRead);

    serializeJson(JsonOutput, Serial);
    Serial.println();
  }
}