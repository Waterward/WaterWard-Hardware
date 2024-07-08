// References
// LCD: https://randomnerdtutorials.com/esp32-esp8266-i2c-lcd-arduino-ide/
// Temperature: https://www.makerguides.com/ds18b20-arduino-tutorial/
// helpful about esp32: https://www.techtonions.com/esp32-pinout-simplified-no-more-confusion-when-choosing-gpios/
// NoceMCU Esp32s Datasheet: https://docs.ai-thinker.com/_media/esp32/docs/nodemcu-32s_product_specification.pdf

/*
 On Linux, Arduino IDE may error to upload due to usb not enough permissions, in this case
 use this command `sudo chmod a+rw /dev/ttyUSB0` but change the /dev/ value to what ever the IDE says
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Ultrasonic.h>

#include "OneWire.h"
#include "DallasTemperature.h"
#include "GravityTDS.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <FlowSensor.h>

#include "SECRET_VARS.h"

// -------------
// PINS
// -------------
#define ONE_WIRE_BUS 4
#define PIN_TURBIDITY_SENSOR 35
#define PIN_SWITCH_LCD 33  // see https://forum.arduino.cc/t/esp32-pins-that-support-pullup/1173356/4
#define PIN_US_TRIGGER 12
#define PIN_US_ECHO 14
#define PIN_TDS 26
#define PIN_WATERFLOW 25
#define PIN_PH 15
#define PIN_PUMP_RELAY 5
#define PIN_VALVE_RELAY 17
#define PIN_SWITCH_PUMP_RELAY 13
#define PIN_SWITCH_VALVE_RELAY 2


// -------------
// Defines
// -------------
#define waterflowSensorType YFS201


// Device ID
String device_id = "esp32"; // hardcoded by manufacturer
String currentTankId = ""; // once paired by application mqtt will work

// -------------
// Registeration
// -------------
// Create an Ultrasonic object
Ultrasonic ultrasonic(PIN_US_TRIGGER, PIN_US_ECHO);
// Set the LCD address (you may need to change this depending on your LCD module)
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Create a new instance of the oneWire class to communicate with any OneWire device:
OneWire oneWire(ONE_WIRE_BUS);
// Pass the oneWire reference to DallasTemperature library:
DallasTemperature sensors(&oneWire);
// tds
GravityTDS gravityTds;  // https://mikroelectron.com/Product/Analog-TDS-Total-Dissolved-Solids-Sensor-Meter-for-domestic-water-hydroponic-and-other-water-quality-testing/


FlowSensor Sensor(waterflowSensorType, PIN_WATERFLOW);

// -------------
// Intervals
// -------------
unsigned long lastSensorsCheck = 0;         // Variable to store the last time the task1 was executed
unsigned long intervalSensorsCheck = 1000;  // Interval for task1 (in milliseconds)

unsigned long lastButtonCheck = 0;       // Variable to store the last time the task2 was executed
unsigned long intervalButtonCheck = 50;  // Interval for task2 (in milliseconds)

// -------------
// Extra Vars
// -------------
int lcdCounter = 0;

// waterflow
int waterflowPulses = 0;
float factorConversion = 7.5;  //to convert from frequency to flow rate
float waterVolume = 0;
float waterVolumeAvg = 0;
int waterflowTime;
int waterflowDeltaTime;
// if (lcdCounter == NULL)
//   lcdCounter = 0;

// PH
float calibration_value = 21.34;

// wifi & mqtt
WiFiClientSecure espClient;


PubSubClient client(espClient);
// -------------
// Sensor Data
// -------------
float currentWaterflow = 0;
float currentWaterflowAvg = 0;
float currentPh = 0;
float currentLevel = 0;
float currentTemperature = 0;
float currentTurbidity = 0;
String currentTurbidityMessage;
float currentTds = 0;
bool lastPumpState = false;
bool lastValveState = false;

// -------------
// Code
// -------------

// a must function for waterflow sensor library
void IRAM_ATTR count()
{
  Sensor.count();
}

// Temperature Sensor
float readTemperature() {

  // Send the command for all devices on the bus to perform a temperature conversion:
  sensors.requestTemperatures();

  // Fetch the temperature in degrees Celsius for device index:
  float tempC = sensors.getTempCByIndex(0);  // the index 0 refers to the first device
  // Fetch the temperature in degrees Fahrenheit for device index:
  // float tempF = sensors.getTempFByIndex(0);

  // Print the temperature in Celsius in the Serial Monitor:
  // Serial.print("Temperature: ");
  // Serial.print(tempC);
  // Serial.print(" \xC2\xB0"); // shows degree symbol
  // Serial.print("C  |  ");

  // Print the temperature in Fahrenheit
  // Serial.print(tempF);
  // Serial.print(" \xC2\xB0"); // shows degree symbol
  // Serial.println("F");

  mqtt_publish("temperature", String(tempC));
  currentTemperature = tempC;

  return tempC;

  // Wait 1 second:
  // delay(1000);
}

// Level Sensor
float readLevel() {
  // Read the distance from the Ultrasonic sensor
  float distance = ultrasonic.read();

  // Convert distance to water level (adjust as needed)
  // int waterLevel = map(distance, 0, 200, 100, 0); // 200 cm

  // mqtt_publish("level", String(distance));
  mqtt_publish("waterLevel", String(distance));
  currentLevel = distance;

  return distance;
}

// Turbidity Sensor
String readTurbidity() {
  int val = analogRead(PIN_TURBIDITY_SENSOR);
  long turbidity = map(val, 0, 4000, 0, 100);

  String state;
  if (turbidity < 25)
    state = "(Clean)";
  else if (turbidity > 25 && turbidity < 50)
    state = "(Little Cloudy)";
  else if (turbidity > 50 && turbidity < 75)
    state = "(Cloudy)";
  else
    state = "(Dirty)";

  String message = String(turbidity) + " " + state;

  // mqtt_publish("turbidity", String(message));
  mqtt_publish("turbidity", String(turbidity));
  currentTurbidity = turbidity;
  currentTurbidityMessage = message;

  return message;
}

float readTDS() {
  float tdsValueRaw = analogRead(PIN_TDS);  // then get the value
  // float temperature = readTemperature();  //add your temperature sensor and read it
  float temperature = 25.0;                   //add your temperature sensor and read it
  gravityTds.setTemperature(temperature);     // set the temperature and execute temperature compensation
  gravityTds.update();                        //sample and calculate
  float tdsValue = gravityTds.getTdsValue();  // then get the value
  Serial.println("TDS: " + String(tdsValue));
  Serial.println("TDS Raw: " + String(tdsValueRaw));
  // Serial.print(tdsValue);
  // Serial.println("ppm");
  // delay(1000);
  // return analogRead(PIN_TDS);
  mqtt_publish("TDS", String(tdsValue));
  currentTds = tdsValue;

  return tdsValue;
  // return calcTDS();
}

float* readWaterflow_old() {
  static float returnArray[2];

  float frequency = getWaterflowFrequency();  //we obtain the frequency of the pulses in Hz
  // Serial.print();
  float flow_L_m = frequency / factorConversion;  //calculate the flow in L/m
  waterflowDeltaTime = millis() - waterflowTime;  //calculate the time variation
  waterflowTime = millis();
  // total water flow
  float diff = (flow_L_m / 60) * (waterflowDeltaTime / 1000);
  waterVolume = waterVolume + diff;  // volume(L)=flow(L/s)*time(s)
  // average water flow
  waterVolumeAvg = diff;  // volume(L)=flow(L/s)*time(s)
  // Serial.println(waterVolume);
  // Serial.println(waterVolumeAvg);
  returnArray[0] = waterVolume;
  returnArray[1] = waterVolumeAvg;
  mqtt_publish("waterflow", "Total: " + String(waterVolume) + ", Avg: " + String(waterVolumeAvg));
  currentWaterflow = waterVolume;
  currentWaterflowAvg = waterVolumeAvg;
  
  return returnArray;
}

float* readWaterflow() {
  static float returnArray[2];
  Sensor.read();
  Serial.print("Flow rate (L/minute): ");
  Serial.println(Sensor.getFlowRate_m());
  Serial.print("Flow rate totla volume: ");
  Serial.println(Sensor.getVolume());

  waterVolume = Sensor.getFlowRate_s(); // per second
  waterVolumeAvg = Sensor.getVolume();
  currentWaterflow = waterVolume; 
  currentWaterflowAvg = waterVolumeAvg;

  mqtt_publish("waterflow", "Total: " + String(waterVolume) + ", Avg: " + String(waterVolumeAvg));
  returnArray[0] = waterVolume;
  returnArray[1] = waterVolumeAvg;
  
  return returnArray;
}

unsigned long timeSinceLastPhSample = 0;
unsigned long timeCheckLastPhSample = 30;
unsigned long int avgval;
int buffer_arr[10];
float getPh(float voltage) {
  return 7 + ((2.5 - voltage) / 0.18);
}

int samples = 10;
float adc_resolution = 4096.0;
float readPh() {
  int measurings = 0;
  for (int i = 0; i < samples; i++) {
    measurings += analogRead(PIN_PH);
    // delay(10);
  }
  float voltage = 3.3 / adc_resolution * measurings / samples;
  // Serial.print("pH= ");
  // Serial.println(ph(voltage));
  float value =  getPh(voltage);
  mqtt_publish("pH", String(value));
  currentPh = value;
  
  return value;
  // return analogRead(PIN_PH);
}

// -------------
// UTILITIES
// -------------
void lcdWrite(int col, int row, bool clearLine, String text) {
  lcd.setCursor(col, row);
  if (clearLine)
    lcd.print("                ");
  // lcd.clear();
  lcd.setCursor(col, row);
  lcd.print(text);
}


// usage:
// 1- Define a counter before the loop
// 2- Use this method inside loop with counter param
void printLoadingLCD(int& counter) {
  String loading = ".";
  for (int i = 1; i < counter; i++) // begin from 1, let the loop below add more
    loading += ".";
  // Serial.println(counter);
  // Serial.println(loading);

  for (int i = 0; i < counter; i++) {
    lcdWrite(i, 1, false, loading);
    loading += ".";
  }
  counter++;

  if (counter > 12) // reset, lcd line limit
    counter = 1;
}

void tempLCD(float value) {
  lcdWrite(0, 0, true, "Temperature");
  lcdWrite(0, 1, true, String(value) + " C");  // " \xC2\xB0" +
}

void turbidityLCD(float value) {
  lcdWrite(0, 0, true, "Turbidity:");
  lcdWrite(0, 1, true, String(value));  // " \xC2\xB0" +
}

void levelLCD(int value) {
  lcdWrite(0, 0, true, "Water Level:");
  lcdWrite(0, 1, true, String(value) + " cm");  // " \xC2\xB0" +
}

void tdsLCD(int value) {
  lcdWrite(0, 0, true, "TDS:");
  lcdWrite(0, 1, true, String(value) + " ppm");  // " \xC2\xB0" +
}

void waterflowLCD(float totalValue, float avgValue) {
  lcdWrite(0, 0, true, "Waterflow:");
  lcdWrite(0, 1, true, "T " + String(totalValue) + " L, A " + String(avgValue));  // " \xC2\xB0" +
}

void PhLCD(float value) {
  lcdWrite(0, 0, true, "pH Value:");
  lcdWrite(0, 1, true, String(value));  // " \xC2\xB0" +
}

void switchLCD() {
  // Serial.println("Counter: " + String(lcdCounter));

  // test all
  readAll();
  
  switch (lcdCounter) {
    case 0:  // water level
      levelLCD(currentLevel);
      break;
    case 1:  // temp
      tempLCD(currentTemperature);
      break;
    case 2:  // turbidity
      turbidityLCD(currentTurbidity);
      break;
    case 3:  // tds
      tdsLCD(currentTds);
      break;
    case 4:
      {  // waterflow
        // float* theArray = readWaterflow();
        // float value[2] = { *theArray, *(theArray + 1) };
        // waterflowLCD(value[0], value[1]);
        waterflowLCD(currentWaterflow, currentWaterflowAvg);
        // Serial.println("waterflow..");
        // waterflowLCD(readTDS());
        break;
      }
    case 5:  // pH
      PhLCD(currentPh);
      break;
    default:  // reset
      lcdCounter = 0;
      switchLCD();
      break;
  }
}

void readAll() {
  Serial.println("Reading All...");
  readLevel();
  readTemperature();
  readTurbidity();
  readTDS();
  readWaterflow();
  readPh();
}

bool checkElapsedTime(unsigned long& lastCheck, unsigned long interval) {
  unsigned long currentMillis = millis();  // Get the current time

  if (currentMillis - lastCheck >= interval) {
    // Update the last check time
    lastCheck = currentMillis;

    return true;
  }
  return false;
}

void incrementWaterflowCount() {
  waterflowPulses++;  //increment the pulse variable
}

unsigned long timeSinceLastWaterflowCheck = 0;
int getWaterflowFrequency() {
  int frequency;
  // Serial.println("pulses before: " + String(waterflowPulses));
  waterflowPulses = 0;  // reset
  // delay(1000);
  // interrupts();     // We enable the interruptions
  // delay(500);
  // if (checkElapsedTime(timeSinceLastWaterflowCheck, 1000)) {}   //sample for 1 second
  // noInterrupts();  // We disable the interruptions
  // Serial.println("pulses after: " + String(waterflowPulses));
  frequency = waterflowPulses;  //Hz(pulses per second)
  return frequency;
}

// -------------------- FUNCTIONS

void togglePumpRelay(bool value) {
  value = !value; // relay logic is reversed
  digitalWrite(PIN_PUMP_RELAY, value);
  lastPumpState = value;
}
void toggleValveRelay(bool value) {
  value = !value;
  digitalWrite(PIN_VALVE_RELAY, value);
  lastValveState = value;
}

void checkRelays(String topic, bool value) {
  if (topic.endsWith("togglepump"))
    togglePumpRelay(value);

  if (topic.endsWith("togglevalve"))
    toggleValveRelay(value);
}

bool stringToBool(String str) {
  str.toLowerCase(); // Convert the string to lower case for case-insensitive comparison
  str.trim(); // Remove any leading or trailing whitespace

  if (str == "true" || str == "1" || str == "yes") {
    return true;
  } else if (str == "false" || str == "0" || str == "no") {
    return false;
  } else {
    // Optionally, handle invalid input
    Serial.println("Invalid input for boolean conversion.");
    return false; // Default value for invalid input
  }
}

void wifi_setup() {
  // delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // int loadingCounter = 1;
  lcdWrite(0, 0, false, "WiFi ...");
  lcdWrite(0, 1, false, WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    // printLoadingLCD(loadingCounter);
    delay(500);
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String result = "";
  for (int i = 0; i < length; i++) {
    result += (char)payload[i];
  }
  checkComamnd(topic, payload, length);
  Serial.println(result);

  checkRelays(String(topic), stringToBool(result));
}

void mqtt_reconnect() {
  mqtt_reconnect(false);
}

void mqtt_reconnect(bool printToLcd) {
  // Loop until we’re reconnected
  if (printToLcd) {
    lcdWrite(0, 0, false, "MQTT ...");
    lcdWrite(0, 1, false, MQTT_USERNAME);
  }
  int loadingCounter = 1;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection… ");
    // if (printToLcd) {
    //   printLoadingLCD(loadingCounter);
    // }
    String clientId = "ESP32Client";
    // Attempt to connect
    if (client.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASS)) {
      Serial.println("connected!");
      // Once connected, publish an announcement…
      client.publish(MQTT_PUB_TOPIC, "Hello, I am Connected");
      // … and resubscribe
      client.subscribe(MQTT_SUB_TOPIC);
      // client.subscribe("waterward/ayham/hub/togglevalve");
      client.subscribe("devices/esp32/commands");
      // client.subscribe("#");
      // client.subscribe(MQTT_SUB_TOPIC);
    } else {
      Serial.print("failed, rc = ");
      Serial.print(client.state());
      Serial.println("try again in 1 second(s)");
      // Wait x seconds before retrying
      delay(1000);
    }
  }
}

long lastMqttMsg = 0;
void mqtt_publish(String topic, String message) {
  // mqtt_reconnect();

  if (currentTankId == "") {
    return;
  }

  unsigned long now = millis();
  if (now - lastMqttMsg > 1000) {
    // lastMqttMsg = now;

    // char topic[256]; // Buffer for the final topic string
    // String message = "Hello World! #" + String(value);
    // String newTopic = String(MQTT_PUB_TOPIC) + String("/") + topic;
    String newTopic = "tanks/" + String(currentTankId) + String("/") + topic;
    // String topic = mainTopic.concat("/").concat(topic);
    // sprintf(topic, "%s/%s", MQTT_PUB_TOPIC, sub_topic); // Concatenate using sprintf
    // String topic = "Asdasd";
    client.publish(newTopic.c_str(), message.c_str());
    Serial.print("Publish message to " + newTopic + ": ");
    Serial.println(message);
    // delay(50);
    // lastMqttMsg = now;
  }
}

void checkComamnd(char* topic, byte* payload, unsigned int length) {
  // Check if the received message is for attaching/detaching the tank
  if (strcmp(topic, (String("devices/") + device_id + "/commands").c_str()) == 0) {
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }
    const char* command = doc["command"];
    const char* tankId = doc["tankId"];

    Serial.print("Command: ");
    Serial.println(command);
    Serial.print("Tank ID: ");
    Serial.println(tankId);

    if (strcmp(command, "attach") == 0) {
      currentTankId = String(tankId);
      Serial.println("Attached to tank: " + currentTankId);
      client.subscribe(("tanks/" + currentTankId + "/togglepump").c_str());
      client.subscribe(("tanks/" + currentTankId + "/togglevalve").c_str());
    } else if (strcmp(command, "detach") == 0) {
      if (currentTankId == tankId) {
        currentTankId = "";
        Serial.println("Detached from tank: " + String(tankId));
      }
    }
  }
}


// -------------
// MAIN FUNCTIONS
// -------------

void setup() {
  // delay(500);
  
  Serial.begin(115200);

  // pinMode(PIN_SWITCH_LCD, INPUT_PULLUP);  // Set pin X as input with internal pull-up resistor
  pinMode(PIN_SWITCH_LCD, INPUT_PULLUP);          // Set pin X as input with internal pull-up resistor
  pinMode(PIN_PH, INPUT);                         // ph
  pinMode(PIN_PUMP_RELAY, OUTPUT);                // relay
  pinMode(PIN_VALVE_RELAY, OUTPUT);               // relay
  pinMode(PIN_SWITCH_PUMP_RELAY, INPUT_PULLUP);   // relay
  pinMode(PIN_SWITCH_VALVE_RELAY, INPUT_PULLUP);  // relay

  togglePumpRelay(false);
  toggleValveRelay(false);

  lcd.init();
  // lcd.begin(16, 2);
  delay(1);
  lcd.backlight();  // Turn on the backlight
  // lcd.print("Water Level:");

  // Start up the library:
  sensors.begin();

  // waterflow
	Sensor.begin(count);

  // TDS -- TDS
  pinMode(PIN_TDS, INPUT);
  gravityTds.setPin(PIN_TDS);
  gravityTds.setAref(3.3);       //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();            //initialization

  // Waterflow
  // pinMode(PIN_WATERFLOW, INPUT);
  // attachInterrupt(digitalPinToInterrupt(PIN_WATERFLOW), incrementWaterflowCount, RISING);  //(Interrupt 0(Pin2),function,rising edge)
  // waterflowTime = millis();

  // wifi
  wifi_setup();

  // mqtt
  espClient.setCACert(MQTT_CERT);
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqtt_callback);
  mqtt_reconnect(true);
}

void loop() {

  // mqtt
  mqtt_reconnect();
  client.loop();
  // client.loop();
  // mqtt_subscribe();

  // Serial.println(String(digitalPinToInterrupt(PIN_WATERFLOW)));

  // button check
  if (checkElapsedTime(lastButtonCheck, intervalButtonCheck)) {
    // Serial.println("Button checked");
    // Serial.print("Btn: ");
    // Serial.println(digitalRead(PIN_SWITCH_LCD));
    if (digitalRead(PIN_SWITCH_LCD) == LOW) {
      lcdCounter += 1;
      switchLCD();  // force update
    }
    if (digitalRead(PIN_SWITCH_PUMP_RELAY) == LOW) {
      togglePumpRelay(!digitalRead(PIN_PUMP_RELAY));
    }
    if (digitalRead(PIN_SWITCH_VALVE_RELAY) == LOW) {
      toggleValveRelay(!digitalRead(PIN_VALVE_RELAY));
    }
  }

  // sensors check
  // if (checkElapsedTime(lastSensorsCheck, intervalSensorsCheck)) {
  //   // readTemperature();
  //   // Serial.println("Sensors checked");

  //   // last
  //   switchLCD();
  // }

  switchLCD();

  // Add a delay to avoid rapid updates
  delay(1000);
}