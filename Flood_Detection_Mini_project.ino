#include <ESP8266WiFi.h>;
#include <WiFiClient.h>;

#include <ThingSpeak.h>;

#include<DHT.h>
#include<DHT_U.h>

#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define MAX_HEIGHT_OF_DAM 100

const char* ssid = "ViLiCiA"; //Your Network SSID
const char* password = "aaaallll"; //Your Network Password

//Set the WiFi library Structures :
WiFiClient client;

// Thinkspeak to upload data :
unsigned long myChannelNumber = 1012335; //Your Channel Number (Without Brackets)
const char * myWriteAPIKey = "8HLO1F54AKGTP2S3"; //Your Write API Key


int WaterInDam;
float Temperature;
float Humidity;
int WaterLevel;
float WaterFlowRate;


int UltrasonicEchoPin = (16);
int UltrasonicTrigPin = (5); 
int WaterLevelPin = (A0);

//WaterFlowRate
byte sensorInterrupt = 0;  // 0 = digital pin 2
byte WaterFlowSensorPin       = 2;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.
float calibrationFactor = 4.5;

volatile byte pulseCount;  
unsigned long oldTime;

void ICACHE_RAM_ATTR pulseCounter();

void setup(){

    //Start Serial Monitor
    Serial.begin(9600);

    delay(10);  
    
    ThingSpeak.begin(client);
  
    pinMode (UltrasonicEchoPin, INPUT);    // The ECHO pin will recieve the rebounded wave, so it must be an input type.
    pinMode (UltrasonicTrigPin, OUTPUT);  // Same as above, the TRIG pin will send the ultrasonic wave.
  
    pinMode (WaterLevelPin, INPUT);
    
    pinMode (WaterFlowSensorPin, INPUT);
    digitalWrite(WaterFlowSensorPin, HIGH);
    pulseCount        = 0;
    WaterFlowRate     = 0.0;
    oldTime           = 0;
    //attachInterrupt(sensorInterrupt, pulseCounter, FALLING);//FALLING or RISING edge can start pulse counter
    
    dht.begin();                          //Begin DHT sensor
        digitalWrite (LED_BUILTIN,LOW);

}
 
void loop(){
  
  // attempt to connect to Wifi network:
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print("\n\n\nAttempting to connect to WPA SSID: ");
    Serial.println(ssid);
    
    // Connect to WPA/WPA2 network:
    WiFi.begin(ssid, password);
    
    delay(10000);
    Serial.println(wl_status_to_string(WiFi.status()));
    
  }
  CalculationsForUltraSonicSensor();

  CalculationsForDHTSensor();

  CalculationsForWaterLevelSensor();

  CalculationsForWaterFlowSensor();

  PrintData();
  UpdateToCloud();

  delay(2000);
  
}

int status = WL_IDLE_STATUS
;

const char* wl_status_to_string(wl_status_t status) {
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
  }
}
void UpdateToCloud(){
  //Update in Thingspeak
  
  ThingSpeak.setField(1,WaterInDam);
  ThingSpeak.setField(2,Temperature);
  ThingSpeak.setField(3,Humidity);
  ThingSpeak.setField(4,WaterLevel);
  ThingSpeak.setField(5,WaterFlowRate);
  ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  
 /*
  ThingSpeak.writeField(myChannelNumber, 1, WaterInDam, myWriteAPIKey); 
  ThingSpeak.writeField(myChannelNumber, 2, Temperature, myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber, 3, Humidity, myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber, 4, WaterLevel, myWriteAPIKey);
  ThingSpeak.writeField(myChannelNumber, 5, WaterFlowRate, myWriteAPIKey);
 */
 
  delay(1000);
}


void CalculationsForUltraSonicSensor(){
  
  //Calculations for Ultrasonic sensor
  digitalWrite (UltrasonicTrigPin, HIGH);
  delay(50);
  digitalWrite (UltrasonicTrigPin, LOW);
  int duration = pulseIn(UltrasonicEchoPin, HIGH);
  WaterInDam = (MAX_HEIGHT_OF_DAM - ((duration/2)/29.1));
}


void CalculationsForDHTSensor(){
  //Calculations for DHT sensor
  // Reading temperature or humidity takes about 250 milliseconds!
  Temperature = dht.readTemperature(); //in Celcius
  //Temperature = dht.readTemperature(); //in Fahrenheit
  Humidity = dht.readHumidity();//in %
  //float HeatIndex = dht.computeHeatIndex(Temperature, Humidity); //Temperature in Fahrenheit

}

void CalculationsForWaterLevelSensor(){
  //Calculations for WaterLevel sensor
  WaterLevel = analogRead(WaterLevelPin);

}

void CalculationsForWaterFlowSensor(){
  
  //Calculations for WaterFlowRate sensor
 
    if((millis() - oldTime) > 5000) { // Only process counters once per five seconds
        
        // Disable the interrupt while calculating flow rate 
        detachInterrupt(sensorInterrupt);

        // Because this loop may not complete in exactly 1 second intervals we calculate
        // the number of milliseconds that have passed since the last execution and use
        // that to scale the output. We also apply the calibrationFactor to scale the output
        // based on the number of pulses per second per units of measure (litres/minute in
        // this case) coming from the sensor.
        WaterFlowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;

        // Note the time this processing pass was executed. Note that because we've
        // disabled interrupts the millis() function won't actually be incrementing right
        // at this point, but it will still return the value it was set to just before
        // interrupts went away.
        oldTime = millis();

        // Reset the pulse counter so we can start incrementing again
        pulseCount = 0;
        
        // Enable the interrupt again now that we've finished sending output
        attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
        
    }
    //delay(5000);
  
}

/*
Insterrupt Service Routine
 */
void pulseCounter() {
    // Increment the pulse counter
    pulseCount++;
}

void PrintData(){
  
  
    Serial.println("WaterInDam : "+String(WaterInDam));
    Serial.println("Temperature : "+String(Temperature));
    Serial.println("Humidity : "+String(Humidity));
    Serial.println("WaterLevel : "+String(WaterLevel));
    Serial.println("WaterFlowRate : "+String(WaterFlowRate));
}
