#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "PubSubClient.h"

#define sda 2               //GPIO02
#define scl 14              //GPIO14
#define power_grid 12       //GPIO12
#define MQ135_switch 5      //GPIO05
#define MQ7_switch 4        //GPIO04
#define DHT11_pin 13        //GPIO13
#define ADC 0               //GPIO00
#define DHTTYPE DHT11       //DHT11
#define APSSID "OPPO A3s"   //SSID
#define APPASS "12345678"   //PASSWORD
#define HM3301 0x40         //Slave Address
#define CMD 0x88            //Write Command

DHT dht(DHT11_pin, DHTTYPE);
char* topic = "channels/772942/publish/NZ2Q52U6P48S30A8";
char* server = "mqtt.thingspeak.com";
unsigned char data[30];
unsigned short value;
unsigned short val[7];
/*const char *str[] = {"PM1.0 Concentration(CF=1, Standard Particulate Matter): ",
                     "PM2.5 Concentration(CF=1, Standard Particulate Matter): ",
                     "PM10  Concentration(CF=1, Standard Particulate Matter): ",
                     "PM1.0 Concentration(Atmospheric Environment): ",
                     "PM2.5 Concentration(Atmospheric Environment): ",
                     "PM10  Concentration(Atmospheric Environment): "
                    };*/
WiFiClient wifiClient;
PubSubClient client(server, 1883, wifiClient);

void callback(char* topic, byte* payload, unsigned int lenght) {
  //handle message arrived
}

void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  pinMode(power_grid, OUTPUT);
  pinMode(MQ135_switch, OUTPUT);
  pinMode(MQ7_switch, OUTPUT);
  pinMode(DHT11_pin, INPUT);
  pinMode(ADC, INPUT);

  ///////////////////////////////////////////////////////////////////////////////
  //Turning ON the power grid
  digitalWrite(power_grid, HIGH);  //power_grid = HIGH
  //delay(90000);                    //Leaves it open for 1min and 30sec
  ///////////////////////////////////////////////////////////////////////////////
  digitalWrite(MQ135_switch, LOW);
  digitalWrite(MQ7_switch, LOW);
  dht.begin();
  delay(100);

  WiFi.begin(APSSID, APPASS);

  Serial.println("");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!!!");
  delay(500);
  Serial.print("Connected to: ");
  Serial.println(APSSID);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  Serial.println("----------------------------------");
  delay(500);
  Serial.println("----------------------------------");
  Serial.println("Air Quality Monitoring System");
  Serial.println("By Romeo Gomez");
  Serial.println("Version: AQMS v1.1, 2019");
  Serial.println("BNS OLPCC OJT's");
  Serial.println("#OneManSquad");
  Serial.println("----------------------------------");
  delay(2500);

  String clientName = "Air Quality Monitoring Device";
  Serial.print("Connecting ");
  Serial.print(server);
  Serial.print(" as ");
  Serial.println(clientName);

  ///////////////////////////////////////////////////////////////////////////////
  //MQTT config
  if (client.connect((char*) clientName.c_str())) {
    Serial.println("Connected to MQTT broker");
    Serial.print("Topic ");
    Serial.println(topic);

    if (client.publish(topic, "Air Quality Monitoring Started!!!")) {
      Serial.println("Publish ok");
    }
    else {
      Serial.println("Publish failed");
    }
  }
  else {
    Serial.println("MQTT connect failed");
    Serial.println("Will reset and try again...");
    abort();
  }
  ///////////////////////////////////////////////////////////////////////////////

  Wire.beginTransmission(HM3301);
  Wire.write(CMD);
  Wire.endTransmission();
  delay(100);

  Serial.println("Air Quality Monitoring Started!!!");
  Serial.println("Sensors pre-heating, please wait...");
  for (int i=0;i<80;i++) {
    delay(1000);
    Serial.print(".");
    if (i == 40) {
      Serial.println("");
      Serial.println("50% Done... Please wait...");
      Serial.print(".");
    }
  }
  Serial.println("");
  Serial.println("Pre-heat done!!!");
  Serial.println("Gathering DATA now!!!");
  delay(100);
}

void loop() {
  int time_out_count = 0;
  
  ///////////////////////////////////////////////////////////////////////////////
  //MQ135 and MQ7 Switch reading
  int MQ135_value = 0;
  int MQ7_value = 0;
  digitalWrite(MQ135_switch, HIGH);
  MQ135_value = analogRead(ADC);
  delay(100);

  digitalWrite(MQ135_switch, LOW);
  delay(10);
  digitalWrite(MQ7_switch, HIGH);
  MQ7_value = analogRead(ADC);
  delay(100);

  digitalWrite(MQ7_switch, LOW);
  ///////////////////////////////////////////////////////////////////////////////
  //DHT11 readings
  float h = dht.readHumidity();     //Humidity
  float t = dht.readTemperature();  //Temperature
  ///////////////////////////////////////////////////////////////////////////////

  Wire.requestFrom(0x40, 29);
  while (29 != Wire.available()) {
    time_out_count++;
    if (time_out_count>10)
    delay(1);
  }
  for (int i=0;i<28;i++) {
    data[i] = Wire.read();
    //Serial.print(data[i],HEX);
    //Serial.print(" ");
    //if ((0==(i)%5) || (0==i)) {
      //Serial.println("");
    //}
  }
  //Serial.println("");
  //Serial.println("");

  value = 0;
  for (int i=1;i<7;i++) {
    value = data[i*2+2]<<8|data[i*2+3];
    //Serial.print(str[i-1]);
    //Serial.println(value);
    val[i-1] = value;
    /*if (i == 1) {
      value[0] = value;
    }
    if (i == 2) {
      value[1] = value;
    }
    if (i == 3) {
      value[2] = value;
    }*/
  }
  //Serial.println("");
  //Serial.println("");
  ///////////////////////////////////////////////////////////////////////////////
  //MQTT sending protocol
  String string_t = t;
  String string_h = h;
  String string_MQ135_value = MQ135_value;
  String string_MQ7_value = MQ7_value;
  String string_val[0] = val[0];
  String string_val[1] = val[1];
  String string_val[2] = val[2];
  String payload = "field1=";
  payload += string_t;
  payload += "&field2=";
  payload += string_h;
  payload += "&field3=";
  payload += string_MQ135_value;
  payload += "&field4=";
  payload += string_MQ7_value;
  payload += "&field5=";
  payload += string_val[0];
  payload += "&field6=";
  payload += string_val[1];
  payload += "&field7=";
  payload += string_val[2];
  payload += "&status=MQTTPUBLISH";
  
  if (client.connected()) {
    Serial.print("Sending: ");
    Serial.println(payload);

    if (client.publish(topic, (char*) payload.c_str())) {
      Serial.println("Publish ok");
    }
    else {
      Serial.println("Publish failed");
    }
  }
  ///////////////////////////////////////////////////////////////////////////////

  //deep sleep mode
  Serial.println("");
  Serial.println("Entering deep sleep mode in ");
  for (int i=3;i>0;i--) {
    Serial.print(i);
    if (i != 1) {
      Serial.print(",");
    }
    delay(1000);
  }
  Serial.print(", Zzzzz...");
  ESP.deepSleep(300e6);
}
