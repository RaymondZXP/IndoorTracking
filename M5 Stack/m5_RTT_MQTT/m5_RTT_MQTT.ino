#include <PubSubClient.h>
#include <string.h>
#include <M5Stack.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>


// Wifi Credentials
char* ssid = "yourwifissid";
char* pass = "yourwifipassword";


// MQTT broker address
const char* mqtt_server = "broker.mqttdashboard.com"; // use your secure broker address

WiFiClient client;
PubSubClient mqtt_client(client);



// Function to connect to WiFi network

void connectToNetwork();
void mqtt_callback(char* topic, byte* message, unsigned int length);
void reconnect();
void update_server_gateway_info();


String wifi_macaddress;

void setup() {
  // put your setup code here, to run once:
  M5.begin();
  M5.Power.begin();

  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(YELLOW);
  M5.Lcd.setTextSize(2);

  Serial.begin(115200);

  // Connecting to network
  connectToNetwork();
  wifi_macaddress = WiFi.macAddress();


  M5.Lcd.println();
  M5.Lcd.print("IP Address: ");
  M5.Lcd.println(WiFi.localIP());


  M5.Lcd.println();
  M5.Lcd.print("MAC Address: ");
  M5.Lcd.println(wifi_macaddress);

  delay(1000);

//This is used when the server is set up, skip
//  update_server_gateway_info();

  delay(1000);
  // Setting MQTT server
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(mqtt_callback);


}


void loop() {



  double x_2 = 0;
  double x_1 =  5.19996571;
  double x_0 = -108.14011876;

  const size_t TAG_CAPACITY = JSON_ARRAY_SIZE(20);
  StaticJsonDocument<200> doc;
  StaticJsonDocument<TAG_CAPACITY> tags_info;
  JsonArray tag_array = tags_info.to<JsonArray>();

  String tag_id = "1";

  char msg[256];
  doc["Id"] = wifi_macaddress;


  int receivedByte;
  int clock_cycle_sum = 0, receivedPackageCount = 0;
  double distance = 0;
  double clock_cycles = 0;




  if (!mqtt_client.connected()) {
    reconnect();
  }
  mqtt_client.loop();


  unsigned long startedWaiting = millis();
  while (millis() - startedWaiting <= 1000)
  {
    if (Serial.available() ) {
      receivedByte = Serial.read();
      if (receivedByte <= 25 && receivedByte >= 18) {
        clock_cycle_sum += receivedByte;
        receivedPackageCount += 1;
      }
    }
  }

  //  receivedPackageCount = 1800;

  if (receivedPackageCount) {


    clock_cycles = double(clock_cycle_sum) / (double)receivedPackageCount;
    distance = x_2 * clock_cycles * clock_cycles + x_1 * clock_cycles + x_0;
    if (distance < 0) {
      distance = 0;
    }



    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println(receivedPackageCount);
    M5.Lcd.println(distance);

    StaticJsonDocument<200> tag;
    tag["id"] = tag_id;
    tag["distance"] = distance;

    tag_array.add(tag);

    doc["value"] = tag_array;

    serializeJson(doc, Serial);
    serializeJsonPretty(doc, msg);
    mqtt_client.publish("FYP/tracking", msg );
  }


}


void connectToNetwork() {
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {

    delay(1000);
    Serial.println("Establishing connection to WiFi..");
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Establishing connection to WiFi..");
  }

  Serial.println("Connected to network");


}

// Set callback for MQTT
void mqtt_callback(char* topic, byte* message, unsigned int length) {

}

// Reconnecting to MQTT
void reconnect() {

  while (WiFi.status() != WL_CONNECTED) {

    delay(1000);
    Serial.println("Reconnecting to WiFi..");
    M5.Lcd.clear();
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Reconnecting to WiFi..");
  }
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(wifi_macaddress.c_str(), "youraccount", "yourpassword")) { // use your secured address and password if you have one
      Serial.println("connected");
      // Subscribe
      mqtt_client.setCallback(mqtt_callback);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void update_server_gateway_info() {


  HTTPClient http_client;
  DynamicJsonDocument gateway_doc(1024);
  StaticJsonDocument<200> gateway_info;
  char gateway_info_str[100];
  boolean gateway_exists = false;

  gateway_info["name"] = "M5 Gateway";
  gateway_info["address"] = wifi_macaddress;

  String check_url = "http://127.0.0.1:8000/api/gateway_macaddress/" + wifi_macaddress;
  serializeJson(gateway_info, gateway_info_str);

  M5.Lcd.println("Updating Server Gateway Info");
  http_client.begin(check_url);
  int httpCode = http_client.GET();

  while (!(httpCode > 0)) { //Check for the returning code
    int httpCode = http_client.GET();
    M5.Lcd.println("Update Failed, retry");
  }

  if (httpCode != 404) {
    gateway_exists = true;
    http_client.end();

  }

  if (!gateway_exists) {
    M5.Lcd.println("Creating gateway on server");
    http_client.begin("http://127.0.0.1:8000/api/anchor");
    http_client.addHeader("Content-Type", "application/json");

    httpCode = http_client.POST(gateway_info_str);

    if (httpCode > 0) {

      String response = http_client.getString();

      Serial.println(httpCode);
      Serial.println(response);

    } else {

      Serial.print("Error on sending PUT Request: ");
      Serial.println(httpCode);

    }
  }
  else {
    M5.Lcd.println("Gateway already exists on server");

  }

  http_client.end(); //Free the resources
}
