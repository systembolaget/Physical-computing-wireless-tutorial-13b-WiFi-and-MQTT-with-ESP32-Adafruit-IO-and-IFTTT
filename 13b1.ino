// Tutorial 13b.1. WiFi and MQTT with ESP32 and Adafruit IO

// Main parts: Adafruit Metro Mini, Adafruit AirLift FeatherWing ESP32,
// free Adafruit IO (AIO) and IFTTT subscription

// Libraries required to interface with the transceiver via SPI and
// to manage WLAN and MQTT communication; use the latest versions
#include <SPI.h>
#include <WiFiNINA.h> // Adafruit's WiFiNINA fork, use version 1.4.0
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

// Variables that remain constant
#define SPIWIFI SPI // SPI port
#define SPIWIFI_SS 10 // AirLift ESP32 chip select pin
#define ESP32_RESET 4 // AirLift ESP32 reset pin
#define SPIWIFI_ACK 3 // AirLift ESP32 ready pin
#define ESP32_GPIO0 -1 // AirLift ESP32 pin not used
//#define WLAN_SSID "#" // DSL router SSID
//#define WLAN_PASS "#" // DSL router key
#define WLAN_SSID "Smartphone" // Smartphone hotspot SSID
#define WLAN_PASS "dcb62d5396ad" // Smartphone hotspot key
#define AIO_SERVER "io.adafruit.com" // MQTT broker/server host
#define AIO_SERVERPORT 8883 // Secure port, 1883 insecure port
#define AIO_USERNAME "LagomBra" // AIO user name
#define AIO_KEY "aio_sLpl67qR53hH7aqSFvfEGrp5Ss9O" // AIO key
const int intervalWLAN = 1000; // WLAN (re-)connection interval
const int intervalPublish = 7000; // MQTT broker publish interval

// Variables that can change
bool stateLED = LOW; // Tracks if the LED is on/off
unsigned long timeNowFlashes = 0; // Timestamp that updates each loop() iteration
unsigned long intervalFlashes = 0; // Tracks the LED's on/off flash interval
bool stateFlashing = false; // Tracks if flashing was triggered
unsigned long timeNowWLAN = 0; // Timestamp that updates each loop() iteration
unsigned long timeNowPublish = 0; // Timestamp that updates each loop() iteration
bool AIOconnected = false; // Flag for publishing and subscribing

// Instances an object from the WiFiNINA library to connect and transfer
// data with SSL/TLS support
WiFiSSLClient client;

// Instances a client object from the MQTT_Client library with a WLAN
// client, MQTT server, port and credentials
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Instance a publishing object from the MQTT_Client library; a "feed" is
// just an AIO-specific MQTT topic
Adafruit_MQTT_Publish testfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Testfeed");

void setup()
{
  // Serial monitor printing is only needed for debugging
  Serial.begin(9600);

  // Override default pins with the AirLift's breakout board pins
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESET, ESP32_GPIO0, &SPIWIFI);

  // Check if the AirLift FeatherWing's WLAN module works
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Communication with WLAN module failed");

    // If not indicate there is a WLAN module malfunction
    WiFi.setLEDs(0, 255, 255); // Magenta (built-in LED has GRB colour order)

    // And trap code execution here forever
    while (true)
    {}
  }

  // Indicate there is neither WLAN router nor MQTT broker connection yet
  WiFi.setLEDs(0, 255, 0); // Red

  // Only to better see the status GRB LED
  delay(1000);
}

void loop()
{
  // A call to this function (re)connects to the WLAN router
  connectToWLAN();

  // A call to this function (re)connects to the MQTT broker
  connectToMQTT();

  // Timer to avoid hitting the free AIO tier rate limit and not block
  // code execution like using delay() does
  if (AIOconnected && (millis() - timeNowPublish > intervalPublish))
  {
    // Create a new timestamp for the next loop() execution
    timeNowPublish = millis();

    // Publish dummy data for testing the AIO feed and dashboard
    testfeed.publish(random(1024));
    
    Serial.println("Data published to AIO");
    
    // Trigger flashing as visual publishing clue
    stateFlashing = true;
  }

  // If an event triggered flashing
  if (stateFlashing == true)
  {
    // A call to this function flashes the GRB LED three times in white
    flashLED(150, 100, 3, 160, 160, 160);
  }
}

void flashLED(int timeon, int timeoff, byte flashes, byte g, byte r, byte b)
{
  // A variable to count how often the GRB LED flashed (on/off). The static
  // keyword preserves a variable's value between function calls, unlike a
  // local variable that is declared and destroyed at each new function call
  static byte counter = 0;

  // Check if it is time for a flash
  if (millis() - timeNowFlashes > intervalFlashes)
  {
    // Create a new timestamp for the next loop() execution
    timeNowFlashes = millis();

    // First check, if the GRB LED was off (= LOW)
    if (stateLED == LOW)
    {
      // If it was, use the on-time set in the function call
      intervalFlashes = timeon;

      // Then switch the GRB LED on, with the specified colours
      WiFi.setLEDs(g, r, b);

      // And remember that it is now on
      stateLED = HIGH;
    }

    // Otherwise, if the LED was on (= HIGH)
    else
    {
      // If it was, use the off-time set in the function call
      intervalFlashes = timeoff;

      // Then switch the GRB LED off
      WiFi.setLEDs(0, 0, 0);

      // And remember that it is now off
      stateLED = LOW;

      // Finally increment the counter variable after each on/off cycle
      counter++;
    }
  }

  // Check if the number of on/off cycles matches the number of flashes
  // set in the function call
  if (counter >= flashes)
  {
    // If it does, reset the flash cycle counter to zero
    counter = 0;

    // And stop triggering flashes. This ensures there is only a "single
    // shot/one shot" operation
    stateFlashing = false;

    return;
  }
}

void connectToWLAN()
{
  // Return to loop() if already connected to the WLAN router
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  // Check if it is time for a WLAN router connection attempt
  if (millis() - timeNowWLAN >= intervalWLAN)
  {
    // Create a new timestamp for the next loop() execution
    timeNowWLAN = millis();

    // Clean up connection if prior connection attempt did not succeed
    WiFi.disconnect();
    WiFi.end();

    Serial.println("Trying to connect to WLAN router");

    // Indicate that a connection attempt is being made
    WiFi.setLEDs(160, 255, 0); // Yellow

    // Attempt to connect to WLAN router
    WiFi.begin(WLAN_SSID, WLAN_PASS);

    // Possible return codes for WiFi.status()
    // WL_NO_SHIELD       = 255
    // WL_IDLE_STATUS     = 0
    // WL_NO_SSID_AVAIL   = 1
    // WL_SCAN_COMPLETED  = 2
    // WL_CONNECTED       = 3
    // WL_CONNECT_FAILED  = 4
    // WL_CONNECTION_LOST = 5
    // WL_DISCONNECTED    = 6
    Serial.println(WiFi.status());

    Serial.println("Connection to WLAN router successful");

    // Indicate that a WLAN router connection was established
    WiFi.setLEDs(255, 0, 0); // Green

    // Only to better see the status GRB LED
    delay(2000);

    // A call to this function prints various connection details
    printWLANStatus();
  }
}

void connectToMQTT()
{
  // Return to loop() if already connected to MQTT broker
  if (mqtt.connected())
  {
    return;
  }

  // When connected to the WLAN router, attempt to connect to MQTT broker,
  // meaning the AIO server on the Internet. Remember that being connected
  // to a WLAN router does not yet mean there is an Internet connection
  if (WiFi.status() == WL_CONNECTED)
  {

    Serial.println("Trying to connect to Adafruit IO");

    // Stores the error code returned by connect()
    int8_t MQTTerror;

    // In case the error code is not 0 = successful connection
    while ((MQTTerror = mqtt.connect()) != 0)
    {
      // Print an MQTT library error message matching the error code
      switch (MQTTerror)
      {
        case -2: Serial.println("Failed to subscribe"); break;
        case -1: Serial.println("Connection failed"); break;
        case 1: Serial.println("The Server does not support the level of the MQTT protocol requested"); break;
        case 2: Serial.println("The Client identifier is correct UTF-8 but not allowed by the Server"); break;
        case 3: Serial.println("The MQTT service is unavailable"); break;
        case 4: Serial.println("The data in the user name or password is malformed"); break;
        case 5: Serial.println("Not authorized to connect"); break;
        case 6: Serial.println("Exceeded reconnect rate limit. Please try again later"); break;
        case 7: Serial.println("You have been banned from connecting. Please contact the MQTT server administrator for more details"); break;
        default: Serial.println("Unknown error"); break;
      }

      // And, in case of an error
      if (MQTTerror != 0)
      {
        Serial.println("Retry to connect to Adafruit IO");

        // AUSPROBIEREN DURCH FALSCHE AIO CREDENTIALS #################################

        // Indicate that the MQTT connection failed
        WiFi.setLEDs(0, 255, 255); // Magenta

        // Only to better see the status GRB LED
        delay(2000);

        // Send a MQTT disconnect packet and end the connection
        mqtt.disconnect();

        return;
      }
    }

    // Toggle flag to enable publishing and subscribing
    AIOconnected = true;

    Serial.println("Connection to Adafruit IO successful");
  }
}

void printWLANStatus()
{
  // Print the ESP32's MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");

  // A call to this function fetches the MAC address
  printMacAddress(mac);

  // Print the SSID of the WLAN network connected to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print the ESP32's IP address assigned by the router
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // Print the router's subnet mask, usually 255.255.255.0
  Serial.print("Subnet mask: ");
  Serial.println((IPAddress)WiFi.subnetMask());

  // Print the rounter's IP address
  Serial.print("Gateway IP: ");
  Serial.println((IPAddress)WiFi.gatewayIP());

  // Print the WLAN router's signal strength received
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}
