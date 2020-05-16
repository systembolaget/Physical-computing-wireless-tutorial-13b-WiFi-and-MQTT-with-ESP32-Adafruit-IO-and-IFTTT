// Tutorial 13b. WiFi and MQTT with ESP32, Adafruit IO and IFTTT

// Main parts: Adafruit Metro Mini, Adafruit AirLift FeatherWing ESP32,
// CdS photoresistor, momentary switch

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
#define WLAN_SSID "Lagom"
#define WLAN_PASS "63948100905083530879"
#define AIO_SERVER "io.adafruit.com" // MQTT broker/server host
#define AIO_SERVERPORT 8883 // Secure port, 1883 insecure port
#define AIO_USERNAME "Systembolaget"
#define AIO_KEY "aio_kDpJ433pAtTmV770wRnyRaFpc0tR"

const byte pinSwitch = A0; // Analog input pin from momentary switch
const byte pinSensor = A1; // Analog input pin from photoresistor
const int intervalPhotoresistor = 30000; // Publishing interval 30s

// Variables that can change
bool stateLED = LOW; // Tracks if the LED is on/off
unsigned long timeNowLED = 0; // Timestamp that updates each loop() iteration
unsigned long timeIntervalLED = 0; // Tracks the LED's on/off interval
bool flashTriggered = false; // Tracks if flashing was triggered
byte lastMomentarySwitchState = HIGH; // Momentary switch assumed open at start
unsigned long timeNowPhotoresistor = 0; // Timestamp that updates each loop() iteration

// A struct can bundle two or more different variables (members),
// and each can be of a different data type, quite like a shopping
// bag can contain different goods. Here, the struct is first named
// (transmission), then two variables (members) are declared, and
// the struct is assigned to a variable (nodeTX) so it can be used
struct transmission
{
  bool momentaryswitch; // Momentary switch, values 0 or 1 (false or true)
  int photoresistor; // Photoresistor, values 0 - 1023 (1023 maximum brightness)
  // Add more variables for more feeds
} nodeTX;

// Status must be set to idle for when WiFi.begin() is called for
// the first time; it remains so until the WLAN connection attempt
// fails, succeeds or is re-established
int status = WL_IDLE_STATUS;

// Instances an object from the WiFiNINA library to connect and
// transfer data with SSL/TLS support
WiFiSSLClient client;

// Instances a client object from the MQTT_Client library with the
// WLAN client, MQTT server, port and login credentials
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Instance publishing objects from the MQTT_Client library for each
// struct member (a feed is an Adafruit IO specific MQTT topic)
Adafruit_MQTT_Publish momentaryswitchfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Momentaryswitch");
Adafruit_MQTT_Publish photoresistorfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Photoresistor");

void setup()
{
  // Serial monitor printing is only needed for debugging
  Serial.begin(9600);
  while (!Serial); Serial.println();

  // Override default pins with the AirLift's breakout board pins
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESET, ESP32_GPIO0, &SPIWIFI);

  // Indicate there is no WLAN router connection yet; GRB colour order!
  WiFi.setLEDs(0, 255, 0); // Red

  // To better see the status indicating RGB LED's state
  delay(2000);

  // Initialise momentary switch pin with an internal pull-up resistor
  // so that the momentary switch is read as open (= HIGH) at start
  pinMode (pinSwitch, INPUT_PULLUP);

  // Initialise sensor pin with an internal pull-up resistor
  pinMode (pinSensor, INPUT_PULLUP);
}

void loop()
{
  // A call to this function connects to the WLAN router
  connectToWLAN();

  // A call to this function connects to the MQTT broker
  connectToMQTT();

  // A call to this function fetches a reading from the momentary switch pin
  readMomentarySwitch();

  // A call to this function fetches a reading from the sensor pin
  readPhotoresistor();

  // If the momentary switch button was pressed down
  if (nodeTX.momentaryswitch == true)
  {
    // Then publish the struct member value to the MQTT broker
    momentaryswitchfeed.publish((long) nodeTX.momentaryswitch);

    // And set the momentary switch button struct member variable to 0
    // (off), so that each button press only transmits a single 1 (on)
    // when pressing the button down; otherwise, a 1 would be sent for
    // as long as the button is held in the pressed down state
    nodeTX.momentaryswitch = false;

    // To better see the Adafruit IO dashboard indicator state
    delay(250);

    // Publish the struct member value to the MQTT broker. It must be
    // cast into a long data-type, because the compiler cannot know if
    // the transmission struct members (bool, char, byte, int) shall
    // be promoted to a long or a float data-type
    momentaryswitchfeed.publish((long) nodeTX.momentaryswitch);
  }

  // If the momentary switch button was pressed down
  if (flashTriggered == true)
  {
    // Flash the LED with different on and off intervals n times
    flash(125, 25, 5, 0, 0, 255); // Blue
  }

  // Publish the struct member value to the MQTT broker periodically
  publishData(timeNowPhotoresistor, intervalPhotoresistor, photoresistorfeed, (long) nodeTX.photoresistor);
}

// The transmission struct defined above could hold many variables,
// each with a different data-type (bool, char, byte, int). Instead
// of writing four functions, and creating a lot of redundant code,
// a template function can be used. It accepts various data-types
// (T data) as parameters and the compiler will instantiate as many
// different functions as are necessary
// Instead of using values, the parameters for keeping time and the
// MQTT library feed must be passed with their memory address (&),
// so that the function can change their value
template<typename T> void publishData(unsigned long &now, const int interval, Adafruit_MQTT_Publish &feed, T data)
{
  // Check if it is time to publish to the feed stored at &feed
  if (millis() - now >= interval)
  {
    // Update the timestamp at &now for the next loop() execution
    now = millis();

    // Indicate that data will be sent
    flashTriggered = true;

    // Publish T data to the MQTT broker feed
    feed.publish(data);
  }
}

void flash(int timeon, int timeoff, byte flashes, byte g, byte r, byte b)
{
  // A variable to count how often the LED flashed (on/off). The static
  // keyword preserves a variable's value between function calls, unlike
  // a local variable declared and destroyed at each new function call
  static byte counter = 0;

  // Check if it is time to flash the LED
  if (millis() - timeNowLED > timeIntervalLED)
  {
    // Create a new timestamp for the next loop() execution
    timeNowLED = millis();

    // First check, if the LED was off (= LOW); if it was
    if (stateLED == LOW)
    {
      // Use the on-time set in the function call
      timeIntervalLED = timeon;

      // Then switch the LED on with the specified colours
      WiFi.setLEDs(g, r, b);

      // And remember that it is now on
      stateLED = HIGH;
    }

    // Otherwise, if the LED was on (= HIGH)
    else
    {
      // Use the off-time set in the function call
      timeIntervalLED = timeoff;

      // Then switch the LED off
      WiFi.setLEDs(0, 0, 0);

      // And remember that it is now off
      stateLED = LOW;

      // Finally increment the counter variable at each on/off cycle
      counter++;
    }
  }

  // Check if the number of on/off cycles matches the number of flashes
  // set in the function call; if it does
  if (counter >= flashes)
  {
    // Reset the flash cycle counter to zero
    counter = 0;

    // And stop the switch triggering flashes, if the user continues
    // holding the momentary switch button down. This ensures there
    // is only a "single shot"/"one shot" operation
    flashTriggered = false;

    return;
  }
}

void readMomentarySwitch()
{
  // The momentary switch is hardware debounced with a 1uF capacitor; no
  // debouncing code is necessary. See http://www.gammon.com.au/switches
  // Read the voltage from the momentary switch pin to see if something
  // has changed (was the button pressed or released?)
  byte momentarySwitchState = digitalRead (pinSwitch);

  // Has the momentary switch state changed since the last time it was
  // checked (once every loop() iteration)?
  if (momentarySwitchState != lastMomentarySwitchState)
  {
    // First, store the current switch state for the next time around
    lastMomentarySwitchState = momentarySwitchState;

    // Next, test if the switch was closed (button was pressed)
    if (momentarySwitchState == LOW)
    { // If it was, set the struct member value to 1
      nodeTX.momentaryswitch = true;

      // And remember that the momentary switch button was pressed
      flashTriggered = true;
    }
    else
    {
      // If it wasn't, set the struct member value to 0
      nodeTX.momentaryswitch = false;
    }
  }
}

void readPhotoresistor()
{
  // Read the voltage from the sensor pin and map the ADC's output to
  // the value range of 0 - 1023. The minimum and maximum values of 100
  // - 1000 used here are specific to the room and daytime where this
  // set-up was used; the theoretically achievable range is 0 - 1023.
  // Constraining the output of map() clips outlier values passed into
  // the map() function
  nodeTX.photoresistor = constrain(map(analogRead(pinSensor), 100, 1000, 0, 1023), 0, 1023);
}

void connectToWLAN()
{
  // Check if the AirLift FeatherWing's WLAN module works
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Connection to AirLift FeatherWing WLAN module failed");

    // If not, indicate there is a WLAN module malfunction
    WiFi.setLEDs(0, 255, 255); // Magenta

    // And stop forever
    while (true);
  }

  // Return to loop() if already connected to the WLAN router
  if (WiFi.status() == WL_CONNECTED)
  {
    return;
  }

  Serial.println("Trying to connect to WLAN router");

  // Indicate there is no WLAN router connection just yet
  WiFi.setLEDs(128, 255, 0); // Orange

  do
  {
    // Start connection to WLAN router and print a status value
    status = WiFi.begin(WLAN_SSID, WLAN_PASS);
    // WL_IDLE_STATUS     = 0
    // WL_NO_SSID_AVAIL   = 1
    // WL_SCAN_COMPLETED  = 2
    // WL_CONNECTED       = 3
    // WL_CONNECT_FAILED  = 4
    // WL_CONNECTION_LOST = 5
    // WL_DISCONNECTED    = 6
    Serial.println(WiFi.status());
    delay(100);
  }
  while (status != WL_CONNECTED);

  Serial.println("Connection to WLAN router successful");

  // Indicate that the WLAN router connection was established
  WiFi.setLEDs(192, 255, 0); // Yellow

  // To better see the status indicating RGB LED's state
  delay(1000);

  // A call to this function prints the WLAN network SSID, the signal
  // strength and the WiFi module's IP address
  printWLANStatus();
}

void connectToMQTT()
{
  // Return to loop() if already connected to the MQTT broker
  if (mqtt.connected())
  {
    return;
  }

  Serial.println("Trying to connect to Adafruit IO");

  // Stores a printable string version of the error code returned by
  // connect()
  int8_t MQTTerrorString;

  // In case the error code is not 0 = successful connection, then
  while ((MQTTerrorString = mqtt.connect()) != 0)
  {
    // Print an error message that matches the error code
    switch (MQTTerrorString)
    {
      case 1: Serial.println("Wrong protocol"); break;
      case 2: Serial.println("ID rejected"); break;
      case 3: Serial.println("Server unavailable"); break;
      case 4: Serial.println("Bad username/password"); break;
      case 5: Serial.println("Not authenticated"); break;
      case 6: Serial.println("Failed to subscribe"); break;
      default: Serial.println("Connection failed"); break;
    }

    // In case of an error
    if (MQTTerrorString >= 0)
    {
      // Send a MQTT disconnect packet and break the connection
      mqtt.disconnect();

      // And indicate that the connection failed
      WiFi.setLEDs(255, 0, 255); // Cyan

      Serial.println("Retry to connect");

      // Wait n seconds before returning to loop() and trying again
      delay(3000);
    }
  }

  // Indicate that the MQTT broker connection was established
  WiFi.setLEDs(255, 0, 0); // Green

  // To better see the status indicating RGB LED's state
  delay(1000);

  Serial.println("Connection to Adafruit IO successful");
}

void printWLANStatus()
{
  // Print the SSID of the WLAN network connected to
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // Print the received WLAN signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");

  // Print the dynamically assigned IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}
