// Tutorial 13b.3. WiFi and MQTT with ESP32, Adafruit IO and IFTTT

// Main parts: Adafruit Metro Mini, Adafruit AirLift FeatherWing ESP32,
// CdS photoresistor, momentary switch, free Adafruit IO (AIO) and IFTTT
// subscriptions

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
const int intervalMQTT = 1000; // MQTT (re-)connection interval
const int intervalPublish = 5000; // MQTT broker publish interval
const byte pinSwitch = A0; // Analog input pin from momentary switch
const byte pinSensor = A1; // Analog input pin from photoresistor

// An enum is a user-defined data-type, consisting of a set of named
// values or elements. Here, the variable "connectionState" can contain
// any of the five different states of the WLAN and MQTT state machine.
// The ": byte" specifier means the states internally correspond to
// the single byte values 0 to 4
enum : byte {
  WLAN_DOWN_MQTT_DOWN,
  WLAN_STARTING_MQTT_DOWN,
  WLAN_UP_MQTT_DOWN,
  WLAN_UP_MQTT_STARTING,
  WLAN_UP_MQTT_UP
} connectionState;

// Variables that can change
byte stateSwitchLast = HIGH; // Momentary switch assumed open at start

bool stateLED = LOW; // Tracks if the LED is on/off
unsigned long timeNowFlashes = 0; // Timestamp that updates each loop() iteration
unsigned long intervalFlashes = 0; // Tracks the LED's on/off flash interval
bool stateFlashing = false; // Tracks if flashing was triggered

unsigned long timeNowWLAN = 0; // Timestamp that updates each loop() iteration
unsigned long timeNowMQTT = 0; // Timestamp that updates each loop() iteration
unsigned long timeNowPublish = 0; // Timestamp that updates each loop() iteration

bool AIOconnected = false; // Flag for publishing and subscribing
bool enableSerialPrint = true; // Flag for printing to the serial monitor

// A struct can bundle two or more different variables (members),
// and each can be of a different data-type, quite like a shopping bag
// can contain different goods. Here, the struct is named (transmission),
// then two variables (members) are declared and the struct is assigned
// to a variable (nodeTX) to be used. nodeTX stands for transmitting
// node, because later, this set-up can receive data from other remote
// sensor nodes, all bundled and transmitted together
struct transmission
{
  bool momentaryswitch; // Momentary switch, values 0 or 1 (false or true)
  int photoresistor; // Photoresistor, values 0 - 1023 (1023 maximum brightness)
  // Add variables here if you want to receive(RX)/transmit(TX) more
} nodeTX;

// Instances an object from the WiFiNINA library to connect and
// transfer data with SSL/TLS support
WiFiSSLClient client;

// Instances a client object from the MQTT_Client library with a
// WLAN client, MQTT server, port and credentials
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Instance publishing objects from the MQTT_Client library for each
// struct member; a "feed" is just an AIO-specific MQTT topic
Adafruit_MQTT_Publish momentaryswitchfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Momentaryswitch");
Adafruit_MQTT_Publish photoresistorfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Photoresistor");

void setup()
{
  // Serial monitor printing is only needed for debugging
  Serial.begin(9600);

  // Override default pins with the AirLift's breakout board pins
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESET, ESP32_GPIO0, &SPIWIFI);

  // Indicate there is no WLAN router connection yet; GRB colour order!
  WiFi.setLEDs(0, 255, 0); // Red

  // Only to better see the status GRB LED
  delay(1000);

  // Initialise momentary switch pin with an internal pull-up resistor
  // so that the momentary switch is read as open (= HIGH) at start
  pinMode (pinSwitch, INPUT_PULLUP);

  // Initialise sensor pin with an internal pull-up resistor
  pinMode (pinSensor, INPUT_PULLUP);
}

void loop()
{
  // A call to this function connects to the WLAN router and MQTT broker
  connectToWLANAndMQTT();

  // A call to this function fetches a reading from the momentary switch pin
  readMomentarySwitch();

  // A call to this function fetches a reading from the sensor pin
  readPhotoresistor();

  // If the momentary switch button was pressed
  if (AIOconnected && nodeTX.momentaryswitch == true)
  {
    // Then publish the struct member value to the MQTT broker
    momentaryswitchfeed.publish((long) nodeTX.momentaryswitch);

    // And set the momentary switch button struct member variable to 0
    // (off), so that each button press only transmits a single 1 (on)
    // when pressing the button down; otherwise, a 1 would be sent for
    // as long as the button is held in the pressed state
    nodeTX.momentaryswitch = false;

    // Only to better see the Adafruit IO dashboard indicator change
    delay(500);

    // Publish the struct member value to the MQTT broker. It must be
    // cast into a long data-type, because the compiler cannot know if
    // the transmission struct members (bool, char, byte, int) shall
    // be promoted to a long or a float data-type
    momentaryswitchfeed.publish((long) nodeTX.momentaryswitch);
  }

  // Timer to avoid hitting the free AIO tier rate limit and not block
  // code execution like using delay() does
  if (AIOconnected && (millis() - timeNowPublish > intervalPublish))
  {
    // Create a new timestamp for the next loop() execution
    timeNowPublish = millis();

    // Publish the struct member value to the AIO feed and dashboard
    //publishData(timeNowPublish, intervalPublish, photoresistorfeed, (long) nodeTX.photoresistor);
    photoresistorfeed.publish((long) nodeTX.photoresistor);

    // And trigger flashing as a visual clue
    stateFlashing = true;
  }

  // If an event triggered flashing
  if (stateFlashing == true)
  {
    // A call to this function flashes the GRB LED three times (white)
    flashLED(125, 75, 3, 255, 255, 255); // White
  }
}

// The transmission struct defined above could hold many variables,
// each with a different data-type (bool, char, byte, int). Instead
// of writing four functions, and creating a lot of redundant code,
// a template function can be used. It accepts various data-types
// (T data) as parameters and the compiler will instantiate as many
// different functions as necessary. Instead of using values, the
// parameters for keeping time and the MQTT library feed must be
// passed with their memory address (&), so that the function can
// change their value
/*template<typename T> void publishData(unsigned long &now, const int interval, Adafruit_MQTT_Publish &feed, T data)
  {
  // Check if it is time to publish
  if (millis() - now >= interval)
  {
    // Update the timestamp at &now for the next loop() execution
    now = millis();

    // Publish T data to the MQTT broker feed
    feed.publish(data);

    // And trigger flashing as a visual clue
    stateFlashing = true;
  }
  }*/

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
  }
}

void readMomentarySwitch()
{
  // The momentary switch is hardware debounced with a 1uF capacitor; no
  // debouncing code is necessary. See http://www.gammon.com.au/switches
  // Read the voltage from the momentary switch pin to see if something
  // has changed (was the button pressed or released?)
  byte stateSwitchNow = digitalRead (pinSwitch);

  // Has the momentary switch state changed since the last time it was
  // checked (once every loop() iteration)?
  if (stateSwitchNow != stateSwitchLast)
  {
    // First, store the current switch state for the next time around
    stateSwitchLast = stateSwitchNow;

    // Next, test if the switch was closed (button was pressed)
    if (stateSwitchNow == LOW)
    {
      // If it was, set the struct member value to 1
      nodeTX.momentaryswitch = true;

      // And trigger flashing as a visual clue
      stateFlashing = true;
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

void connectToWLANAndMQTT()
{
  // The first time the function runs, the WLAN router and MQTT broker
  // are disconnected; therefore set the state machine's initial state
  static byte stateConnection = WLAN_DOWN_MQTT_DOWN;

  // Next, check if the previously successful WLAN router and MQTT broker
  // connection was dropped
  if (WiFi.status() == WL_DISCONNECTED && stateConnection == 4)
  {
    // If so, disable publishing
    AIOconnected = false;

    Serial.println("Restart WLAN connection");

    // And reset the state machine to its initial state (restart)
    stateConnection = WLAN_DOWN_MQTT_DOWN;
  }

  // Based on the state machine's state, at each loop() execution,
  // execute only the corresponding code (switch statement)
  switch (stateConnection)
  {
    // If there is no WLAN router connection
    case WLAN_DOWN_MQTT_DOWN:
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("Start WLAN connection");

        WiFi.setLEDs(0, 255, 0); // Red

        // Start the connection
        WiFi.begin(WLAN_SSID, WLAN_PASS);

        // Set the timer
        timeNowWLAN = millis();

        // And advance the state machine to the next state
        stateConnection = WLAN_STARTING_MQTT_DOWN;
      }
      // The break keyword means that no further case code will be
      // evaluated. In other words, the code immediately returns
      // to loop(), because after the last case statement, there
      // is no other code in this function to jump to
      break;

    // If the WLAN router connection was started
    case WLAN_STARTING_MQTT_DOWN:

      // And it is time
      if (millis() - timeNowWLAN >= intervalWLAN)
      {
        Serial.println("Wait for WLAN connection");

        // And if the WLAN router connection is established
        if (WiFi.status() == WL_CONNECTED)
        {
          // Print various device and network details
          printWLANStatus();

          // And advance the state machine to the next state
          stateConnection = WLAN_UP_MQTT_DOWN;
        }
        else
        {
          // Otherwise, if the WLAN router connection was not established
          Serial.println("Retry WLAN connection");

          // Clear the connection for the next attempt
          WiFi.end();
          WiFi.disconnect();

          // And reset the state machine to its initial state (restart)
          stateConnection = WLAN_DOWN_MQTT_DOWN;
        }
      }
      break;

    // If the WLAN router connection was established
    case WLAN_UP_MQTT_DOWN:

      // And if no MQTT broker connection was established yet
      if ((WiFi.status() == WL_CONNECTED) && !mqtt.connected())
      {
        Serial.println("WLAN connected. Start MQTT connection");

        WiFi.setLEDs(255, 0, 0); // Green

        // Set the timer
        timeNowMQTT = millis();

        // And advance the state machine to the next state
        stateConnection = WLAN_UP_MQTT_STARTING;
      }
      break;

    // If the MQTT broker connection was started
    case WLAN_UP_MQTT_STARTING:

      // And it is time
      if (millis() - timeNowMQTT >= intervalMQTT)
      {
        Serial.println("WLAN connected. Wait for MQTT connection");

        // And if the MQTT broker connection is established
        if (mqtt.connect() == 0)
        {
          // Advance the state machine
          stateConnection = WLAN_UP_MQTT_UP;
        }
        else
        {
          // Otherwise if the MQTT broker connection could not be established
          Serial.println("Retry MQTT connection");

          // Reset the state machine to its previous state (go back)
          stateConnection = WLAN_UP_MQTT_DOWN;
        }
      }
      break;

    // If both the WLAN router and MQTT broker connections were established
    case 4:
      if (enableSerialPrint == true)
      {
        // Print only once to stop serial monitor pollution
        Serial.println("WLAN and MQTT connected");
      }

      // Toggle flag to disable repeat printing of last debug message
      enableSerialPrint = false;

      WiFi.setLEDs(0, 0, 0); // Off

      // Toggle flag to enable publishing and subscribing
      AIOconnected = true;
      break;
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
