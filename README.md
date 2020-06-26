# Physical computing wireless tutorial - WiFi and MQTT with ESP32, Adafruit IO and IFTTT

Easy Arduino TX to cloud services. How to publish and tweet       send and chart data with Adafruit IO and trigger tweets from a momentary switch button or other event

13b1 (basic principle) - connect to WLAN router and MQTT broker with two functions; TX dummy data (random numbers)  
13b2 (working prinicple) - connect to WLAN router and MQTT broker with a state machine; TX dummy data (random numbers)  
13b3 (fully working principle) - connect to WLAN router and MQTT broker with a state machine; TX photoresistor values, TX momentary switch status and tweet via IFTTT

### Result

![](Assets/13b%20result.jpg)

### Schematic

![](Assets/13b%20schematic.png)

### BOM

<pre>
€ 14,00 Adafruit Metro Mini 328 5V 16MHz microcontroller
€ 19,00 Adafruit AirLift FeatherWing - ESP32 WiFi
€  2,00 CdS photoresistor
€  1,00 Tactile button momentary switch
€  4,00 Half-size transparent breadboard
€  2,00 Breadboard mini modular black
€  1,00 Jumper cables
€  1,00 2,1mm DC barrel-jack
€  2,00 1µF & 100 µF 10V el. caps, 4,7kΩ resistor
€  3,00 DURACELL Plus 9V battery
€ 49,00
</pre>  

### Useful links

μc https://www.adafruit.com/product/2590  
WiFi module https://www.adafruit.com/product/4264  
Adafruit WiFiNINA library https://github.com/adafruit/WiFiNINA  
Adafruit MQTT library https://github.com/adafruit/Adafruit_MQTT_Library  
