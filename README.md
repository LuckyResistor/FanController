# Fan Controller

This is the software for the fan controller project. It is written for the following hardware and software configuration:

- Arduino IDE Version 1.6.9 (or newer)
    - Using the Adafruit RGB LCD shield library version 1.0.1.
    - Using the Adafruit DHT sensor library version 1.2.3.
    - Using the Adafruit RTClib library version 1.2.0.
    - Using the built-in SD library by Arduino, SparkFun version 1.0.8.
- The hardware is based on Arduino Uno.
    - On top the the Arduino there is the Adafruit Data Logger shield.
    - On top of that is the Adafruit LCD shield with a 16x2 display.
- The hardware is powered using a 12V power supply.
- There is a PC fan with PWM connected to pin 5.
- There are two DHT22 temperature sensors connected to pin 2 and 4.
- The SD card is accessed using SPI and pin 10 for chip select.
- The RTC is accessed using I2C.

Read all details about the required hardware here:

http://luckyresistor.me

Select "Fan Controller" from the projects menu.

