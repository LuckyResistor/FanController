//
// Lucky Resistor's Fan Controller
// ---------------------------------------------------------------------------
// (c)2016 by Lucky Resistor. See LICENSE for details.
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//

#include <Adafruit_MCP23017.h>
#include <Adafruit_RGBLCDShield.h>
#include <DHT.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>

#include "PIDController.h"


// Constants
const uint8_t cFanControlPin = 5;
const uint8_t cTemperatureSensor1Pin = 2;
const uint8_t cTemperatureSensor2Pin = 4;
const uint8_t cSdCardChipSelectPin = 10;
const uint16_t cUpdateDelay = 2200; // Update every ~2.2 seconds.

// Control constants
const uint8_t cControlMinimumTemperature = 33;
const uint8_t cControlMinimumFanSpeed = 0xa0;
const uint8_t cControlMaximumTemperature = 38;
const uint8_t cControlMaximumFanSpeed = 0xff;


// The instance to access the LCD display.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// The two temperature sensors
DHT temperatureSensor1(cTemperatureSensor1Pin, DHT22);
DHT temperatureSensor2(cTemperatureSensor2Pin, DHT22);

// The real time clock
RTC_DS1307 rtc;


// The current fan speed.
uint8_t fanSpeed = 0xa0;

// The current temperatures.
float temperature1 = 0;
float temperature2 = 0;

// The PID algorithm
lr::PIDController pidController;


// Setup everything.
void setup() {
  // Initialize the serial output.
  Serial.begin(57600);
  while (!Serial) { _NOP(); }
  Serial.println(F("Starting Log Control"));
  
  // Setup the LCD display.
  lcd.begin(16, 2);
  lcd.setBacklight(0x7);

  // Setup the temperature sensors
  temperatureSensor1.begin();
  temperatureSensor2.begin();

  // Setup the PID controller.
  pidController.begin(2.0f, 1.0f, 0.5f);
  pidController.setOutputLimits(0.0f, 255.0f);
  pidController.setOutputReverse(true);
  pidController.setOutputDrift(-0.05f);
  pidController.setOutputValue(fanSpeed);
  pidController.setTargetValue(32.0f);

  // Initialize the SD card library
  if (!SD.begin(cSdCardChipSelectPin)) {
    Serial.println(F("Initialize SD card failed."));
    lcd.print("SD Card Error!");
    delay(5000);
    lcd.clear();
  }

  // Set the pin for the FAN control.
  pinMode(cFanControlPin, OUTPUT);

  // Write some welcome message.
  lcd.print(F(" Fan Controller"));
  lcd.setCursor(0, 1);
  lcd.print(F("    Welcome!"));
  delay(2000);
  lcd.clear();
}


// Print a decimal number padded with an optional zero to the LCD.
void lcdPrintPaddedDecimal(uint8_t value) {
  if (value < 10) {
    lcd.print('0');
  }
  lcd.print(value, DEC);
}


// All measurements are done every 10 seconds.
void loop() {
  // Get the current time
  DateTime now = rtc.now();
  
  // First measure the current temperatures.
  temperature1 = temperatureSensor1.readTemperature();
  temperature2 = temperatureSensor2.readTemperature();

  // Calculate the new fan speed.
  const float maximumTemperature = max(temperature1, temperature2);
  fanSpeed = static_cast<uint8_t>(pidController.calculateOutput(maximumTemperature));
  analogWrite(cFanControlPin, fanSpeed);
  
  // Build the display
  lcd.setCursor(0, 0);
  lcd.print(F("1:"));
  lcd.print(static_cast<uint8_t>(round(temperature1)), DEC);
  lcd.print('\xdf');
  lcd.print('C');
  lcd.print(' ');
  lcd.print(F("2:"));
  lcd.print(static_cast<uint8_t>(round(temperature2)), DEC);
  lcd.print('\xdf');
  lcd.print('C');
  lcd.print(' ');
  lcd.print('F');
  lcd.print(static_cast<char>('0' + (fanSpeed/26)));
  lcd.setCursor(0, 1);
  lcd.print(now.year(), DEC);
  lcd.print('-');
  lcdPrintPaddedDecimal(now.month());
  lcd.print('-');
  lcdPrintPaddedDecimal(now.day());
  lcd.print(' ');
  lcdPrintPaddedDecimal(now.hour());
  lcd.print(':');
  lcdPrintPaddedDecimal(now.minute());

  writeLogEntry(now, temperature1, temperature2, fanSpeed);

  // Wait 5 seconds for the next measurement.
  delay(cUpdateDelay);
}


// Write a single log entry.
void writeLogEntry(const DateTime &time, const float temperature1, const float temperature2, uint8_t fanSpeed) {
  // Generate the log file name with the date and time.
  char *fileName = new char[32]; // temperature_yyyy-mm-dd.log
  snprintf(fileName, 32, "td%02d%02d%02d.log", (time.year() % 100), time.month(), time.day());
  // Open the file on the sd card.
  File logFile = SD.open(fileName, FILE_WRITE);
  delete fileName;
  // Generate the log line.
  char line[80]; // yyyy-mm-ddThh:mm:ss,30.76,25.09,128
  snprintf(
    line,
    80,
    "%04d-%02d-%02dT%02d:%02d:%02d,%d.%02d,%d.%02d,%d",
    time.year(),
    time.month(),
    time.day(),
    time.hour(),
    time.minute(),
    time.second(),
    static_cast<int>(temperature1),
    static_cast<int>(temperature1*100.0f)%100,
    static_cast<int>(temperature2),
    static_cast<int>(temperature2*100.0f)%100,
    static_cast<int>(fanSpeed));
  // Check if the file could be opened for writing.
  if (logFile) {
    // Write it into the file.
    logFile.println(line);
    logFile.close();
  } else {
    // Display a ! character to signal a problem with log writing.
    lcd.setCursor(13, 0);
    lcd.print('!');
  }
  // Send it to serial.
  Serial.println(line);
}






