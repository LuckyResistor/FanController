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
const uint8_t cFanControlPin = 5; // The pin where the fan PWM is connected.
const uint8_t cTemperatureSensor1Pin = 2; // The pin for the first temperature sensor data.
const uint8_t cTemperatureSensor2Pin = 4; // The pin for the second temperature sensor data.
const uint8_t cSdCardChipSelectPin = 10; // The pin where the SD card chip select is connected.
const uint16_t cUpdateDelay = 2200; // The update every ~2.2 seconds.
const uint8_t cStartFanSpeed = 0x80; // The initial fan speed.
const uint8_t cControlTargetTemperature = 31; // The maximum target temperature.

// Custom characters for the LCD display.
const uint8_t cFanCharacterMask[] PROGMEM = {
  0b01100,
  0b01100,
  0b00101,
  0b11011,
  0b11011,
  0b10100,
  0b00110,
  0b00110,
};
const char cFanCharacter = '\x05';
const uint8_t cTemp1CharacterMask[] PROGMEM = {
  0b01000,
  0b10100,
  0b10100,
  0b10100,
  0b10100,
  0b11101,
  0b11101,
  0b01000
};
const char cTemp1Character = '\x06';
const uint8_t cTemp2CharacterMask[] PROGMEM = {
  0b01000,
  0b10101,
  0b10101,
  0b10100,
  0b10100,
  0b11101,
  0b11101,
  0b01000
};
const char cTemp2Character = '\x07';


// The instance to access the LCD display.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// The two temperature sensors
DHT temperatureSensor1(cTemperatureSensor1Pin, DHT22);
DHT temperatureSensor2(cTemperatureSensor2Pin, DHT22);

// The real time clock
RTC_DS1307 rtc;


// The current fan speed.
uint8_t fanSpeed = cStartFanSpeed;

// The PID controller instance.
lr::PIDController pidController;


/// Setup everything.
///
void setup() {
  // Set the pin for the fan PWM control.
  pinMode(cFanControlPin, OUTPUT);
  analogWrite(cFanControlPin, 0x00);

  // Initialize the serial output.
  Serial.begin(115200);
  while (!Serial) { _NOP(); } // Wait until it is ready.
  Serial.println(F("Starting Fan Controller"));
  
  // Setup the LCD display.
  lcd.begin(16, 2);
  lcd.setBacklight(0x7);
  // Add custom characters for the fan as bar display.
  uint8_t bars[] = {B10000, B11000, B11100, B11110, B11111};
  uint8_t character[8];
  for (uint8_t i = 0; i < 5; ++i) {
    for (uint8_t j = 0; j < 8; ++j) character[j] = bars[i];
    lcd.createChar(i, character);
  }
  // Add custom characters from the masks.
  memcpy_P(character, cFanCharacterMask, 8);
  lcd.createChar(cFanCharacter, character);
  memcpy_P(character, cTemp1CharacterMask, 8);
  lcd.createChar(cTemp1Character, character);
  memcpy_P(character, cTemp2CharacterMask, 8);
  lcd.createChar(cTemp2Character, character);
  
  // Setup the temperature sensors
  temperatureSensor1.begin();
  temperatureSensor2.begin();

  // Setup the PID controller.
  pidController.begin(2.0f, 1.0f, 0.5f);
  pidController.setOutputLimits(0.0f, 255.0f);
  pidController.setOutputReverse(true);
  pidController.setOutputDrift(-0.05f);
  pidController.setOutputValue(fanSpeed);
  pidController.setTargetValue(30.0f);

  // Initialize the SD card library
  if (!SD.begin(cSdCardChipSelectPin)) {
    Serial.println(F("Initialize SD card failed."));
    lcd.print("SD Card Error!");
    delay(5000);
    lcd.clear();
  }

  // Write some welcome message and wait 2 seconds.
  lcd.print(F(" Fan Controller"));
  lcd.setCursor(0, 1);
  lcd.print(F("    Welcome!"));
  delay(2000);
  lcd.clear();
}


/// Print a decimal number padded with an optional zero to the LCD.
///
/// @param value The value to display.
///
void lcdPrintPaddedDecimal(uint8_t value) {
  if (value < 10) {
    lcd.print('0');
  }
  lcd.print(value, DEC);
}


/// A special loop for manual control mode.
///
void manualLoop() {
  uint8_t displayUpdateDelay = 0; // A delay to keep the update delay.
  bool manualDisplay = false; // A flag to switch between the "manual" message and the date/time
  uint8_t lastButtonState = 0; // The last button state for comparision
  // Loop until "break".
  while (true) {
    // Get the current button state.
    const uint8_t currentButtonState = lcd.readButtons();
    // Check which buttons got a press.
    const uint8_t pressedButtons = (lastButtonState ^ currentButtonState) & currentButtonState;
    // If left increase the fan speed.
    if (pressedButtons & BUTTON_LEFT) {
      if (fanSpeed < 0x10) {
        fanSpeed = 0;
      } else {
        fanSpeed -= 0x10;
      }
      analogWrite(cFanControlPin, fanSpeed);
      displayUpdateDelay = 0; // force display update.
    } else if (pressedButtons & BUTTON_RIGHT) { // if right decrease the fan speed.
      if (fanSpeed >= 0xf0) {
        fanSpeed = 0xff;
      } else {
        fanSpeed += 0x10;
      }
      analogWrite(cFanControlPin, fanSpeed);
      displayUpdateDelay = 0; // force display update.
    } else if (pressedButtons & BUTTON_DOWN) { // if down, exit this loop.
      pidController.setOutputValue(fanSpeed); // Make sure the PID controller knows the new fan speed.
      break;
    }
    lastButtonState = currentButtonState; // Store the current button state for the next loop.
    // Every few seconds, refresh the display.
    if (displayUpdateDelay == 0) {
        // Get the current time.
        DateTime now = rtc.now();
        // Measure the current temperatures.
        const float temperature1 = temperatureSensor1.readTemperature();
        const float temperature2 = temperatureSensor2.readTemperature();
        // Build the display.
        updateTemperatureDisplay(temperature1, temperature2);
        updateFanSpeedDisplay();
        // Switch between the "manual control" message and the date/time display.
        if (manualDisplay) {
          lcd.setCursor(0, 1);
          lcd.print(F(" Manual Control "));
        } else {
          updateTimeDisplay(now);
        }
        manualDisplay = !manualDisplay;
    }
    // Make sure the display is refreshed as specified in cUpdateDelay.
    if (++displayUpdateDelay >= (cUpdateDelay/100)) {
      displayUpdateDelay = 0;
    }
    delay(100);
  }
}


/// Update the temperature display
///
/// @param temperature1 The first temperature value in degree celsius.
/// @param temperature2 The second temperature value in degree celsius.
///
void updateTemperatureDisplay(float temperature1, float temperature2) {
  lcd.setCursor(0, 0);
  lcd.print(cTemp1Character);
  lcd.print(static_cast<uint8_t>(round(temperature1)), DEC);
  lcd.print('\xdf');
  lcd.print('C');
  lcd.print(' ');
  lcd.print(cTemp2Character);
  lcd.print(static_cast<uint8_t>(round(temperature2)), DEC);
  lcd.print('\xdf');
  lcd.print('C');
}


/// Update the time display.
///
/// @param now The time to display.
///
void updateTimeDisplay(const DateTime &now) {
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
}


/// Print a single bar character of the given value (0-5).
///
/// @param value The bar value to display (0-5).
///
void lcdPrintBar(const uint8_t value) {
  if (value == 0) {
    lcd.print(' ');
  } else {
    lcd.print(static_cast<char>(value-1));
  }
}


/// Update the fan speed display.
///
void updateFanSpeedDisplay() {
  lcd.setCursor(12, 0);
  lcd.print(cFanCharacter);
  const uint8_t displayedValue = (fanSpeed>>4);
  if (displayedValue > 10) {
    lcdPrintBar(5);
    lcdPrintBar(5);
    lcdPrintBar(displayedValue - 10);
  } else if (displayedValue > 5) {
    lcdPrintBar(5);
    lcdPrintBar(displayedValue - 5);
    lcdPrintBar(0);
  } else {
    lcdPrintBar(displayedValue);
    lcdPrintBar(0);
    lcdPrintBar(0);
  }
}


/// The main loop of the controller
///
void loop() {

  // Check if manual mode is requested.
  const uint8_t buttonState = lcd.readButtons();
  if (buttonState & BUTTON_UP) {
    manualLoop(); // go to manual loop, until left back to this loop.
  }

  // Get the current time from the RTC.
  DateTime now = rtc.now();

  // First measure the current temperatures.
  const float temperature1 = temperatureSensor1.readTemperature();
  const float temperature2 = temperatureSensor2.readTemperature();

  // Calculate the new fan speed using the PID controller.
  const float maximumTemperature = max(temperature1, temperature2);
  fanSpeed = static_cast<uint8_t>(pidController.calculateOutput(maximumTemperature));
  analogWrite(cFanControlPin, fanSpeed);
  
  // Refresh the display.
  updateTemperatureDisplay(temperature1, temperature2);
  updateFanSpeedDisplay();
  updateTimeDisplay(now);

  // Write all values into a log file on the SD card.
  writeLogEntry(now, temperature1, temperature2, fanSpeed);

  // Wait "cUpdateDelay" miliseconds for the next measurement.
  delay(cUpdateDelay);
}


/// Write a single log entry.
///
/// @param time The time to write.
/// @param temperature1 The first temperature to write.
/// @param temperature2 The second temperature to write.
/// @param fanSpeed The fan speed to write.
///
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
    // Display a ! character on the display to signal a problem with log writing.
    lcd.setCursor(11, 0);
    lcd.print('!');
  }
  // Send the same log line to the serial line.
  Serial.println(line);
}






