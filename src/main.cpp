#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>

#define LOG_FILENAME "/log.csv"

#define PIN_POW_SD A3
#define PIN_POW_GPS A4
#define PIN_SD_CS 4
#define PIN_LED 5

File fp;
SdFat sd;
TinyGPSPlus gps;
SoftwareSerial ss(A1, A2); // RX, TX

void PowSD(uint8_t val)
{
	if (val == 1)
		digitalWrite(PIN_POW_SD, 0);
	else
		digitalWrite(PIN_POW_SD, 1);
}

void PowGPS(uint8_t val)
{
	if (val == 1)
		digitalWrite(PIN_POW_GPS, 0);
	else
		digitalWrite(PIN_POW_GPS, 1);
}

ISR(WDT_vect)
{
}

void setup()
{
	Serial.begin(9600); // from GPS
	ss.begin(115200); // for debug console
	pinMode(PIN_POW_SD, OUTPUT); pinMode(PIN_POW_GPS, OUTPUT);
	PowSD(0); PowGPS(0);
	pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, 0);
	for (uint8_t i = 0; i < 3; i++){ digitalWrite(PIN_LED, 1); delay(100); digitalWrite(PIN_LED, 0); delay(100); }
/*
	if (!sd.begin(PIN_SD_CS, SD_SCK_MHZ(50)))
	{
		sd.initErrorPrint();
		while (1)
		{
			digitalWrite(PIN_LED, 1);
			delay(100);
			digitalWrite(PIN_LED, 0);
			delay(100);
		}
	}
	fp = sd.open(LOG_FILENAME, O_WRONLY | O_CREAT); // for SdFat.h
	fp.println("hogehoge");
	fp.println("upipi");
	fp.close();
*/
  	cli();
 	MCUSR = 0;
  	WDTCSR |= 0b00011000; // set WDCE & WDE
  	WDTCSR =  0b01000000 | 0b00100000; // enable WDT interrupt, cycle = 4s
  	sei();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set sleep mode

//	PowGPS(1);
}


void loop()
{
	digitalWrite(PIN_LED, 1); delay(500); digitalWrite(PIN_LED, 0);
	sleep_mode();
/*
	digitalWrite(PIN_LED, 1); 
	while (Serial.available() > 0)
	{
		char c = Serial.read();
		ss.write(c);
		gps.encode(c);
	}
	if (gps.location.isUpdated() && gps.date.isValid() && gps.time.isValid()){
		ss.println("Lat="); ss.print(gps.location.lat(), 6);
		ss.println(" Lng="); ss.println(gps.location.lng(), 6);
		ss.print(gps.date.year()); ss.print(F("/")); ss.print(gps.date.month()); ss.print(F("/")); ss.println(gps.date.day());
		ss.print(gps.time.hour()); ss.print(F(":")); ss.print(gps.time.minute()); ss.print(F(":")); ss.println(gps.time.second());
		digitalWrite(PIN_LED, 0);
		sleep_mode();
	}
*/
}
