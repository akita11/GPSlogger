#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include <TinyGPSPlus.h>

#define LOG_FILENAME "/log.csv"

#define PIN_POW_SD A3
#define PIN_POW_GPS A4
#define PIN_SD_CS 4
#define PIN_LED 5

File fp;
SdFat sd;
TinyGPSPlus gps;

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

const char *gpsStream =
		"$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
		"$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
		"$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
		"$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
		"$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
		"$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

void setup()
{
	Serial.begin(9600); // from GPS
	pinMode(PIN_POW_SD, OUTPUT);
	digitalWrite(PIN_POW_SD, 1);
	pinMode(PIN_POW_GPS, OUTPUT);
	digitalWrite(PIN_POW_GPS, 1);
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, 0);
	PowSD(1);
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
	fp.close();

	while (*gpsStream)
	{
		if (gps.encode(*gpsStream++))
		{
			Serial.print(F("Location: "));
			if (gps.location.isValid())
			{
				Serial.print(gps.location.lat(), 6);
				Serial.print(F(","));
				Serial.print(gps.location.lng(), 6);
			}

			Serial.print(F("  Date/Time: "));
			if (gps.date.isValid())
			{
				Serial.print(gps.date.month());
				Serial.print(F("/"));
				Serial.print(gps.date.day());
				Serial.print(F("/"));
				Serial.print(gps.date.year());
			}

			Serial.print(F(" "));
			if (gps.time.isValid())
			{
				if (gps.time.hour() < 10)
					Serial.print(F("0"));
				Serial.print(gps.time.hour());
				Serial.print(F(":"));
				if (gps.time.minute() < 10)
					Serial.print(F("0"));
				Serial.print(gps.time.minute());
				Serial.print(F(":"));
				if (gps.time.second() < 10)
					Serial.print(F("0"));
				Serial.print(gps.time.second());
				Serial.print(F("."));
				if (gps.time.centisecond() < 10)
					Serial.print(F("0"));
				Serial.print(gps.time.centisecond());
			}
		}
	}
}

void loop()
{
	while (Serial.available() > 0)
	{
		char c = Serial.read();
		gps.encode(c);
		if (gps.location.isUpdated())
		{
			Serial.print("Lat=\t");
			Serial.print(gps.location.lat(), 6);
			Serial.print(" Lng=\t");
			Serial.println(gps.location.lng(), 6);
		}
	}
}
