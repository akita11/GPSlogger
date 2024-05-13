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
	if (val == 1){
		digitalWrite(PIN_POW_GPS, 0);
		UCSR0B |= _BV(TXEN0) | _BV(RXEN0); // enable UART0
		pinMode(0, OUTPUT); // RXD for GPS
	}
	else{
		digitalWrite(PIN_POW_GPS, 1);
		UCSR0B &= ~(_BV(TXEN0) | _BV(RXEN0)); // enable UART0
		digitalWrite(1, 0); // TXD for GPS = 0
		pinMode(0, INPUT);
		digitalWrite(0, 0); // RXD for GPS = 0
	}
}

volatile uint16_t cnt = 0;
ISR(WDT_vect)
{
	cnt++;
}

void setup()
{
	Serial.begin(9600); // from GPS
	ss.begin(115200); // for debug console
	pinMode(PIN_POW_SD, OUTPUT); pinMode(PIN_POW_GPS, OUTPUT);
	pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, 0);
	for (uint8_t i = 0; i < 3; i++){ digitalWrite(PIN_LED, 1); delay(100); digitalWrite(PIN_LED, 0); delay(100); }
  	cli();
 	MCUSR = 0;
  	WDTCSR |= 0b00011000; // set WDCE & WDE
  	WDTCSR =  0b01000000 | 0b00100000; // enable WDT interrupt, cycle = 4s
  	sei();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set sleep mode
	cnt = 0;
	PowSD(1); PowGPS(0);
}

// GPS on   : 38mA
// SD write :

uint16_t cc = 0;
void loop()
{
//	for (uint8_t i = 0; i < cnt; i++){ digitalWrite(PIN_LED, 1); delay(100); digitalWrite(PIN_LED, 0); delay(100); }
	digitalWrite(PIN_LED, 1); delay(10); digitalWrite(PIN_LED, 0);
	ss.println(cnt);
	if (cnt == 1){
		PowGPS(1);
		digitalWrite(PIN_LED, 1); 
		uint8_t fin = 0;
		while(fin == 0){
			while (Serial.available() > 0)
			{
				char c = Serial.read();
				ss.write(c);
				gps.encode(c);
				cc++;
			}
/*
			if (cc > 100){
				cc = 0;
				ss.print('[');
				ss.print(gps.location.isValid());
				ss.print(gps.date.isValid());
				ss.print(']');
			}
*/
//			if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()){
			if (gps.location.isValid()){
				fin = 1;
			}
		}
		digitalWrite(PIN_LED, 0);
		PowGPS(0);
		ss.print("Lat="); ss.print(gps.location.lat(), 6);
		ss.print(" Lng="); ss.println(gps.location.lng(), 6);
		ss.print(gps.date.year()); ss.print(F("/")); ss.print(gps.date.month()); ss.print(F("/")); ss.print(gps.date.day()); ss.print(' ');
		ss.print(gps.time.hour()); ss.print(F(":")); ss.print(gps.time.minute()); ss.print(F(":")); ss.println(gps.time.second());
/*
		ss.print("initializing SD card...");
		PowSD(1);
		delay(1000);
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
		ss.print("write to SD card...");
		fp = sd.open(LOG_FILENAME, O_WRONLY | O_APPEND);
		fp.print(gps.date.year()); fp.print(gps.date.month()); fp.print(gps.date.day());
		fp.print(',');
		fp.print(gps.time.hour()); fp.print(gps.time.minute()); fp.print(gps.time.second());
		fp.print(',');
		fp.print(gps.location.lat(), 6);
		fp.print(',');
		fp.println(gps.location.lng(), 6);
		fp.close();
		ss.println("done");
		delay(1000);
		PowSD(0);
*/
		cnt = 0;
	}
	sleep_mode();
}
