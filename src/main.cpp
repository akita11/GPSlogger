#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
//#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>

#define LOG_FILENAME "/log.csv"

//#define PIN_POW_SD A3 // for v09
//#define PIN_POW_GPS A4 // for v09
#define PIN_POW_SD A0 // for v1
#define PIN_POW_GPS A4 // for v1
#define PIN_SD_CS 4
#define PIN_LED A1 // for v1
//#define PIN_LED 5 // for v09
//#define PIN_LED 13 // for v1

#define LEN_LINE 128
char line[LEN_LINE];

float lng, lat;
char lng_c, lat_c;
uint16_t tm;

uint8_t NMEAparse(char *line)
{
  uint8_t p = 0;
  uint8_t f = 0;
  char buf[64];
  uint8_t pb = 0;
  uint8_t fValid = 0;
  while(p < strlen(line)){
    char c = line[p];
    if (c == ','){
      buf[pb] = '\0';
      if (f == 3) lng = atof(buf);
      if (f == 4) lng_c = buf[0];
      if (f == 5) lat = atof(buf);
      if (f == 6) lat_c = buf[0];
      if (f == 9) tm = atoi(buf);
      pb = 0;
      f++;
    }
    else buf[pb++] = c;
    if (f == 2 && c == 'A') fValid = 1;
    p++;
  }
  return(fValid);
}

File fp;
SdFat sd;
//TinyGPSPlus gps;
//SoftwareSerial ss(A1, A2); // RX, TX // for v09
SoftwareSerial ss(A2, 10); // RX, TX

void PowSD(uint8_t val)
{
	if (val == 1){
		digitalWrite(PIN_POW_SD, 0);
		digitalWrite(PIN_SD_CS, 1);
		SPCR |= _BV(SPE); // enable SPI
	}
	else{
		digitalWrite(PIN_POW_SD, 1);
		digitalWrite(PIN_SD_CS, 0);
		SPCR &= ~(_BV(SPE)); // disable SPI
		PORTB &= ~(_BV(PB3) | _BV(PB4) | _BV(PB5)); // MOSI, SCK, SS = 0
	}
}

void PowGPS(uint8_t val)
{
	if (val == 1){
		digitalWrite(PIN_POW_GPS, 0);
		pinMode(0, INPUT);
		UCSR0B |= (_BV(TXEN0) | _BV(RXEN0)); // enable UART0
	}
	else{
		digitalWrite(PIN_POW_GPS, 1);
		UCSR0B &= ~(_BV(TXEN0) | _BV(RXEN0)); // disable UART0
		PORTD = 0x00; // 0-7 = 0 (TXD,RXD=0)
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

	// flash LED at boot
	for (uint8_t i = 0; i < 3; i++){ digitalWrite(PIN_LED, 1); delay(100); digitalWrite(PIN_LED, 0); delay(100); }
  	cli();
 	MCUSR = 0;
  	WDTCSR |= 0b00011000; // set WDCE & WDE
  	WDTCSR =  0b01000000 | 0b00100000; // enable WDT interrupt, cycle = 4s
  	sei();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set sleep mode
	cnt = 0;

	DDRC = 0xff; // A0-A5 as OUTPUT
	PORTC = 0x00; // A0-A5 = 0
	DDRB = 0xff; // 8-13 as OUTPUT
	PORTB = 0x00; // 8-13 = 0
	DDRD = 0xff; // 0-7 as OUTPUT 
	PORTD = 0x00; // 0-7 = 0

	PowSD(0); PowGPS(0);
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
		uint8_t pBuf = 0;
		while(fin == 0){
			while(Serial.available() > 0 && pBuf < LEN_LINE){
				char c = Serial.read();
				if (c == '\r'){
	 				line[pBuf] = '\0';
// $GNRMC,,V,,,,,,,,,,M*4E
// $GNRMC,002154.000,V,3632.61109,N,13642.31699,E,0.13,0.00,,,,A*6A
// $GNRMC,002220.000,A,3632.64273,N,13642.30496,E,0.00,0.00,220524,,,A*7B
					if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C'){
						ss.print('*');
						ss.println(line);
						ss.println(NMEAparse(line));
					}
					ss.println(line);
					pBuf = 0;
				}
				line[pBuf++] = c;
			}
/*
			while (Serial.available() > 0)
			{
				char c = Serial.read();
				ss.write(c);
				gps.encode(c);
				cc++;
			}
			while (Serial.available() > 0)
			{
				char c = Serial.read();
				ss.write(c);
				gps.encode(c);
				cc++;
			}
			if (cc > 100){
				cc = 0;
				ss.print('[');
				ss.print(gps.location.isValid());
				ss.print(gps.date.isValid());
				ss.print(']');
			}
//			if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()){
			if (gps.location.isValid()){
//				fin = 1;
			}
*/
		}
		digitalWrite(PIN_LED, 0);
		PowGPS(0);
//		ss.print("Lat="); ss.print(gps.location.lat(), 6);
//		ss.print(" Lng="); ss.println(gps.location.lng(), 6);
//		ss.print(gps.date.year()); ss.print(F("/")); ss.print(gps.date.month()); ss.print(F("/")); ss.print(gps.date.day()); ss.print(' ');
//		ss.print(gps.time.hour()); ss.print(F(":")); ss.print(gps.time.minute()); ss.print(F(":")); ss.println(gps.time.second());
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
