#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include <SoftwareSerial.h>
#include <avr/wdt.h> 
#include <avr/sleep.h>

#define LOG_FILENAME "log.csv"

#define PIN_POW_SD A0 // for v1
#define PIN_POW_GPS A4 // for v1
#define PIN_SD_CS 4
#define PIN_LED A1 // for v1

#define LEN_LINE 128
char line[LEN_LINE];

//float lng, lat;
char lng[20], lat[20];
char lng_c, lat_c;
char  dt[8], tm[8];
//float height;
char height[20];

uint8_t NMEAparse(char *line, uint8_t type)
{
	// for type 0:
	// $GNRMC,,V,,,,,,,,,,M*4E
	// $GNRMC,002154.000,V,3632.61109,N,13642.31699,E,0.13,0.00,,,,A*6A
	// $GNRMC,002220.000,A,3632.64273,N,13642.30496,E,0.00,0.00,220524,,,A*7B

	// for type 1:
	// $GPGGA,052400.00,3539.3146239,N,13945.6411751,E,4,07,0.59,4.987,M,34.035,M,1.0,3403*76
	uint8_t p = 0;
	uint8_t f = 0;
	char buf[64];
	uint8_t pb = 0;
	uint8_t fValid = 0;
	while(p < strlen(line)){
		char c = line[p];
		if (c == ','){
			buf[pb] = '\0';
			if (type == 0){
				if (f == 1){ buf[6] = '\0'; strcpy(tm, buf);}
//				if (f == 3) lng = atof(buf);
				if (f == 3){ strcpy(lng, buf);}
				if (f == 4) lng_c = buf[0];
//				if (f == 5) lat = atof(buf);
				if (f == 5){ strcpy(lat, buf);}
				if (f == 6) lat_c = buf[0];
				if (f == 9){ strcpy(dt, buf);}
			}
			else if (type == 1){
//				if (f == 9) height = atof(buf);
				if (f == 9){ strcpy(height, buf);}
			}
			pb = 0;
			f++;
		}
		else buf[pb++] = c;
		if (type == 0 && f == 2 && c == 'A') fValid = 1;
		p++;
	}
	return(fValid);
	}

File fp;
SdFat sd;
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

volatile uint16_t cnt;
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
//	WDTCSR =  0b01000000 | 0b00100000; // enable WDT interrupt, cycle = 4s
	WDTCSR =  0b01000000 | 0b00100001; // enable WDT interrupt, cycle = 8s
	sei();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set sleep mode

	Serial.println("start"); delay(1000);
	cnt = 0;
	DDRC = 0xff; // A0-A5 as OUTPUT
	PORTC = 0x11; // A0&A4 = 1, others=0
	DDRB = 0xff; // 8-13 as OUTPUT
	PORTB = 0x00; // 8-13 = 0
	DDRD = 0xfe; // 1-7 as OUTPUT, 1 as INPUT 
	PORTD = 0x00; // 0-7 = 0
	ACSR |= _BV(ACD); // disable analog comparator
	ADCSRA = 0x00; // disable ADC

	PowSD(0); PowGPS(0);
	cnt = 10298;
}

// GPS on   : 35mA
// SD write : 30mA?
// sleep		: 0.1mA -> 0.01mA (CMP&ADC off)

#define N_TRIAL_MAX 300

void loop()
{
	digitalWrite(PIN_LED, 1); delay(10); digitalWrite(PIN_LED, 0);
//	Serial.println(cnt); delay(1000);
//	ss.println(cnt);
//	if (cnt == 10){ // 8s * 10 = 80s
	if (cnt == 10300){ // 8.192s * 10000 = 81920s = 22.76h
//		ss.println("start logging");
//		Serial.println("start logging"); delay(1000);
		PowGPS(1);
		digitalWrite(PIN_LED, 1); 
		uint8_t fin = 0;
		uint8_t pBuf = 0;
		uint16_t nTrial = 0;
		while(fin == 0 && nTrial < N_TRIAL_MAX){
			while(Serial.available() > 0 && pBuf < LEN_LINE){
				char c = Serial.read();
				line[pBuf++] = c;
				if (c == '\r' || c == '\n'){
					line[pBuf] = '\0';
					if (pBuf > 10){
						if (line[3] == 'R' && line[4] == 'M' && line[5] == 'C'){
							digitalWrite(PIN_LED, 1 - digitalRead(PIN_LED)); 
//							ss.println('*');
//							ss.println(line);
							fin = NMEAparse(line, 0);
							nTrial++;
						}
						else if (line[3] == 'G' && line[4] == 'G' && line[5] == 'A'){
							NMEAparse(line, 1);
						}
					}
					pBuf = 0;
				}
			}
		}
		ss.println("done");
		digitalWrite(PIN_LED, 0);
		ss.print(lng); ss.print(' '); ss.print(lng_c); ss.print(' ');
		ss.print(lat); ss.print(' '); ss.print(lat_c); ss.print(' ');
		ss.print(dt); ss.print(' '); ss.println(tm);
		PowGPS(0);
		if (nTrial < N_TRIAL_MAX){
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
			fp = sd.open(LOG_FILENAME, FILE_WRITE);
			ss.println(fp);
			fp.print(dt); fp.print(',');
			fp.print(tm); fp.print(',');
			fp.print(lat); fp.print(',');	fp.print(lat_c); fp.print(',');
			fp.print(lng); fp.print(','); fp.print(lng_c); fp.print(',');
			fp.print(height); fp.print(',');
			fp.println(nTrial); 
			fp.close();
			ss.println("done");
			delay(1000);
			PowSD(0);
		}
		cnt = 0;
	}
	sleep_mode();
}
