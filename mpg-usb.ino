#include <Encoder.h>

#include "Arduino.h"

//#define DEBUG 1


#if defined(__AVR_ATmega32U4__)
#define BOARD "Teensy 2.0"
#define TEENSY2
#elif defined(__AVR_AT90USB1286__)       
#define BOARD "Teensy++ 2.0"
#define TEENSY2PP
#elif defined(__MKL26Z64__)       
#define BOARD "Teensy LC"
#define TEENSYLC
#else
#error "Unsupported board"
#endif

/*
#elif defined(__MK20DX128__)       
        #define BOARD "Teensy 3.0"
#elif defined(__MK20DX256__)       
        #define BOARD "Teensy 3.2" // and Teensy 3.1 (obsolete)
#elif defined(__MK64FX512__)
        #define BOARD "Teensy 3.5"
#elif defined(__MK66FX1M0__)
        #define BOARD "Teensy 3.6"
*/

#ifdef TEENSY2PP
const int ledPin = 6;
#elif defined(TEENSY2)
const int ledPin = 11;
#elif defined(TEENSYLC)
const int ledPin = 13;
#endif


// Pins used
#ifdef TEENSY2PP
#define LE_ENCA  2 // D2 encoder pins
#define LE_ENCB  3 // D3

#define X_AXIS   4 // D4
#define Y_AXIS   5 // D5
#define Z_AXIS   7 // D7
#define A_AXIS   8 // D8

#define MULT_1   9 // D9
#define MULT_10  10 // D10
#define MULT_100 12 // D12
#elif defined(TEENSY2)
#define LE_ENCA  5 // D0 encoder pins
#define LE_ENCB  6 // D1

#define X_AXIS   7  // D2
#define Y_AXIS   8  // D3
#define Z_AXIS   9  // C6
#define A_AXIS   10 // C7

#define MULT_1   2 // B2
#define MULT_10  3 // B3
#define MULT_100 4 // B7
#define LED      1 // B1
#define E_STOP    0 // B0
#endif

//extern volatile uint8_t usb_configuration;

/*
    This uses raw hid to send packets when the wheel is turned.
    It sends the multiplier which tells the host how much to move per event
    It sends the axis to move
    and it sends the encoder wheel data...
    one byte tells the speed 1 to 10 where 10 is the fastest speed 1 is the slowest
    one byte is the number of ticks the wheel moved since the last packet in case we miss some (it is probably one per packet though) it is plus or minus depending on the direction
*/

int lst_e = 0;
unsigned long last_time = 0;
unsigned long maxtime= 100000; // after this we presume it is very slow
unsigned long mintime= 4000; // fastest we can move is about 1ms
byte last_estop= 0;
byte buffer[64];

Encoder enc(LE_ENCA, LE_ENCB);
typedef unsigned char byte;

void setup (void)
{

    pinMode(MULT_1, INPUT_PULLUP);
    pinMode(MULT_10, INPUT_PULLUP);
    pinMode(MULT_100, INPUT_PULLUP);

    pinMode(X_AXIS, INPUT_PULLUP);
    pinMode(Y_AXIS, INPUT_PULLUP);
    pinMode(Z_AXIS, INPUT_PULLUP);
	pinMode(A_AXIS, INPUT_PULLUP);
	
	pinMode(E_STOP, INPUT_PULLUP);
	last_estop= digitalReadFast(E_STOP) == LOW ? 0 : 1;

#ifdef LED
	// Pendant led
	pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
#endif
	// Teensy led
	pinMode(ledPin, OUTPUT);
	digitalWrite(ledPin, HIGH);

    last_time = micros();
#ifdef DEBUG
    Serial.begin (115200);   // debugging
    Serial.println("Starting...\r\n");
#endif
}  // end of setup

bool change= false;
char s= 0;
byte speed= 0;
byte axis= 0;
byte estop= 0;
byte mult= 0;
unsigned long t= 0;

// main loop
void loop (void)
{
	if(change) {
		change= false;
		// send hid packet
		buffer[0]= 0x12;
		buffer[1]= 0x34;
		buffer[2]= axis;
		buffer[3]= mult;
		buffer[4]= s;
		buffer[5]= speed;
		buffer[6]= estop;
		// debug
		buffer[7]= t>>24;
		buffer[8]= t>>16;
		buffer[9]= t>>8;
		buffer[10]= t&0xFF;

		int n = RawHID.send(buffer, 0);
		if(n > 0) {
			digitalWrite(ledPin, HIGH);
		}else{
			digitalWrite(ledPin, LOW);
		}
	}
	
	// normally low and goes high when latched
	estop= digitalReadFast(E_STOP) == LOW ? 0 : 1;
	if(last_estop != estop) {
		last_estop= estop;
		change= true;
		return;
	}

	if(estop == 1) {
		// Pendant led
		digitalWrite(LED, LOW);
		return;
	}
	
	axis= 0;
    if(digitalReadFast(X_AXIS) == LOW) {
        axis= 1;
    } else if(digitalReadFast(Y_AXIS) == LOW) {
        axis= 2;
    } else if(digitalReadFast(Z_AXIS) == LOW) {
        axis= 3;
    } else if(digitalReadFast(A_AXIS) == LOW) {
        axis= 4;
    } else {
        #if 0
        Serial.print(mintime); Serial.print(", ");
        Serial.print(maxtime); Serial.println("");
        #endif
        // Pendant led
        digitalWrite(LED, LOW);
        enc.write(0);
        lst_e= 0;
        return;
    }

    // Pendant led
    digitalWrite(LED, HIGH);

    mult = 1;
    if(digitalReadFast(MULT_1) == LOW) {
        mult = 1;
    } else if(digitalReadFast(MULT_10) == LOW) {
        mult = 10;
    } else if(digitalReadFast(MULT_100) == LOW) {
        mult = 100;
	}
	
    int e = enc.read();
#ifdef DEBUG
    Serial.print(e); Serial.print(", ");
    Serial.print(axis); Serial.print(", ");
    Serial.print(mult); Serial.println("");
#endif
	// we get the delta since the last read
    if(e != lst_e) {
        int d = e - lst_e;

        if(abs(d) >= 4) { // needs to be one full detent
            // get time since last read so we can gauge the speed
            unsigned long now = micros();
            unsigned long delta_us = now - last_time;
            last_time = now;

            lst_e = e;
            s = d / 4; // get number of ticks (each detent is 4 ticks)
            t= (delta_us * abs(s));
            if(t > maxtime) {
                speed= 0;
            } else if(t <= mintime) {
                speed= 10;
            } else {
                speed= 50000 / t;
            }
			change= true;
        }
	}
}
