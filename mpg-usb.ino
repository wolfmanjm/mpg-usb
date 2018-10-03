#include <Encoder.h>

#include "Arduino.h"

//#define DEBUG 1

// Pins used
#define LE_ENCA  2 // D2 encoder pins
#define LE_ENCB  3 // D3

#define X_AXIS   4 // D4
#define Y_AXIS   5 // D5
#define Z_AXIS   7 // D7
#define A_AXIS   8 // D8

#define MULT_1   9 // D9
#define MULT_10  10 // D10
#define MULT_100 11 // D11

const int ledPin = 6;

//extern volatile uint8_t usb_configuration;

/*
    This uses raw hid to send packets when the wheel is turned.
    It sends the multiplier which tells the host how much to move per event
    It sends the axis to move
    and it sends the encoder wheel data...
    one byte tells the speed 1 to 100 where 100 is the fastest speed 1 is the slowest
    one byte is the number of ticks the wheel moved since the last packet in case we miss some (it is probably one per packet though) it is plus or minus demending on the direction
*/

int lst_e = 0;
unsigned long last_time = 0;
unsigned long maxtime= 100000; // after this we presume it is very slow
unsigned long mintime= 4000; // fastest we can move is about 1ms
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

    pinMode(ledPin, OUTPUT);

    last_time = micros();
#ifdef DEBUG
    Serial.begin (115200);   // debugging
    Serial.print("Starting...\r\n");
#endif
}  // end of setup

// main loop
void loop (void)
{
    byte axis= 0;
    if(digitalReadFast(X_AXIS) == LOW) {
        axis= 1;
    } else if(digitalReadFast(Y_AXIS) == LOW) {
        axis= 2;
    } else if(digitalReadFast(Z_AXIS) == LOW) {
        axis= 3;
    } else if(digitalReadFast(A_AXIS) == LOW) {
        axis= 4;
    } else {
        #ifdef DEBUG
        Serial.print(mintime); Serial.print(", ");
        Serial.print(maxtime); Serial.println("");
        #endif
        enc.write(0);
        lst_e= 0;
        return;
    }

    byte mult = 1;
    if(digitalReadFast(MULT_1) == LOW) {
        mult = 1;
    } else if(digitalReadFast(MULT_10) == LOW) {
        mult = 10;
    } else if(digitalReadFast(MULT_100) == LOW) {
        mult = 100;
    }

    int e = enc.read();
    // we get the delta since the last read
    if(e != lst_e) {
        int d = e - lst_e;

        if(abs(d) >= 4) { // needs to be one full detent
            // get time since last read so we can gauge the speed
            unsigned long now = micros();
            unsigned long delta_us = now - last_time;
            last_time = now;

            lst_e = e;
            char s = d / 4; // get number of ticks (each detent is 4 ticks)
            unsigned long t= (delta_us * abs(s));
            byte speed;
            if(t > maxtime) {
                speed= 0;
            } else if(t <= mintime) {
                speed= 10;
            } else {
                speed= 50000 / t;
            }

            // send hid packet
            buffer[0]= 0x12;
            buffer[1]= 0x34;
            buffer[2]= axis;
            buffer[3]= mult;
            buffer[4]= s;
            buffer[5]= speed;
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
    }
}


