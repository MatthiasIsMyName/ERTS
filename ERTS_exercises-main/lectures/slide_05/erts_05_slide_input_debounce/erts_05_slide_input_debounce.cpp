#include "erts_05_slide_input_debounce.h"

//Uses an Arduino Mega with input interrupt
//to show the button HW debounce with low pass filter (schmitt trigger)

//some useful links:
//- https://forum.arduino.cc/t/hardware-debounce-schmitt-trigger-and-input_pullup/570543

//HW:
//- 1x push button
//- 2x 10 kOhm resistors (Widerstand)
//- 1x 100 nF capacitor (Kondensator)

//prototype
void isr();

void setup()
{
    //init serial interface
    Serial.begin(9600);

    //I/O Pin config for debug with logic analyzer
    pinMode(9, OUTPUT);

    //I/O Pin config for INPUT and interrupt
    pinMode(3, INPUT); //pin with HW debounce
    attachInterrupt(digitalPinToInterrupt(3), isr, RISING);
}

void loop()
{
}

void isr()
{
    #define MASK_FOR_PIN_9 (1 << PH6) //digital pin 9 is on PORTH bit 6 -> 0b01000000
    PORTH ^= MASK_FOR_PIN_9; //toggle pin

    Serial.println("pressed...");
}
