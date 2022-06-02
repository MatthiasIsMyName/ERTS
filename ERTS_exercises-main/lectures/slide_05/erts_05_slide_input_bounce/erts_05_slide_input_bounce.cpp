#include "erts_05_slide_input_bounce.h"

//Uses an Arduino Mega with input interrupt
//to show the button bounce problem

//HW:
//- 1x push button (connected to ground, because of INPUT_PULLUP usage)

//prototype
void isr();

void setup()
{
    //init serial interface
    Serial.begin(9600);

    //I/O Pin config for debug with logic analyzer
    pinMode(9, OUTPUT);

    //I/O Pin config for INPUT and interrupt
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), isr, RISING);
}

void loop()
{

}

//ISR is called, when button (PIN 2) is pressed
void isr()
{
    #define MASK_FOR_PIN_9 (1 << PH6) //digital pin 9 is on PORTH bit 6 -> 0b01000000
    PORTH ^= MASK_FOR_PIN_9; //toggle pin

    Serial.println("pressed...");
}
