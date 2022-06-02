// Do not remove the include below
#include "erts_05_slide_timer_watchdog.h"

#include <avr/wdt.h> //wdt_enable, WDTO_2S

//Uses an Arduino Mega with a watchdog: restart on timeout

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
    Serial.println("Restart...");

    //I/O Pin config for debug with logic analyzer
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH); //initially set high

    //I/O Pin config for INPUT and interrupt
    pinMode(3, INPUT); //pin with HW debounce
    attachInterrupt(digitalPinToInterrupt(3), isr, RISING);

    //enable watchdog
    wdt_enable(WDTO_2S); //2s timeout time
}

void loop()
{
}

void isr()
{
    Serial.println("feed the dog...");
    wdt_reset(); //fee the dog
}
