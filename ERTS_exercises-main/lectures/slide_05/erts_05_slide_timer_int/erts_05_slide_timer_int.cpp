#include "erts_05_slide_timer_int.h"

//Uses an Arduino Mega in "alarm mode" -> generates an interrupt after a
//specified time (number of clock cycles)

//This example uses timer 4
//Details:
// - 16 bit counter register: TNCT4
// - 3x 16 bit output compare and match register: OCR4A (is used here), OCR4B, OCR4C
// - clock of timer 4: sys clock -> 16 MHz
// - possible prescalers: 1/8/64/256/1024 on sys clock

//some useful links:
//- https://oscarliang.com/arduino-timer-and-interrupt-tutorial/
//- https://forum.arduino.cc/t/what-timers-and-ressources-use-analogwrite/395028/4
//- https://www.robotshop.com/community/forum/t/arduino-101-timers-and-interrupts/13072
//- https://www.arduino.cc/en/Hacking/PinMapping2560

void setup()
{
    //init serial interface
    Serial.begin(9600);

    //I/O Pin config for debug with logic analyzer
    pinMode(9, OUTPUT);

    { //configure timer
        noInterrupts();

        //stop/reconfigure timer
        TCCR4A = 0;
        TCCR4B = 0;
        TCCR4C = 0;

        //reset counter
        TCNT4 = 0;

        //set prescaler
        TCCR4B |= (0 << CS42) | (0 << CS41)| (1 << CS40);
                               // prescaler, here: 1
                               // possible prescalers: (1/8/64/256/1024)
                               // 0b000 -> clock/timer off
                               // 0b001 -> sys clock -> prescaler 1
                               // 0b010 -> sys clock -> prescaler 8
                               // 0b011 -> sys clock -> prescaler 64
                               // 0b100 -> sys clock -> prescaler 256
                               // 0b101 -> sys clock -> prescaler 1024
                               //-> sys clock = 16 MHz
                               //-> one tick: 1/16 MHz -> 0.0625us

        OCR4A = 1024;          //compare and match value for output compare register A of timer 4
                               //0.0625us * 1024 = 64 us

        TCCR4B |= 1 << WGM12;  //CTC mode (clear timer counter): set TCNT4 = 0 after TCNT4 reached OCR4A
        TIMSK4 |= 1 << OCIE4A; //enable "Output Compare A Match Interrupt Enable"

        interrupts();
    }
}

void loop()
{

}

//ISR is called, when TCNT4 (timer counter) reaches OCR4A (compare and match value)
//-> ISR is called every 64us
ISR(TIMER4_COMPA_vect)
{
    #define MASK_FOR_PIN_9 (1 << PH6) //digital pin 9 is on PORTH bit 6 -> 0b01000000

    PORTH ^= MASK_FOR_PIN_9; //toggle pin

    //stop/reconfigure timer (for one shot mode)
//	TCCR4A = 0;
//	TCCR4B = 0;
//	TCCR4C = 0;
}
