#include "erts_05_slide_timer_measure.h"

//Uses an Arduino Mega in "stop mode" -> measure timer ticks (time)

//This example uses timer 4
//Details:
// - 16 bit counter register: TNCT4
// - 3x 16 bit output compare and match register: OCR4A (is used here), OCR4B, OCR4C
// - clock of timer 4: sys clock -> 16 MHz
// - possible prescalers: 1/8/64/256/1024 on sys clock

//Attention:
//before read/write TCNT4, the interrupts should be disabled, because TCNT4 is 16 bit
//and the ATmega2560 is only a 8 bit processor. See micros() function for an example.

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

        interrupts();
    }
}

//using timer 4 directly
void loop()
{
    uint16_t counter = 0;

    { //functional part that should be measured
        #define MASK_FOR_PIN_9 (1 << PH6) //digital pin 9 is on PORTH bit 6 -> 0b01000000
        TCNT4 = 0;                  //reset timer counter
        PORTH |= MASK_FOR_PIN_9;    //pin 9 on //digitalWrite(9, HIGH);

        delayMicroseconds(10);      //simulate some heavy work

        counter = TCNT4;            //get timer counter
        PORTH &= ~MASK_FOR_PIN_9;   //pin 9 off //digitalWrite(9, LOW);
    }

    //float micro_sec = 0.0625 * counter; //0.0625 = 1/(16*10^6)
    uint16_t micro_sec = counter/16; //counter/(16*10^6) -> s; counter/(16) -> us

    Serial.print("Counter: ");
    Serial.println(counter);
    Serial.print("micro sec: ");
    Serial.println(micro_sec);
    delay(1000);
}

//using Arduino micros() -> uses internally timer 0
//void loop()
//{
//    uint16_t micro_sec = 0;
//
//    { //functional part that should be measured
//        #define MASK_FOR_PIN_9 (1 << PH6) //digital pin 9 is on PORTH bit 6 -> 0b01000000
//        micro_sec = micros();
//        PORTH |= MASK_FOR_PIN_9;    //pin 9 on //digitalWrite(9, HIGH);
//
//        delayMicroseconds(10);      //simulate some heavy work
//
//        micro_sec = micros() - micro_sec;
//        PORTH &= ~MASK_FOR_PIN_9;   //pin 9 off //digitalWrite(9, LOW);
//    }
//
//    Serial.print("micro sec: ");
//    Serial.println(micro_sec);
//    delay(1000);
//}
