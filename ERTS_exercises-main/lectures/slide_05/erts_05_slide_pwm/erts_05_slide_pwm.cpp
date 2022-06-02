#include "erts_05_slide_pwm.h"


void setup()
{
    pinMode(9, OUTPUT); //HW timer based PWM pin (timer 2)
    pinMode(10, OUTPUT); //SW based PWM
    pinMode(11, OUTPUT); //HW timer based PWM pin (timer 1: OC1A)

    analogWrite(9, 64); //pwm -> 25% duty cycle

    //T_P (period) = 490 Hz -> 0.002040816 sec -> 002.040816 milli sec -> 2040.816 micro sec
    //25% => 2040.816 micro sec / 4 -> 510.204 micro sec

    { //configure timer
        noInterrupts();

        //stop/reconfigure timer
        TCCR1A = 0;
        TCCR1B = 0;
        TCCR1C = 0;

        //reset counter
        TCNT1 = 0;

        //set prescaler
        TCCR1B |= (0 << CS12) | (1 << CS11)| (1 << CS10);
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

        OCR1A = 64;          //compare and match value for output compare register A of timer 4
                               //0.0625us * 1024 = 64 us
        //ICR1 = 32640;

        //TCCR4B |= 1 << WGM12;  //CTC mode (clear timer counter): set TCNT4 = 0 after TCNT4 reached OCR4A
        //TIMSK4 |= 1 << OCIE4A; //enable "Output Compare A Match Interrupt Enable"


        //clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
        //table 17-4: page 155
        TCCR1A |= 2 << COM1A0;

        //fast pwm: table 17-2; page 145
        //TCCR1A |= (1 << WGM11) | (0 << WGM10);
        //TCCR1B |= (1 << WGM13) | (1 << WGM12);

        //fast pwm: table 17-2; page 145: Fast PWM, 8-bit: TOP 0x00FF, BOTTOM: OCRnX
        //TCCR1A |= (0 << WGM11) | (1 << WGM10);
        //TCCR1B |= (0 << WGM13) | (1 << WGM12);

        //fast pwm: table 17-2; page 145: Fast PWM, 8-bit: TOP 0x00FF, BOTTOM: OCRnX
        //TCCR1A |= (1 << WGM11) | (0 << WGM10);
        //TCCR1B |= (0 << WGM13) | (1 << WGM12);

        //phase correct PWM: 8-bit, TOP: 0x00FF, BOTTOM: OCRnX
        TCCR1A |= (0 << WGM11) | (1 << WGM10);
        TCCR1B |= (0 << WGM13) | (0 << WGM12);

        interrupts();
    }
}

//TODO: configure timer 4 directly for a custom period and duty cycle

void loop()
{
    //SW PWM (manually):
    digitalWrite(10, HIGH);
    delayMicroseconds(510); //busy wait for: 510 micro seconds

    digitalWrite(10, LOW);
    delayMicroseconds(1530); //busy wait for: 1530 micro seconds (=2040 - 510)
}
