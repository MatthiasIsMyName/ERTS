#include "erts_09_slide_freertos_timer.h"

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <timers.h>

//Arduino HW def
const uint8_t LED_PIN = LED_BUILTIN;

//FreeRTOS definitions
TimerHandle_t timer = NULL;
int timerId = 1; //optional: id for timer

//prototypes
void timerCallback(TimerHandle_t xTimer);
void log_message(const char* message);

void setup() {
  //configure serial port
  Serial.begin(9600);
  log_message("\r\n\r\nStart...\r\n");

  //configure HW
  pinMode(LED_PIN, OUTPUT);

  //create timer
  timer = xTimerCreate(
    "SW Timer",                //name of timer
    pdMS_TO_TICKS(1000),       //period in ticks
    pdTRUE,                    //true=auto reload, false=one shot
    &timerId,                  //address timer id variable
    timerCallback              //address of timer callback function
  );

  configASSERT(timer != NULL); //assert that timer handle is valid

  //start timer
  xTimerStart(timer, 100);     //starts the timer in 100 ticks
}


void loop() {

}

void log_message(const char* message) {
  Serial.println(message);
}

void timerCallback(TimerHandle_t xTimer) {
  //work...

  log_message("worker_task: toggle led");
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  Serial.println(*(int*)pvTimerGetTimerID(xTimer));
}

