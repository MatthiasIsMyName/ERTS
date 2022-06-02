// Do not remove the include below
#include "erts_09_slide_freertos_tn.h"

#include <Arduino_FreeRTOS.h>
#include <task.h>

//Arduino HW def
const uint8_t LED_PIN = LED_BUILTIN;
const uint8_t BUTTON_PIN = 2;

//FreeRTOS definitions
TaskHandle_t deferred_isr_task_handle = NULL;

//prototypes
void deferred_isr_task(void* pvParameters);
void log_message(const char* message);
void button_isr();

void setup()
{
  //configure serial port
  Serial.begin(9600);
  log_message("\r\n\r\nStart...\r\n");

  //configure HW
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, RISING);

  //create semaphore
  configASSERT(worker_semaphore != NULL);

  //create task
  BaseType_t result = xTaskCreate(
    deferred_isr_task,  //address of task function
    "worker_task",      //name of task
    128L,               //task stack size in words: 128*2=256 bytes (configSTACK_DEPTH_TYPE)
    NULL,               //task parameters
    16,                 //task priority (2-55)
    &deferred_isr_task_handle //pointer to task handle
  );

  configASSERT(result == pdPASS);                 //assert that the task could be created
  configASSERT(deferred_isr_task_handle != NULL); //assert that task handle is valid
}

void loop()
{

}

void log_message(const char* message)
{
  Serial.println(message);
}

void deferred_isr_task(void* pvParameters) {
  log_message("worker_task started...");

  while(true) {
    log_message("worker_task sleep...");
    //wait infinitely for a task notification
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    log_message("worker_task wake up...");

    for (uint8_t i = 0; i < 10; ++i) {
      log_message("worker_task: toggle led");
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

bool button_debounce() {
  static unsigned long previous_ms = 0;
  const unsigned long DEBOUNCE_DELAY_MS = 1000; //ms

  unsigned long current_ms = millis();
  if((current_ms - previous_ms) < DEBOUNCE_DELAY_MS) {
    return false;
  } else {
    previous_ms = current_ms;
    return true;
  }
}

void button_isr() {
  if(!button_debounce()){
    return;
  }

  log_message("ISR: task notify");

  BaseType_t taskChangeRequired = pdFALSE;
  vTaskNotifyGiveFromISR(deferred_isr_task_handle,
                         &taskChangeRequired);

  //to immediately run the scheduler (change task)
  if (taskChangeRequired == pdTRUE) {
    taskYIELD();
  }
}
