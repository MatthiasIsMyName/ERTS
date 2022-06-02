#include "erts_09_slide_freertos_pp.h"

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>

//Arduino HW def
const uint8_t LED_PIN = LED_BUILTIN;
const uint8_t BUTTON_PIN = 2;

//FreeRTOS definitions
SemaphoreHandle_t log_semaphore = NULL;

SemaphoreHandle_t ping_semaphore = NULL;
SemaphoreHandle_t pong_semaphore = NULL;

TaskHandle_t ping_task_handle = NULL;
TaskHandle_t pong_task_handle = NULL;

//prototypes
void ping_task(void* pvParameters);
void pong_task(void* pvParameters);
void log_message(const char* message);

void setup() {
  //configure serial port
  Serial.begin(9600);

  //configure HW
  pinMode(LED_PIN, OUTPUT);

  //create semaphore
  log_semaphore = xSemaphoreCreateBinary();
  configASSERT(log_semaphore != NULL);
  xSemaphoreGive(log_semaphore);

  ping_semaphore = xSemaphoreCreateBinary();
  configASSERT(ping_semaphore != NULL);

  //let the ping task start first
  xSemaphoreGive(ping_semaphore);

  pong_semaphore = xSemaphoreCreateBinary();
  configASSERT(pong_semaphore != NULL);

  //start message
  log_message("\r\n\r\nStart...\r\n");

  //create tasks
  BaseType_t result = pdFALSE;
  result = xTaskCreate(
    ping_task,          //address of task function
    "ping_task",        //name of task
    128L,               //task stack size in words: 128*2=256 bytes (configSTACK_DEPTH_TYPE)
    NULL,               //task parameters
    16,                 //task priority (2-55)
    &ping_task_handle   //pointer to task handle
  );

  configASSERT(result == pdPASS);         //assert that the task could be created
  configASSERT(ping_task_handle != NULL); //assert that task handle is valid

  result = xTaskCreate(
    pong_task,          //address of task function
    "pong_task",        //name of task
    128L,               //task stack size in words: 128*2=256 bytes (configSTACK_DEPTH_TYPE)
    NULL,               //task parameters
    16,                 //task priority (2-55)
    &pong_task_handle   //pointer to task handle
  );

  configASSERT(result == pdPASS);         //assert that the task could be created
  configASSERT(pong_task_handle != NULL); //assert that task handle is valid
}

void loop()
{

}

void log_message(const char* message)
{
  xSemaphoreTake(log_semaphore, portMAX_DELAY);

  Serial.println(message);

  xSemaphoreGive(log_semaphore);
}

void ping_task(void* pvParameters) {
  log_message("ping_task started...");
  vTaskDelay(0);

  while (true) {
    //wait infinitely for semaphore
    xSemaphoreTake(ping_semaphore, portMAX_DELAY);

    log_message("ping...");
    vTaskDelay(pdMS_TO_TICKS(1000));

    xSemaphoreGive(pong_semaphore);
  }
}

void pong_task(void* pvParameters) {
  log_message("pong_task started...");
  vTaskDelay(0);

  while (true) {
    //wait infinitely for semaphore
    xSemaphoreTake(pong_semaphore, portMAX_DELAY);

    log_message("       ...pong");
    vTaskDelay(pdMS_TO_TICKS(1000));

    xSemaphoreGive(ping_semaphore);
  }
}
