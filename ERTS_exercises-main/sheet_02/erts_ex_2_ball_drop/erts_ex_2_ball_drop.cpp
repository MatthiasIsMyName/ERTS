#include "erts_ex_2_ball_drop.h"

//TODO: Modify FreeRTOSConfig.h
//- configMAX_PRIORITIES     -> set to an appropriate value
//- configMINIMAL_STACK_SIZE -> set to 256L

//Attention: At least FreeRTOS version V10.4.6 is required

//enable assertion
#define configASSERT 1

//includes
#include <TimerFive.h>
#include <Arduino_FreeRTOS.h> //FreeRTOS
#include <queue.h>            //FreeRTOS queues
#include <semphr.h>           //FreeRTOS semaphores
#include <task.h>             //FreeRTOS tasks

//pin definitions
const int MOTOR_PWM_PIN    = 5;  //PWM control { jumpers 3 | 5 | 6 } for motor A…
const int MOTOR_DIR_PIN    = 4;  //direction control { 2 | 4 | 7 } for motor A…
const int MAGNET_PWM_PIN   = 9;  //PWM control { 9 | 10 | 11 } for motor B…
const int MAGNET_DIR_PIN   = 8;  //direction control { 8 | 12 | 13 } for motor B…

const int BUTTON_PIN       = 3;  //push button pin
const int LIGHT_SENSOR_PIN = 2;  //light sensor pin
const int POTI_PIN         = A0; //analog input pin for potentiometer

//other defines
const unsigned long DEBOUNCE_DELAY_MS = 200;
const short number_of_states = 3;
const unsigned long required_angle = 278;
const unsigned long drop_time_ms = 359;
const unsigned long full_circle_angle = 360;

//prototypes
void log_task(void* pvParameters);
void log_message(const char* message);
void log_message_ISR(const char* message);
void motor_control_task(void* pvParameters);
void trigger_motor();
void button_ISR();
void light_sensor_ISR();
void drop_ball_task(void* pvParameters);

//variables
volatile int rotation_time_ms = 0;           //measured rotation time of disc in ms
volatile int32_t wait_time_drop_ball_ms = 0; //calculated wait time to drop disc in ms
volatile unsigned long last_button_press_time_ms = 0;
volatile unsigned long last_light_sensor_detected_ms = 0;
volatile byte disk_turns = 0;

enum STATES {
    RUNNING = 0, //disc is rotating, magnet onDEBOUNCE_DELAY
    DROPPING,    //disc is rotating, magnet is released after x rotations
    STOPPED      //disc stopped, magnet off
};
volatile int control_state = RUNNING; //controls the state of ball drop system


//task handles
TaskHandle_t log_task_handle = nullptr;
TaskHandle_t motor_control_task_handle = nullptr;
TaskHandle_t drop_ball_task_handle = nullptr;
//queue handles
QueueHandle_t log_queue = NULL; // @suppress("Type cannot be resolved")

#define MAX_LOG_LENGTH (50)
typedef char log_message_t[MAX_LOG_LENGTH];

//semaphore handles
SemaphoreHandle_t motor_timer_semaphore = NULL;
SemaphoreHandle_t ball_drop_semaphore = NULL;

void setup() {
    //init serial connection
    Serial.begin(9600);
    Serial.println("\r\n\r\n(re)start...\r\n");

    //configure I/O debug pin
    pinMode(LED_BUILTIN, OUTPUT);

    //configure I/O pins
    pinMode(LIGHT_SENSOR_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(POTI_PIN, INPUT);

    //configure motor and magnet driver pins
    pinMode(MOTOR_PWM_PIN, OUTPUT);
    pinMode(MOTOR_DIR_PIN, OUTPUT);
    pinMode(MAGNET_PWM_PIN, OUTPUT);
    pinMode(MAGNET_DIR_PIN, OUTPUT);

    //motor control: initial -> motor off
    digitalWrite(MOTOR_DIR_PIN, HIGH); //direction
    analogWrite(MOTOR_PWM_PIN, 0);     //speed through PWM value (0 = stopped)

    //magnet control: initial - magnet on
    digitalWrite(MAGNET_DIR_PIN, HIGH); //direction
    digitalWrite(MAGNET_PWM_PIN, HIGH); //magnet on

    //create queues
    log_queue = xQueueCreate(8, sizeof(log_message_t));
    configASSERT(log_queue != NULL);

    //create semaphore(s)
    motor_timer_semaphore = xSemaphoreCreateBinary();
    configASSERT(motor_timer_semaphore != NULL);

    ball_drop_semaphore = xSemaphoreCreateBinary();
    configASSERT(ball_drop_semaphore != NULL);


    //create task(s)
    BaseType_t result = xTaskCreate(
        log_task,                  //address of task function
        "log_task",                //name of task
        256L,                      //task stack size in words: 256*2=512 bytes (configSTACK_DEPTH_TYPE)
        nullptr,                   //task parameters
        10,                        //task priority (2-16)
        &log_task_handle           //pointer to task handle
    );
    configASSERT(result == pdPASS);           //assert that the task could be created
    configASSERT(log_task_handle != nullptr); //assert that task handle is valid

    BaseType_t motor_control_task_result = xTaskCreate(
		motor_control_task,                  //address of task function
		"motor_control_task",                //name of task
		256L,                      //task stack size in words: 256*2=512 bytes (configSTACK_DEPTH_TYPE)
		nullptr,                   //task parameters
		12,                        //task priority (2-16)
		&motor_control_task_handle           //pointer to task handle
	);
	configASSERT(motor_control_task_result == pdPASS);           //assert that the task could be created
	configASSERT(motor_control_task_handle != nullptr); //assert that task handle is valid

	BaseType_t drop_ball_task_result = xTaskCreate(
		drop_ball_task,                  //address of task function
		"drop_ball_task",                //name of task
		256L,                      //task stack size in words: 256*2=512 bytes (configSTACK_DEPTH_TYPE)
		nullptr,                   //task parameters
		15,                        //task priority (2-16)
		&drop_ball_task_handle           //pointer to task handle
	);
	configASSERT(drop_ball_task_result == pdPASS);           //assert that the task could be created
	configASSERT(drop_ball_task_handle != nullptr); //assert that task handle is valid

    //button ISR
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN),button_ISR, RISING);

    //light sensor ISR
    attachInterrupt(digitalPinToInterrupt(LIGHT_SENSOR_PIN), light_sensor_ISR, RISING);

    //start timer motor_control
    Timer5.initialize(100000);
    Timer5.attachInterrupt(trigger_motor);
}

//idle task
void loop() {

}

void log_task(void* pvParameters) {
    Serial.println("start log task");

    while (true) {
        log_message_t received_log_message;
        BaseType_t result = xQueueReceive(log_queue, (void*)&received_log_message, portMAX_DELAY);

        if(result == pdPASS){
            //debug information
            Serial.println(received_log_message);
            Serial.flush();
        } else {
            Serial.println("error: log_task: no item received");
            Serial.flush();
        }
    }
}

void log_message(const char* message) {
    log_message_t log_message;
    strncpy(log_message, message, MAX_LOG_LENGTH);

    xQueueSend(log_queue, &log_message, portMAX_DELAY);
}

void log_message_ISR(const char* message) {
    log_message_t log_message;
    strncpy(log_message, message, MAX_LOG_LENGTH);

    BaseType_t taskChangeRequired = pdFALSE;
    BaseType_t result = xQueueSendToBackFromISR(log_queue, &log_message, &taskChangeRequired);
    if(result == errQUEUE_FULL){
        Serial.print("error: log_message_ISR: queue full\r\n");
        Serial.flush();
    }

    //to immediately run the scheduler (change task)
    if (taskChangeRequired == pdTRUE) {
        taskYIELD();
    }
}

//your ISRs and tasks...
void button_ISR(){
	unsigned long current_button_press_time_ms = millis();
	if(last_button_press_time_ms == 0 || current_button_press_time_ms - last_button_press_time_ms > DEBOUNCE_DELAY_MS){
		last_button_press_time_ms = current_button_press_time_ms;
		control_state = (control_state + 1) % number_of_states;
		if(control_state == RUNNING){
			digitalWrite(MAGNET_PWM_PIN, HIGH);
		}
	}
}

void light_sensor_ISR(){
	unsigned long current_light_sensor_detected_ms = millis();
	rotation_time_ms = current_light_sensor_detected_ms - last_light_sensor_detected_ms;
	last_light_sensor_detected_ms = current_light_sensor_detected_ms;

	wait_time_drop_ball_ms = ( ( rotation_time_ms * required_angle ) / full_circle_angle ) - drop_time_ms;
	while(wait_time_drop_ball_ms < 0){
		wait_time_drop_ball_ms += rotation_time_ms;
	}

	if(control_state == DROPPING){
		if(disk_turns == 2){
			disk_turns = 0;
			BaseType_t taskChangeRequired = pdFALSE;
			xSemaphoreGiveFromISR(ball_drop_semaphore, &taskChangeRequired);
			//to immediately run the scheduler (change task)
			if (taskChangeRequired == pdTRUE) {
			taskYIELD();
			}
		}
		else{
			disk_turns++;
		}
//		log_message_ISR(String(disk_turns).c_str());
	}

}

void trigger_motor(){
	BaseType_t taskChangeRequired = pdFALSE;
	xSemaphoreGiveFromISR(motor_timer_semaphore, &taskChangeRequired);
	//to immediately run the scheduler (change task)
	if (taskChangeRequired == pdTRUE) {
	taskYIELD();
	}
}

void motor_control_task(void* pvParameters){
	while(true){
		//wait infinitely for semaphore
		xSemaphoreTake(motor_timer_semaphore, portMAX_DELAY);
		if(control_state == STOPPED){
			analogWrite(MOTOR_PWM_PIN, 0);
		}
		else{
			analogWrite(MOTOR_PWM_PIN, analogRead(POTI_PIN));
		}
	}
}

void drop_ball_task(void* pvParameters){
	while(true){
		//wait infinitely for semaphore
		xSemaphoreTake(ball_drop_semaphore, portMAX_DELAY);
		delay(wait_time_drop_ball_ms);
		digitalWrite(MAGNET_PWM_PIN, LOW);
	}
}

/***************************************************************************/
// FreeRTOS extended error handlers
// The error handlers print some information to the Serial for a
// proper error handling.

void vApplicationAssertHook()
{
    Serial.println("");
    Serial.println("error: assertion failed");
    Serial.flush();

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Arduino Mega with 2560
    DDRB  |= _BV(DDB7);
    PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Seeed Goldilocks with 1284p
    DDRB  |= _BV(DDB7);
    PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
    DDRB  |= _BV(DDB5);
    PORTB |= _BV(PORTB5);       // Main (red PB5) LED on. Main LED on.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
    DDRC  |= _BV(DDC7);
    PORTC |= _BV(PORTC7);       // Main (red PC7) LED on. Main LED on.

#endif

    for(;;)
    {
        _delay_ms(2000);

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Arduino Mega with 2560
        PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Seeed Goldilocks with 1284p
        PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
        PINB  |= _BV(PINB5);       // Main (red PB5) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
        PINC  |= _BV(PINC7);       // Main (red PC7) LED toggle. Main LED slow blink.

#endif

    }
}

void vApplicationMallocFailedHook(void)
{
    Serial.println("");
    Serial.println("error: malloc failed");
    Serial.flush();

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // Arduino Mega with 2560
    DDRB  |= _BV(DDB7);
    PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Seeed Goldilocks with 1284p
    DDRB  |= _BV(DDB7);
    PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
    DDRB  |= _BV(DDB5);
    PORTB |= _BV(PORTB5);       // Main (red PB5) LED on. Main LED on.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
    DDRC  |= _BV(DDC7);
    PORTC |= _BV(PORTC7);       // Main (red PC7) LED on. Main LED on.

#endif

    for(;;)
    {
        _delay_ms(50);

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Mega with 2560
        PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED fast blink.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Seeed Goldilocks with 1284p
        PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED fast blink.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
        PINB  |= _BV(PINB5);       // Main (red PB5) LED toggle. Main LED fast blink.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
        PINC  |= _BV(PINC7);       // Main (red PC7) LED toggle. Main LED fast blink.

#endif

    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    Serial.println("");
    Serial.println("error: stack overflow detected on");
    Serial.print("- task: ");
    Serial.println(pcTaskName);
    Serial.flush();

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Arduino Mega with 2560
    DDRB  |= _BV(DDB7);
    PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Seeed Goldilocks with 1284p
    DDRB  |= _BV(DDB7);
    PORTB |= _BV(PORTB7);       // Main (red PB7) LED on. Main LED on.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
    DDRB  |= _BV(DDB5);
    PORTB |= _BV(PORTB5);       // Main (red PB5) LED on. Main LED on.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
    DDRC  |= _BV(DDC7);
    PORTC |= _BV(PORTC7);       // Main (red PC7) LED on. Main LED on.

#endif

    for(;;)
    {
        _delay_ms(2000);

#if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)  // Arduino Mega with 2560
        PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Seeed Goldilocks with 1284p
        PINB  |= _BV(PINB7);       // Main (red PB7) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino Uno with 328p
        PINB  |= _BV(PINB5);       // Main (red PB5) LED toggle. Main LED slow blink.

#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
        PINC  |= _BV(PINC7);       // Main (red PC7) LED toggle. Main LED slow blink.

#endif

    }
}
