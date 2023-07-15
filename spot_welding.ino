
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <timers.h>

#define DETECT_DELAY_TIMER_PERIOD pdMS_TO_TICKS(2000)

SemaphoreHandle_t interruptSemaphore;
TimerHandle_t xOneShotTimer;


const int fet_cntl =  10;     //Pin D10 of Arduino UNO
const int detect =  2;       //Pin D9 of Arduino UNO
const int cntl_pulse_len = 5; // 5ms

void setup() {

  pinMode(fet_cntl, OUTPUT);
  digitalWrite(fet_cntl, HIGH);
  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  Serial.println("Serial Opened Successfully");
  
  interruptSemaphore = xSemaphoreCreateBinary();
  if (interruptSemaphore != NULL) {
    // Attach interrupt for Arduino digital pin
    attachInterrupt(digitalPinToInterrupt(detect), interruptHandler, CHANGE);
  }
  
  xOneShotTimer = xTimerCreate(
                  "OneShot",
                  DETECT_DELAY_TIMER_PERIOD,
                  pdFALSE,
                  0,
                  prvOneShotTimerCallback);
  
  xTaskCreate(
        TaskDetect
        , "Detect" // A name just for humans
        , 128      // This stack size can be checked and adjusted by reading the Stack Highwater
        , NULL
        , 2        // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        , NULL );

}

void loop() {
}

void interruptHandler() {
  
  xSemaphoreGiveFromISR(interruptSemaphore, NULL);
}

void TaskDetect(void *pvParameters) // This is a task.
{
  (void) pvParameters;
  BaseType_t xTimer1Started;
  
  pinMode(detect, INPUT);

  for(;;)
  {
    if (xSemaphoreTake(interruptSemaphore, 0) == pdPASS) {
      if (digitalRead(detect) == HIGH)
      {
        //Start the timer
        Serial.println("Short is detected!!");
        xTimer1Started = xTimerStart(xOneShotTimer, 0);
      }
      else
      {
        //Stop the timer
        Serial.println("Open is detected!!");
        xTimer1Started = xTimerStop(xOneShotTimer, 0);
      }
    }

    vTaskDelay( pdMS_TO_TICKS(10) );
  }
}

static void prvOneShotTimerCallback( TimerHandle_t xTimer )
{
  TickType_t xTimeNow;
  /* Obtain the current tick count. */
  xTimeNow = xTaskGetTickCount();
  /* Output a string to show the time at which the callback was executed. */
  
  Serial.print("FET Control Enable");
  Serial.println(xTimeNow/31);
  digitalWrite(fet_cntl, LOW);
  delay(cntl_pulse_len);
  digitalWrite(fet_cntl, HIGH);
  Serial.println("FET Control Release");
}

/*
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    incomingByte = Serial.read();
    if (incomingByte == 'u')
    {
      Serial.println("Up");
      digitalWrite(fet_cntl, LOW);
      delay(loopPeriod);
      digitalWrite(fet_cntl, HIGH);
    }
    
    // say what you got:
    //Serial.print("PWM value is: ");
    //Serial.println(pwm_out, DEC);
  }
}
*/
