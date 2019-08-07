/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("Blinker");
  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chThdSleepMilliseconds(100);
  }
}


volatile uint8_t flag;
void adc_cb(ADCDriver *adcp, adcsample_t *buffer, size_t n)
{
    flag = 1;
}

/*
 * Application entry point.
 */
#define NBR_CHANNELS 3
#define DEPTH 5
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);

  /*
   * 
   */
  ADCConfig cfg = {ANALOG_REFERENCE_AVCC};
  ADCConversionGroup group = {0, NBR_CHANNELS, adc_cb, 0x7};
  adcsample_t buffer[DEPTH*NBR_CHANNELS];

  adcStart(&ADCD1, &cfg);
  
  /*
   * Starts the LED blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while(TRUE) {
    adcStartConversion(&ADCD1, &group, buffer, DEPTH);
    while(!flag)
        ;
    flag = 0;
    for(int i=0; i<DEPTH; i++)
        chprintf((BaseSequentialStream *)&SD1, "%d %d %d\n", buffer[i*NBR_CHANNELS],
                 buffer[i*NBR_CHANNELS + 1],
                 buffer[i*NBR_CHANNELS + 2]);
    chThdSleepMilliseconds(500);
  }
}
