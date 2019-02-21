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

//adicionar as variaveis do Makefile conforme e-mail do prof Hermano
#include "chprintf.h"

#include <stdio.h>

#include <string.h>
#include <stdlib.h>


#include "hal_i2c.h"
//#include "hal_i2cslave.h"
/*
 * LED blinker thread, times are in milliseconds.
 */

#define BUFFERSIZE 5
static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {
  i2caddr_t slaveaddr=5;
  size_t txbytes=BUFFERSIZE;
  size_t rxbytes=BUFFERSIZE;
  uint8_t rxbuf[BUFFERSIZE];
  uint8_t txbuffer[BUFFERSIZE] = "1234";
  sysinterval_t TIMEOUT = 1000;
  msg_t debug;

  (void)arg;
  chRegSetThreadName("MasterSendI2C");
  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chprintf((BaseSequentialStream *) &SD1, "iniciando processo de envio do master\r\n");
    chThdSleepMilliseconds(500);
    debug=i2cMasterTransmitTimeout(&I2CD1, slaveaddr, txbuffer, txbytes, rxbuf, rxbytes, TIMEOUT);
    chThdSleepMilliseconds(500); //estudar valor real e ver se deve ser empirico esse resultado
    chprintf((BaseSequentialStream *) &SD1, "debug = %s\r\n",debug);
    chprintf((BaseSequentialStream *) &SD1, "final da execucao da thread\r\n");
      }
}



/*
 * Application entry point.
 */
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
  i2cInit();
  /*
   * Activates the serial driver 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);




  /*chnWrite
   * Starts the MasterSendI2C thread
   */
//  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  chnWrite(&SD1, (const uint8_t *)"Ler configuracoes do I2C do teclado\r\n",14);

chThdSleepMilliseconds(1000);
 chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
}
