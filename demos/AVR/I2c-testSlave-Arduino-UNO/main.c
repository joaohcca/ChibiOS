/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the Lice/os/hal/lib/streams/chprintf.se is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"

#include "chprintf.h"

#include <stdio.h>

#include <string.h>
#include <stdlib.h>


#include "hal_i2c.h"
#include <hal_i2cslave.h>
/*
 * Ithread, times are in milliseconds.
 */


static THD_WORKING_AREA(waThread1, 32);
static THD_FUNCTION(Thread1, arg) {
  i2caddr_t slaveaddr=5;
  size_t txbytes=5;
  size_t rxbytes=5;
  uint8_t rxbuf[rxbytes];
  uint8_t txbuffer[txbytes];
  sysinterval_t TIMEOUT=500;
  msg_t debug;
  int errors;
  (void)arg;
  chRegSetThreadName("SlaveRecieveI2C");
  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chprintf((BaseSequentialStream *) &SD1, "iniciando processo de recebimento do slave\r\n");  
    chThdSleepMilliseconds(500);
    /*configurar endereço do slave e "encaixar" o matchaddress*/
    debug = i2cMatchAddress(&I2CD1, slaveaddr);    
    chprintf((BaseSequentialStream *) &SD1, "debug = %d\r\n",debug);
    errors=i2cGetErrors(&I2CD1);  
    chprintf((BaseSequentialStream *) &SD1, "errors = %d\r\n",errors);

    i2cSlaveReceive(&I2CD1, slaveaddr, txbuffer, txbytes, rxbuf, rxbytes, TIMEOUT); 
    //i2cSlaveReply(&I2CD1, slaveaddr, txbuffer, txbytes, rxbuf, rxbytes, TIMEOUT); 
    //chThdSleepMilliseconds(2000); //estudar valor real e ver se deve ser empirico esse resultado
    // chprintf((BaseSequentialStream *) &SD1, "debug = %d\r\n",debug);
    //conversão do valor de debug‘i2cSlaveReceive’
      chprintf((BaseSequentialStream *) &SD1, "final da execucao da thread\r\n");
      for (int n = 0; n < txbytes ;n++){
        chprintf( (BaseSequentialStream *) &SD1, "buffer[%n]=%d ",n,rxbuf[n]);
        }
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
   * Activates the serial driv/os/hal/lib/streams/chprintf.er 1 using the driver default configuration.
   */
  sdStart(&SD1, NULL);
   
  



  /*chnWrite
   * Starts the MasterSendI2C thread
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  
  


  chnWrite(&SD1, (const uint8_t *)"Debug program I2Cv2 slave\r\n", 14);

  while (TRUE) {
    chThdSleepMilliseconds(1000);  
  }
}