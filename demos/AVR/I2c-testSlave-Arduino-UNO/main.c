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
  i2caddr_t slaveaddr=4;
  size_t txbytes=5;
  size_t rxbytes=5;
  uint8_t rxbuffer[rxbytes];
  uint8_t txbuffer[txbytes];
  sysinterval_t TIMEOUT=500;
  msg_t debug;
  int errors;
   for (uint8_t n = 0; n < txbytes ; n++){
        txbuffer[n]=n+10;
        chprintf((BaseSequentialStream *) &SD1, "Slave txbuffer[%d]=%d ",n ,txbuffer[n]);
        }
  (void)arg;
  chRegSetThreadName("SlaveRecieveI2C");
  //i2cSlaveConfigure( slaveaddr, txbuffer, txbytes, rxbuf, rxbytes, TIMEOUT);
  while (true) {
    palTogglePad(IOPORT2, PORTB_LED1);
    chprintf((BaseSequentialStream *) &SD1, "\r\n iniciando processo de recebimento do slave\r\n");  
        //chThdSleepMilliseconds(500);
    /*configurar endereço do slave e "encaixar" o matchaddress*/
    debug = i2cMatchAddress(&I2CD1, slaveaddr);    
    chprintf((BaseSequentialStream *) &SD1, "debug = %d\r\n",debug);
    errors=i2cGetErrors(&I2CD1);  
    chprintf((BaseSequentialStream *) &SD1, "errors = %d\r\n",errors);
    debug= i2cSlaveReply(&I2CD1, slaveaddr, txbuffer, txbytes, rxbuffer, rxbytes, TIMEOUT); 
    chprintf((BaseSequentialStream *) &SD1, "debug slave API = %d\r\n",debug);    
    //i2cSlaveReceive(&I2CD1, slaveaddr, txbuffer, txbytes, rxbuffer, rxbytes, TIMEOUT);
      chprintf((BaseSequentialStream *) &SD1, "final da execucao da thread\r\n");
      for (uint8_t n = 0; n < rxbytes ; n++){
        chprintf( (BaseSequentialStream *) &SD1, "Slave rxbuffer[%d]=%d ",n ,rxbuffer[n]);
        }
    }
    chprintf((BaseSequentialStream *) &SD1, "end\r\n" );   
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
   * Starts the SLAVEI2C thread
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  while (TRUE) {
    chThdSleepMilliseconds(1000);  
  }
}
