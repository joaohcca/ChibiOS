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

/**
 * @file    I2Cv1/hal_i2c_lld.c
 * @brief   AVR/MEGA I2C subsystem low level driver source.
 *
 * @addtogroup I2C
 * @{
 */
#include "hal.h"

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver local definitions.                                                */
/*==========================================================================*/

/*==========================================================================*/
/* Driver exported variables.                                               */
/*==========================================================================*/

/** @brief I2C driver identifier. */
#if AVR_I2C_USE_I2C1 || defined(__DOXYGEN__)
I2CDriver I2CD1;
#endif

/*==========================================================================*/
/* Driver local variables and types.                                        */
/*==========================================================================*/

/*==========================================================================*/
/* Driver local functions.                                                  */
/*==========================================================================*/

/*==========================================================================*/
/* Driver interrupt handlers.   TWI_MASTER_RX_DATA_ACK                                            */
/*==========================================================================*/

#if AVR_I2C_USE_I2C1 || defined(__DOXYGEN__)
/**
 * @brief   I2C event interrupt handler.
 *
 * @notapi
 */
OSAL_IRQ_HANDLER(TWI_vect) {
  OSAL_IRQ_PROLOGUE();

  I2CDriver *i2cp = &I2CD1;

  switch (TWSR & 0xF8) {
  case TWI_START: /*0x08*/
  case TWI_REPEAT_START: /*0x10*/
    TWDR = (i2cp->addr << 1);
    if ((i2cp->txbuf == NULL) || (i2cp->txbytes == 0) || (i2cp->txidx == i2cp->txbytes)) {
      TWDR |= 0x01;
    }
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    break;
  case TWI_MASTER_TX_ADDR_ACK:
  case TWI_MASTER_TX_DATA_ACK:
    if (i2cp->txidx < i2cp->txbytes) {
      TWDR = i2cp->txbuf[i2cp->txidx++];
      TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    }
    else {
      if (i2cp->rxbuf && i2cp->rxbytes) {
        TWCR = ((1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
      }
      else {
        TWCR = ((1 << TWSTO) | (1 << TWINT) | (1 << TWEN));
        _i2c_wakeup_isr(i2cp);
      }
    }
    break;
  case TWI_MASTER_RX_ADDR_ACK: /*0x40*/
    if (i2cp->rxidx == (i2cp->rxbytes - 1)) {
      TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));/*se não puder receber mais va pro 0x58*/
    }
    else {
      TWCR = ((1 << TWEA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));/*se puder 0x50*/
    }
    break;
  case TWI_MASTER_RX_DATA_ACK: /*0x50*/
    i2cp->rxbuf[i2cp->rxidx++] = TWDR; /*atualiza TWDR*/
    if (i2cp->rxidx == (i2cp->rxbytes - 1)) { 
      TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE)); /* ir para 0x58*/
    }
    else {
      TWCR = ((1 << TWEA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE)); /*se puder receber envia ack e fica 0x50*/
    }
    break;
  case TWI_MASTER_RX_DATA_NACK: /*0x58*/
    i2cp->rxbuf[i2cp->rxidx] = TWDR; /*atualiza TWDR*/
    TWCR = ((1 << TWSTO) | (1 << TWINT) | (1 << TWEN)); /*ações no TWCR*/
    _i2c_wakeup_isr(i2cp);
  case TWI_MASTER_TX_ADDR_NACK:
  case TWI_MASTER_TX_DATA_NACK:
  case TWI_MASTER_RX_ADDR_NACK:
    i2cp->errors |= I2C_ACK_FAILURE;
    break;

  /*slave recieve status*/
  case TWI_SLAVE_RX_ADDR_ACK: /*ref $60*/
  /*review cases where state 0x60 should return nack (not initialize ?)
  same decision making on the 0x68 and 0x70)*/
    if (i2cp->rxidx == (i2cp->rxbytes -1)){
      TWCR = (1 << TWINT) | (1 << TWIE);
    }
    else{
      TWCR = ((1 << TWINT) | (1 << TWEA)| (1 << TWIE));
    }
  break;
  case TWI_SLAVE_RX_POST_ARB_LOST: /*ref $68*/
    if (i2cp->rxidx ==(i2cp->rxbytes -1)){
      TWCR = (1 << TWINT) | (1 << TWIE);
    }
    else{
      TWCR = ((1 << TWINT) | (1 << TWEA)| (1 << TWIE));
    }
  case TWI_SLAVE_RX_DATA_ACK: /*ref $80*/
  /*Read the data from the bus to the buffer rxbuf recieves the byte from TWDR*/
  i2cp->rxbuf[i2cp->rxidx++] = TWDR; 
  /*send nack master move to $88*/
  if (i2cp->rxidx == (i2cp->rxbytes - 1)) {
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
  }
  /*buffer can recieve more bytes stay in $80*/
  else {
    i2cp->rxidx++;
    TWCR = ((1 << TWEA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
  }
  break;
  case TWI_SLAVE_RX_DATA_NACK: /*ref $88*/
  i2cp->rxbuf[i2cp->rxidx++] = TWDR; 

   /*implement decision making past communication with flags*/
   /*default to retain addr and not send start*/
   TWCR = ((1 << TWINT) | (1<< TWIE)| (1<<TWEA));
   _i2c_wakeup_isr(i2cp); /*wake up thread*/
  break;
  case TWI_SLAVE_STOP: /*ref A0*/
  /*default to retain addr and not send start*/
   TWCR = ((1 << TWINT) | (1<< TWIE) | (1<<TWEA));
  _i2c_wakeup_isr(i2cp); /*wake up thread*/
  break;
  //slave transmitt status
  case TWI_SLAVE_TX_ADDR_ACK: /*ref $A8*/
  /*load data from buffer to TWDR*/
  /*check if there's more data to transmmit*/
   TWDR = i2cp->txbuf[i2cp->txidx]; 
  if (i2cp->txidx ==(i2cp->txbytes -1)){
    TWCR = ((1 << TWINT) | (1 << TWIE));
  }
  else{
    i2cp->txidx++;
    TWCR = ((1 << TWINT) | (1 << TWEA) | (1 << TWIE));
  }
  break;
  case TWI_SLAVE_TX_POST_ARB_LOST:/*ref $B0*/
  /*load data and check for nack transmission*/
    TWDR = i2cp->txbuf[i2cp->txidx];
  if (i2cp->txidx ==(i2cp->txbytes -1)){
    TWCR = (1 << TWINT | (1 << TWIE));
  }
  else{
    i2cp->txidx++;
    TWCR = ((1 << TWINT) | (1 << TWEA) | (1 << TWIE));
  }
  break;
  case TWI_SLAVE_TX_DATA_ACK: /*ref $B8*/
    TWDR = i2cp->txbuf[i2cp->txidx]; 
  if (i2cp->txidx ==(i2cp->txbytes -1)){
    TWCR = (1 << TWINT | (1 << TWIE));
  }
  else{
    i2cp->txidx++;
    TWCR = ((1 << TWINT) | (1 << TWEA) | (1 << TWIE));
  }
  break;
  case TWI_SLAVE_TX_DATA_NACK: /*ref $C0*/
  /*implement decision making past communication with flags*/
   /*default to retain addr and not send start*/
   TWCR = ((1 << TWINT) | (1<< TWIE)| (1<<TWEA));
  break;
  case TWI_SLAVE_TX_LAST_DATA_ACK: /*ref $C8*/
  /*implement decision making past communication with flags*/
   /*default to retain addr and not send start*/
   TWCR = ((1 << TWINT) | (1<< TWIE)| (1<<TWEA));
   break;
  case TWI_ARBITRATION_LOST:
    i2cp->errors |= I2C_ARBITRATION_LOST;
    break;
  case TWI_BUS_ERROR:
    i2cp->errors |= I2C_BUS_ERROR;
    break;
  default:
    /* FIXME: only gets here if there are other MASTERs in the bus. */
    TWCR = ((1 << TWSTO) | (1 << TWINT) | (1 << TWEN));
    _i2c_wakeup_error_isr(i2cp);
  }

  if (i2cp->errors != I2C_NO_ERROR) {
    TWCR = ((1 << TWSTO) | (1 << TWINT) | (1 << TWEN));
    _i2c_wakeup_error_isr(i2cp);
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* AVR_I2C_USE_I2C1 */

/*==========================================================================*/
/* Driver exported functions.                                               */
/*==========================================================================*/

/**
 * @brief   Low level I2C driver initialization.
 *
 * @notapi
 */
void i2c_lld_init(void) {

  i2cObjectInit(&I2CD1);
  I2CD1.thread = NULL;
}

/**
 * @brief   Configures and activates the I2C peripheral.
 *
 * @param[in] i2cp  pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_start(I2CDriver *i2cp) {

  uint32_t clock_speed = 100000;

  /* TODO: Test TWI without external pull-ups (use internal). */

  /* Configure prescaler to 1. */
  TWSR &= 0xF8;
  if (i2cp->config != NULL)
    clock_speed = i2cp->config->clock_speed;

  /* Configure baudrate. */
  TWBR = ((F_CPU / clock_speed) - 16) / 2;
} 
/**
 * @brief   Deactivates the I2C peripheral.
 *
 * @param[in] i2cp  pointer to the @p I2CDriver object
 *
 * @notapi
 */
void i2c_lld_stop(I2CDriver *i2cp) {

  if (i2cp->state != I2C_STOP) {
    /* Disable TWI subsystem and stop all operations. */
    TWCR &= ~(1 << TWEN);
  }
}

/**
 * @brief   Receives data via the I2C bus as master.
 *
 * @param[in]   i2cp      pointer to the @p I2CDriver object
 * @param[in]   addr      slave device address
 * @param[out]  rxbuf     pointer to the receive buffer
 * @param[in]   rxbytes   number of bytes to be received
 * @param[in]   timeout   the number of ticks before the operation timeouts,
 *                        the following special values are allowed:
 *                        - @a TIME_INFINITE no timeout
 *                      
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 */
msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                     uint8_t *rxbuf, size_t rxbytes,
                                     systime_t timeout) {

  i2cp->errors = I2C_NO_ERROR;
  i2cp->addr = addr;
  i2cp->txbuf = NULL;
  i2cp->txbytes = 0;
  i2cp->txidx = 0;
  i2cp->rxbuf = rxbuf;
  i2cp->rxbytes = rxbytes;
  i2cp->rxidx = 0;

  TWCR = ((1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
  /* a thread fica suspensa até conclusão do processo de recebimento de dados*/
  return osalThreadSuspendTimeoutS(&i2cp->thread, TIME_INFINITE);
}

/**
 * @brief   Transmits data via the I2C bus as master.
 *
 * @param[in]   i2cp      pointer to the @p I2CDriver object
 * @param[in]   addr      slave device address
 * @param[in]   txbuf     pointer to the transmit buffer
 * @param[in]   txbytes   number of bytes to be transmitted
 * @param[out]  rxbuf     pointer to the receive buffer
 * @param[in]   rxbytes   number of bytes to be received
 * @param[in]   timeout   the number of ticks before the operation timeouts,
 *                        the following special values are allowed:
 *                        - @a TIME_INFINITE no timeout
 *                      
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end. <b>After a
 *                      timeout the driver must be stopped and restarted
 *                      because the bus is in an uncertain state</b>.
 *
 * @notapi
 */
msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout) {

  i2cp->errors = I2C_NO_ERROR;
  i2cp->addr = addr;
  i2cp->txbuf = txbuf;
  i2cp->txbytes = txbytes;
  i2cp->txidx = 0;
  i2cp->rxbuf = rxbuf;
  i2cp->rxbytes = rxbytes;
  i2cp->rxidx = 0;

  TWCR = ((1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
  /*A thread fica suspensa ate o final do processo de envio da mensagem*/
  return osalThreadSuspendTimeoutS(&i2cp->thread, TIME_INFINITE);
}

/*
* brief Configure to respond to messages directed to the given i2cadr
* @param[in] i2cp      pointer to the @p I2CDriver object
*  @param[in] i2cadr    I2C bus address
* @return              Length of message OR the type of event received
* @retval I2C_OK       Success
* @retval I2C_ERROR    Cannot match address in addition of those already (como chegar aqui?)
*  * @details MatchAddress calls are cumulative.
*          Specify address zero to match I2C "all call"
*          Most hardware supports matching only a signle nonzero address.
*
* @api
*/
msg_t i2c_lld_matchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr){
  if (i2cadr != 0 ){ 
    uint8_t adr = i2cadr << 1;                                     /*by pass General Call ADDR, add mechanism to implement GC*/
    i2cp->addr = adr;                                               /*Implement slave addr*/
    return I2C_NO_ERROR; 
    }
  else
    return i2c_lld_get_slaveErrors(i2cp);  /*find a ERROR*/
}
/*stop respond to certain addr*/

void i2c_ld_unmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr){
  
  if (i2cp->addr == i2cadr && i2cadr != 0)
  TWAR = 0; //unset previously configured slave addr
}

void i2c_lld_unmatchAll(I2CDriver *i2cp){
  TWAR = 0; //force unset previously configured slave addr
}
/*@brief   Configure callbacksslaveRx & buffers to receive messages
 * @details             Call i2cMatchAddress() after this to start processing
 *     Enabling match addresses before installing handler callbacks can
 *     result in locking the I2C bus when a master accesses those
 *     unconfigured slave addresses
 *
 * @notapi
 */

/*Usar as funcoes do mestre como referência e ver diferenças*/
msg_t  i2c_lld_slaveReceive(I2CDriver *i2cp,  i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout){
  
  
  i2cp->errors = I2C_NO_ERROR;
  i2cp->txbuf = NULL;
  i2cp->txbytes = 0;
  i2cp->txidx = 0;
  i2cp->rxbuf = rxbuf;
  i2cp->rxbytes = rxbytes;
  i2cp->rxidx = 0;

  TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1<< TWEA)); 
  //osal thread  suspend
 return osalThreadSuspendTimeoutS(&i2cp->thread, TIME_INFINITE);
  //acordar thread no final da maquina de estados (osal recapthread)
  }


msg_t  i2c_lld_slaveReply(I2CDriver *i2cp,  i2caddr_t addr,
                                      const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes,
                                      systime_t timeout){
 
  i2cp->errors = I2C_NO_ERROR;
  i2cp->txbuf = txbuf;
  i2cp->txbytes = txbytes;
  i2cp->txidx = 0;
  i2cp->rxbuf = rxbuf;
  i2cp->rxbytes = rxbytes;
  i2cp->rxidx = 0;
  TWCR = ( (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1<< TWEA));
  return osalThreadSuspendTimeoutS(&i2cp->thread, TIME_INFINITE);    
  }



/** @} */

#endif 