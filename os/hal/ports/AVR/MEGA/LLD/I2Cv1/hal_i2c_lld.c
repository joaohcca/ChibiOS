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
/* Driver interrupt handlers.                                               */
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
  case TWI_START:
  case TWI_REPEAT_START:
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
  case TWI_MASTER_RX_ADDR_ACK:
    if (i2cp->rxidx == (i2cp->rxbytes - 1)) {
      TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    }
    else {
      TWCR = ((1 << TWEA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    }
    break;
  case TWI_MASTER_RX_DATA_ACK:
    i2cp->rxbuf[i2cp->rxidx++] = TWDR;
    if (i2cp->rxidx == (i2cp->rxbytes - 1)) {
      TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    }
    else {
      TWCR = ((1 << TWEA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
    }
    break;
  case TWI_MASTER_RX_DATA_NACK:
    i2cp->rxbuf[i2cp->rxidx] = TWDR;
    TWCR = ((1 << TWSTO) | (1 << TWINT) | (1 << TWEN));
    _i2c_wakeup_isr(i2cp);
  case TWI_MASTER_TX_ADDR_NACK:
  case TWI_MASTER_TX_DATA_NACK:
  case TWI_MASTER_RX_ADDR_NACK:
    i2cp->errors |= I2C_ACK_FAILURE;
    break;
  case TWI_SLAVE_RX_ADDR_ACK:
  /*decision making based on rxbytes on slave
    on the last byte recieved send nack to bus*/
  /*same decision making on the 0x68 and 0x70)*/
    if (i2cp->rxidx ==(i2cp->rxbytes -1)){
      TWCR = (1 << TWINT) | (1 << TWIE)
    }
    else{
      TWCR = ((1 << TWINT) | (1 << TWEA)| (1 << TWIE))
    }
  break;
  case TWI_SLAVE_RX_DATA_ACK:
  /*insert the data from bus in TWDR*/
  i2cp->rxbuf[i2cp->rxidx++] = TWDR;
    /*check last byte*/
  if (i2cp->rxidx == (i2cp->rxbytes - 1)) {
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
  }
  /*if not last byte return ack to continue communication with master*/
  else {
    i2cp->rxidx++;
    TWCR = ((1 << TWEA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));
  }
  break;
  case TWI_SLAVE_TX_ADDR_ACK:
  /*load data from buffer to TWDR*/
  /*check if there's more data to transmmit*/
    TWDR = i2cp->txbuf
       if (i2cp->txidx ==(i2cp->txbytes -1)){
    TWCR = ((1 << TWINT) | (1 << TWIE))
  }
  else{
    i2cp->txidx++;
    TWCR = ((1 << TWINT) | (1 << TWEA) | (1 << TWIE))
  }
  break;
  case TWI_SLAVE_TX_ADDR_ACK:
  /*load data and check for nack transmission*/
    TWDR = i2cp->txbuf 
  if (i2cp->txidx ==(i2cp->txbytes -1)){
    TWCR = (1 << TWINT | (1 << TWIE))
  }
  else{
    i2cp->txidx++;
    TWCR = ((1 << TWINT) | (1 << TWEA) | (1 << TWIE))
  }
  break;
  case TWI_SLAVE_STOP: 
  /*currently considering slave add recognition and no start send*/
  TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE));
  break;
  case TWI_SLAVE_TX_DATA_NACK: 
  TWCR = TWCR = ((1 << TWINT) | (1 << TWEA) | (1 << TWIE))
  break;
  case TWI_SLAVE_TX_LAST_DATA_ACK
  TWCR = ((1 << TWINT) | (1 << TWIE))
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
 *
 * @notapi
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

  /* Send START. */
  TWCR = ((1 << TWSTA) | (1 << TWINT) | (1 << TWEN) | (1 << TWIE));

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

  return osalThreadSuspendTimeoutS(&i2cp->thread, TIME_INFINITE);
}

/* Adding:  i2c_lld_matchAddress(), i2c_lld_unmatchAddress(), i2c_lld_unmatchAll(), i2c_lld_slaveReceive() e i2c_lld_slaveReply()
/*@brief Configure to respond to messages directed to the given i2cadr
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
//I2C_TypeDef *dp = i2cp->i2c; /*duvida(1) o que seria o campo do i2c que nao existe no struct*/
  
  if (i2cadr != 0 ) 
    uint32_t adr = i2cadr << 1;                        /*by pass no bit do GC*/
    i2cp->addr = adr                                                /*escrever i2cadr no Two Wire Address Register*/
    TWCR = ((1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1<< TWEA)); /*setar flags no Control register*/
    return I2C_OK
  else
    return I2C_ERROR

}
/* A ideia é parar de responder ao endereço especificado*/
/*checar se tem algo em TWAR e remove o que tiver dúvida(3)*/ 

void i2c_ld_unmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr){
  
  if (i2cp->addr == i2cadr & i2cadr != 0)
  TWAR = 0 //zerar TWAR
  TWCR = ((1 << TWINT) | (1 << TWEN) | (1<< TWEA)); /*setar flags no Control register*/
  /*TO DO: enviar um stop no bus*/
}
//remove TWAR definition set TWSTOP=1 
/*brief*/

/*hermano sugeriu usar uma chamada do anterior*/

void i2c_lld_unmatchAll(I2CDriver *i2cp){
  TWAR = 0 //zerar TWAR
  TWCR = ((1 << TWINT) | (1 << TWEN) | (1<< TWEA));

}

/*@brief   Configure callbacks & buffers to receive messages
 * @details             Call i2cMatchAddress() after this to start processing
 *     Enabling match addresses before installing handler callbacks can
 *     result in locking the I2C bus when a master accesses those
 *     unconfigured slave addresses
 *
 * @notapi
 */

/*Usar as funcoes do mestre como referência e ver diferenças*/
void i2c_lld_slaveReceive(I2CDriver *i2cp, const *i2cp->rxbuf){
}

void i2c_lld_slaveReply(I2CDriver *i2cp, const *i2cp->txbuf){


}


/** @} */

#endif /*HAL_USE_I2C */