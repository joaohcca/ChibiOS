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
 * @file    I2Cv1/hal_i2c_lld.h
 * @brief   AVR/MEGA I2C subsystem low level driver header.
 *
 * @addtogroup I2C
 * @{
 */

#ifndef HAL_I2C_LLD_H
#define HAL_I2C_LLD_H

#if HAL_USE_I2C || defined(__DOXYGEN__)

/*==========================================================================*/
/* Driver constants.                                                        */
/*==========================================================================*/

/**MASTER DEF                                       */
/** @brief   START transmitted.                     */
#define TWI_START                  0x08
/** @brief   Repeated START transmitted.            */
#define TWI_REPEAT_START           0x10
/** @brief   Arbitration Lost.                      */
#define TWI_ARBITRATION_LOST       0x38
/** @brief   Bus errors.                            */
#define TWI_BUS_ERROR              0x00

/** @brief   SLA+W transmitted with ACK response.   */
#define TWI_MASTER_TX_ADDR_ACK     0x18
/** @brief   SLA+W transmitted with NACK response.  */
#define TWI_MASTER_TX_ADDR_NACK    0x20
/** @brief   DATA transmitted with ACK response.    */
#define TWI_MASTER_TX_DATA_ACK     0x28
/** @brief   DATA transmitted with NACK response.   */
#define TWI_MASTER_TX_DATA_NACK    0x30

/** @brief   SLA+R transmitted with ACK response.   */
#define TWI_MASTER_RX_ADDR_ACK     0x40
/** @brief   SLA+R transmitted with NACK response.  */
#define TWI_MASTER_RX_ADDR_NACK    0x48
/** @brief   DATA received with ACK response.       */
#define TWI_MASTER_RX_DATA_ACK     0x50
/** @brief   DATA received with NACK response.      */
#define TWI_MASTER_RX_DATA_NACK    0x58

/* SLAVE DEF                                         */

/*SLAVE_RECIEVER_MODE								 */

/**@brief   own SLA+W recieved ACK has been returned */
#define TWI_SLAVE_RX_ADDR_ACK      0x60

/**@brief   Arbitration lost in SLA+R/W as master, recieved SLA+W ACK RETURN*/
#define TWI_SLAVE_RX_POST_ARB_LOST 0x68

/**@brief  general call address received ACK has been returned */
#define TWI_SLAVE_RX_GCA           0x70

/**@brief  Arbitration lost in SLA+R/W as master; GCA recieved  */
#define TWI_SLAVE_RX_GCA_POST_ARB_LOST      0x78

/**@brief   previously addressed with own SLA+W data recieved ACK */
#define TWI_SLAVE_RX_DATA_ACK      0x80

/**@brief   previously addressed with own SLA+W data recieved NACK */
#define TWI_SLAVE_RX_DATA_NACK      0x88

/**@brief   previously addressed with GCA DATA recieved ACK */
#define TWI_SLAVE_RX_GCA_DATA_ACK      0x90

/**@brief   previously addressed with GCA DATA recieved NACK */
#define TWI_SLAVE_RX_GCA_DATA_NACK      0x98

/**@brief   stop condition or repeated start recieved while still addressed as slave*/
#define TWI_SLAVE_STOP      0xA0

/*SLAVE_TRANSMITTER_MODE                              */

/**@brief   own SLA+R recieved ACK has been returned */
#define TWI_SLAVE_TX_ADDR_ACK      0xA8

/**@brief   Arbitration lost in SLA+R/W as master, recieved SLA+R ACK RETURN */
#define TWI_SLAVE_TX_POST_ARB_LOST       0xB0

/**@brief DATA BYTE in TWDR has been transmitted, ACK returned*/
#define TWI_SLAVE_TX_DATA_ACK					0xB8

/**@brief DATA BYTE in TWDR has been transmitted, NACK returned*/
#define TWI_SLAVE_TX_DATA_NACK					0xC0

/**@brief last DATA BYTE in TWDR has been transmitted, NACK returned*/
#define TWI_SLAVE_TX_LAST_DATA_ACK					0xC8


/*==========================================================================*/
/* Driver pre-compile time settings.                                        */
/*==========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief     I2C driver enable switch.
 * @details   If set to @p TRUE the support for I2C is included.
 * @note      The default is @p FALSE.
 */
#if !defined(AVR_I2C_USE_I2C1) || defined(__DOXYGEN__)
#define AVR_I2C_USE_I2C1           FALSE
#endif
/** @} */

/*==========================================================================*/
/* Derived constants and error checks.                                      */
/*==========================================================================*/

/*==========================================================================*/
/* Driver data structures and types.                                        */
/*==========================================================================*/

/**
 * @brief   Type representing I2C address.
 */
typedef uint8_t i2caddr_t;

/**
 * @brief   I2C Driver condition flags type.
 */
typedef uint8_t i2cflags_t;

/**
 * @brief   Driver configuration structure.
 * @note    Implementations may extend this structure to contain more,
 *          architecture dependent, fields.
 */
typedef struct {

  /**
   * @brief Specifies the I2C clock frequency.
   */
  uint32_t        clock_speed;

} I2CConfig;

typedef struct I2CDriver I2CDriver;
/**
 * @brief   Structure representing an I2C driver.
 */
  /* I2C slave mode support */

typedef struct I2CSlaveMsg I2CSlaveMsg;

/*
  returns the current I2C slave message receive configuration
*/
I2CSlaveMsg *i2cSlaveGetReceiveMsg(I2CDriver *i2cp);


/*
  returns the current I2C slave message reply configuration
*/
I2CSlaveMsg *i2cSlaveGetReplyMsg(I2CDriver *i2cp);


/*
  I2C Slave Message Call Back.
  Invoked from interrupt context just after
  the last byte of the message is transferred or slaveAdr is matched.

  Use i2cSlaveReceiveMsg() or i2cSlaveReplyMsg() to access
  the relevant message handling configuration
*/
typedef void I2CSlaveMsgCB(I2CDriver *i2cp);


/*
  I2CSlaveMsg message handling configurations are normally
  stored in read-only memory.
  They describe either a buffer to contain incoming messages from
  a bus master and associated callback functions, or one
  preloaded with an outgoing reply to a read request and its callbacks.
*/

struct I2CSlaveMsg {
  size_t     size;     			/* sizeof(body) -- zero if master must wait */
  uint8_t   *body;     			/* message contents -- or NULL if master must wait */
  I2CSlaveMsgCB *adrMatched;  	/* invoked when slave address matches -> setar os comandos pro case do TWSR e TWCR*/
  I2CSlaveMsgCB *processMsg;  	/* invoked after message is transferred -> (nescessário??)*/
  I2CSlaveMsgCB *exception;   	/* invoked if error or timeout during transfer */
};


/*
  dummy callback -- placeholder to ignore event
*/
I2CSlaveMsgCB I2CSlaveDummyCB;

struct I2CDriver {
  /**
   * @brief   Driver state.
   */
  i2cstate_t                state;
  /**
   * @brief   Current configuration data.
   */
  const I2CConfig           *config;
  /**
   * @brief   Error flags.
   */
  i2cflags_t                errors;
#if I2C_USE_MUTUAL_EXCLUSION || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the bus.
   */
  mutex_t                   mutex;
#endif /* I2C_USE_MUTUAL_EXCLUSION */
#if defined(I2C_DRIVER_EXT_FIELDS)
  I2C_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields. */
  /**
   * @brief   Thread waiting for I/O completion.
   */
  thread_reference_t        thread;
  /**
   * @brief   Address of slave device.
   */
  i2caddr_t                 addr;
  /**
   * @brief   Pointer to the buffer with data to send.
   */
  const uint8_t             *txbuf;
  /**
   * @brief   Number of bytes of data to send.
   */
  size_t                    txbytes;
  /**
   * @brief   Current index in buffer when sending data.
   */
  size_t                    txidx;
  /**
   * @brief   Pointer to the buffer to put received data.
   */
  uint8_t                   *rxbuf;
  /**
   * @brief   Number of bytes of data to receive.
   */
  size_t                    rxbytes;
  /**
   * @brief   Current index in buffer when receiving data.
   */
  size_t                    rxidx;

  /* additional fields to support I2C slave transactions */
  
  /**
   * @brief     slave address of message being processed
   */
  i2caddr_t                 targetAdr;
  /**
   * @brief     Error Mask for last slave message
   */
  i2cflags_t                slaveErrors;
  /**
   * @brief     Length of most recently transferred slave message
   */
  size_t                  slaveBytes;
  /**
   * @brief     Maximum # of ticks slave may stretch the I2C clock
   */
  sysinterval_t            slaveTimeout;
  /**
   * @brief     Pointer to slave message reception handler
   */
  const I2CSlaveMsg         *slaveRx;
  /**
   * @brief     Pointer to slave message Reply (transmit) handler
   *
   * @note		This is the currently active/just completed reply
   */
  const I2CSlaveMsg         *slaveReply;
  /**
   * @brief     Pointer to handler for next slave received message
   */
  const I2CSlaveMsg         *slaveNextRx;
  /**
   * @brief     Pointer to handler for next slave reply (transmit) message
   *
   * @note		This is used for a reply if no message received first
   */
  const I2CSlaveMsg         *slaveNextReply;
  /**
   * @brief     Pointer to the next RX buffer location.
   */
  uint8_t                   *rxptr;
  /**
   * @brief     Number of bytes in RX phase.
   */
  uint8_t                    *txptr;
};

/**
 * @brief   Type of a structure representing an I2C driver.
 */


/*==========================================================================*/
/* Driver macros.                                                           */
/*==========================================================================*/

/**
 * @brief   Get errors from I2C driver.
 *
 * @param[in] i2cp  pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_errors(i2cp) ((i2cp)->errors)

/**
 * @brief   Get slave errors from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_slaveErrors(i2cp) ((i2cp)->slaveErrors)

/**
 * @brief   Get slave message bytes transferred from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_slaveBytes(i2cp) ((i2cp)->slaveBytes)


/**
 * @brief   Get slave timeout in ticks from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_slaveTimeout(i2cp) ((i2cp)->slaveTimeout)

/**
 * @brief   Set slave timeout in ticks for I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_set_slaveTimeout(i2cp,ticks) ((i2cp)->slaveTimeout=(ticks))

/**
 * @brief   Get slave target address from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_slaveTargetAdr(i2cp) ((i2cp)->targetAdr)

/**
 * @brief   Get slave receive message descriptor from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @notapi
 */
#define i2c_lld_get_slaveReceive(i2cp) ((i2cp)->slaveNextRx)

/**
 * @brief   Get slave reply message descriptor from I2C driver.
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * slave receive message descriptor from I2C driver.
 * @notapi
 */
#define i2c_lld_get_slaveReply(i2cp) ((i2cp)->slaveNextReply)

/*==========================================================================*/
/* External declarations.                                                   */
/*==========================================================================*/

#if !defined(__DOXYGEN__)
#if AVR_I2C_USE_I2C1
extern I2CDriver I2CD1;
#endif
#endif /* !defined(__DOXYGEN__) */

#ifdef __cplusplus
extern "C" {
#endif
  void i2c_lld_init(void);
  void i2c_lld_start(I2CDriver *i2cp);
  void i2c_lld_stop(I2CDriver *i2cp);
  msg_t i2c_lld_master_transmit_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                        const uint8_t *txbuf, size_t txbytes,
                                        uint8_t *rxbuf, size_t rxbytes,
                                        systime_t timeout);
  msg_t i2c_lld_master_receive_timeout(I2CDriver *i2cp, i2caddr_t addr,
                                       uint8_t *rxbuf, size_t rxbytes,
                                       systime_t timeout);
  msg_t i2c_lld_matchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);
  void  i2c_lld_unmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);
  void  i2c_lld_unmatchAll(I2CDriver *i2cp);
  void  i2c_lld_slaveReceive(I2CDriver *i2cp, const I2CSlaveMsg *rxMsg);
  void  i2c_lld_slaveReply(I2CDriver *i2cp, const I2CSlaveMsg *replyMsg);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_I2C */

#endif /* HAL_I2C_LLD_H */

/** @} */
