/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012,2013 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/
/*
   Slave I2C support contributed by Brent Roman of the
    Monterey Bay Aquarium Research Institute
 */

/**
 * @file    i2cslave.h
 * @brief   Slave Mode for the I2C Driver.
 *
 * @addtogroup I2C
 * @{
 */
#ifndef _I2CSLAVE_H_
#define _I2CSLAVE_H_

#if HAL_USE_I2C_SLAVE || defined(__DOXYGEN__)

#include <hal_i2c.h>

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Configure to respond to messages directed to the given i2cadr
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] i2cadr    I2C bus address
 *                      - @a 0 matches "all call"
 *                      .
 * @return              Length of message OR the type of event received
 * @retval I2C_OK       Success
 * @retval I2C_ERROR    Cannot match address in addition of those already
 *
 * @details MatchAddress calls are cumulative.
 *          Specify address zero to match I2C "all call"
 *          Most hardware supports matching only a signle nonzero address.
 *
 * @api
 */
int  i2cMatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);


/**
 * @brief   Configure to ignore messages directed to the given i2cadr
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 * @param[in] i2cadr    I2C bus address
 *                      - @a 0 matches "all call"
 *                      .
 * @details A message being transferred that has already matched the
 *          specified address will continue being processed.
 *          Requests to unmatch an address that is not currently being matched
 *          are ignored.
 *
 * @api
 */
void  i2cUnmatchAddress(I2CDriver *i2cp, i2caddr_t  i2cadr);


/**
 * @brief   Configure to ignore all messages
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @details A message being transferred that has already matched the
 *          specified address will continue being processed.
 *
 * @api
 */
void  i2cUnmatchAll(I2CDriver *i2cp);




/* bus transaction attributes */

/**
 * @brief   return bit mask of errors associated with this slave transaction
 *
 * @param[in] i2cp      pointer to the @p I2CDriver object
 *
 * @return              I2C bus error conditions described in i2c.h
 *
 * @api
 */
/*static inline
  i2cflags_t i2cSlaveErrors(I2CDriver *i2cp)
{
  osalDbgCheck(i2cp != NULL);
  return i2c_lld_get_slaveErrors(i2cp);
}*/


/*
  An event service thread based API library called i2cevent supports processing
  slave messages on a dedicated thread.  This facility is built upon the
  low-level driver's asynchronous callback functions described below:

  Each callback function may alter the processing of subsequent I2C
  messages and read requests by calling i2cSlaveReceive() and
  i2cSlaveReply(), respectively.  Further, callbacks may alter their
  i2cSlaveMsg structs in RAM, but only those for their own channel.
  Such changes take immediate affect.  This facility can be used to
  avoid copying message buffers.

  If receive buffers become full or a reply to a read request cannot be
  generated immediately, the relevant I2CSlaveMsg struct may be substituted
  for another whose body pointer is NULL or whose body size is zero.
  Note that, I2CSlaveMsg structs may be modified
  in place within a channel's callbacks to the same effect.

  A NULL body pointer or zero size causes the slave to signal the master node
  to wait by holding the I2C clock signal low, "stretching it", during the next
  transaction to which that I2CSlaveMsg applies.
  The I2C clock resumes only after a i2cSlaveSetReceive() or SetReply() is
  called with an I2CSlaveMsg containing a non-NULL body,
  or after the transaction timeout expires.

  Therefore, if a NULL body pointer is replaced with a non-NULL one or
  a zero length is replaced with a non-zero one, i2cSlaveReceive() or
  i2cSlaveReply() MUST be called -- even if with the same pointer values --
  to inform the i2c driver that the transaction may resume.

  Note that Receive and Reply processing is initially "locked".
*/

msg_t i2cSlaveReceive(I2CDriver *i2cp, uint8_t *rxbuf, size_t rxbytes, bool gce, sysinterval_t timeout);


msg_t i2cSlaveReply(I2CDriver *i2cp, const uint8_t *txbuf, size_t txbytes,
                                      uint8_t *rxbuf, size_t rxbytes, sysinterval_t timeout);




#ifdef __cplusplus
}
#endif

#endif  /* HAL_USE_I2C_SLAVE */

#endif  /* _I2CSLAVE_H_ */
