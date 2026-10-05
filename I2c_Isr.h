/**
 * \author Mr.Nobody
 * \file I2c_Isr.h
 * \ingroup I2c
 * \brief I2c module interrupt data transfer handler (private to the I2c library)
 *
 * Handler of I2C_XFER_MODE_ISR - the transfer is sequenced and the data are moved by the
 * I2C event and error interrupt service routine. The interrupt enable control is shared with
 * the DMA handler (addressing, end of transfer and errors are handled by I2C interrupts in DMA
 * mode as well).
 *
 */

#ifndef I2C_I2C_ISR_H
#define I2C_I2C_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "I2c_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief I2C interrupt sources (CR2 interrupt enable bits) */
typedef enum
{
    I2C_ISR_IT_NONE   = 0u,                  /**< No interrupt source                                     */
    I2C_ISR_IT_EVT    = I2C_CR2_ITEVTEN,     /**< Event interrupt (SB, ADDR, ADD10, BTF)                  */
    I2C_ISR_IT_BUF    = I2C_CR2_ITBUFEN,     /**< Buffer interrupt (TXE, RXNE - requires EVT)             */
    I2C_ISR_IT_ERR    = I2C_CR2_ITERREN,     /**< Error interrupt (BERR, ARLO, AF, OVR)                   */
    I2C_ISR_IT_EVENTS = ( I2C_CR2_ITEVTEN | I2C_CR2_ITERREN ),                     /**< Transfer sequencing events (DMA and ISR mode) */
    I2C_ISR_IT_ALL    = ( I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN )    /**< All interrupt sources used by the handlers     */
}   i2c_IsrIt_t;


/** \brief Mask of interrupt sources (combination of \ref i2c_IsrIt_t values) */
typedef uint32_t i2c_IsrItMask_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

i2c_RequestState_t I2c_Isr_Check_Config ( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig );
i2c_RequestState_t I2c_Isr_XferInit     ( i2c_PeriphId_t periphId );
i2c_RequestState_t I2c_Isr_XferDeinit   ( i2c_PeriphId_t periphId );
i2c_RequestState_t I2c_Isr_XferStart    ( i2c_PeriphId_t periphId );
i2c_RequestState_t I2c_Isr_XferStop     ( i2c_PeriphId_t periphId );

i2c_RequestState_t I2c_Isr_Set_ItActive   ( i2c_PeriphId_t periphId, i2c_IsrItMask_t itMask );
i2c_RequestState_t I2c_Isr_Set_ItInactive ( i2c_PeriphId_t periphId, i2c_IsrItMask_t itMask );

i2c_RequestState_t I2c_Isr_Handler      ( i2c_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* I2C_I2C_ISR_H */
