/**
 * \author Mr.Nobody
 * \file I2c_Isr.h
 * \ingroup I2c
 * \brief I2c module interrupt data transfer handler (private to the I2c library)
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

/** \brief I2C interrupt sources used by the data transfer handlers (I2C_CR1 enable bits) */
typedef enum
{
    I2C_ISR_IT_NONE   = 0u,                  /**< No interrupt source                                 */
    I2C_ISR_IT_TX     = LL_I2C_CR1_TXIE,     /**< Transmit data register empty (TXIS)                 */
    I2C_ISR_IT_RX     = LL_I2C_CR1_RXIE,     /**< Receive data register not empty (RXNE)              */
    I2C_ISR_IT_NACK   = LL_I2C_CR1_NACKIE,   /**< Not acknowledge received (NACKF)                    */
    I2C_ISR_IT_STOP   = LL_I2C_CR1_STOPIE,   /**< STOP condition detected (STOPF)                     */
    I2C_ISR_IT_TC     = LL_I2C_CR1_TCIE,     /**< Transfer complete / complete reload (TC / TCR)      */
    I2C_ISR_IT_ERR    = LL_I2C_CR1_ERRIE,    /**< Bus error / arbitration lost / overrun              */
    I2C_ISR_IT_EVENTS = ( LL_I2C_CR1_NACKIE | LL_I2C_CR1_STOPIE |
                          LL_I2C_CR1_TCIE   | LL_I2C_CR1_ERRIE  ), /**< Transfer sequencing events (DMA and ISR mode) */
    I2C_ISR_IT_ALL    = ( LL_I2C_CR1_TXIE   | LL_I2C_CR1_RXIE   |
                          LL_I2C_CR1_NACKIE | LL_I2C_CR1_STOPIE |
                          LL_I2C_CR1_TCIE   | LL_I2C_CR1_ERRIE  )  /**< All interrupt sources used by the handlers     */
}   i2c_IsrIt_t;


/** \brief Bit mask of \ref i2c_IsrIt_t values */
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
