/**
 * \author Mr.Nobody
 * \file I2c_Dma.h
 * \ingroup I2c
 * \brief I2c module DMA data transfer handler (private to the I2c library)
 *
 */

#ifndef I2C_I2C_DMA_H
#define I2C_I2C_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "I2c_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

i2c_RequestState_t I2c_Dma_Check_Config ( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig );
i2c_RequestState_t I2c_Dma_XferInit     ( i2c_PeriphId_t periphId );
i2c_RequestState_t I2c_Dma_XferDeinit   ( i2c_PeriphId_t periphId );
i2c_RequestState_t I2c_Dma_XferStart    ( i2c_PeriphId_t periphId );
i2c_RequestState_t I2c_Dma_XferStop     ( i2c_PeriphId_t periphId );
i2c_RequestState_t I2c_Dma_Check_Done   ( i2c_PeriphId_t periphId );

#ifdef __cplusplus
}
#endif

#endif /* I2C_I2C_DMA_H */
