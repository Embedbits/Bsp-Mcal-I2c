/**
 * \author Mr.Nobody
 * \file I2c_Port.h
 * \ingroup I2c
 * \brief Inter-Integrated Circuit (I2C) MCAL module public functionality
 *
 * This file contains all available public functionality, any other files shall
 * not used outside of the module.
 *
 */

#ifndef I2C_I2C_PORT_H
#define I2C_I2C_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "I2c_Types.h"                      /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

i2c_ModuleVersion_t     I2c_Get_ModuleVersion           ( void );

i2c_RequestState_t      I2c_Init                        ( const i2c_Config_t * const i2cConfig );
i2c_RequestState_t      I2c_Deinit                      ( i2c_PeriphId_t periphId );
void                    I2c_Task                        ( void );

i2c_RequestState_t      I2c_Get_DefaultConfig           ( i2c_Config_t * const i2cConfig );

/*------------------------- Peripheral configuration -------------------------*/

i2c_RequestState_t      I2c_Set_PeriphActive            ( i2c_PeriphId_t periphId );
i2c_RequestState_t      I2c_Set_PeriphInactive          ( i2c_PeriphId_t periphId );
i2c_RequestState_t      I2c_Get_PeriphState             ( i2c_PeriphId_t periphId, i2c_FlagState_t * const periphState );

i2c_RequestState_t      I2c_Set_BusFreq                 ( i2c_PeriphId_t periphId, i2c_FreqHz_t busFreq );
i2c_RequestState_t      I2c_Get_BusFreq                 ( i2c_PeriphId_t periphId, i2c_FreqHz_t * const busFreq );

i2c_RequestState_t      I2c_Set_AnalogFilter            ( i2c_PeriphId_t periphId, i2c_AnalogFilter_t analogFilter );
i2c_RequestState_t      I2c_Get_AnalogFilter            ( i2c_PeriphId_t periphId, i2c_AnalogFilter_t * const analogFilter );

i2c_RequestState_t      I2c_Set_DigitalFilter           ( i2c_PeriphId_t periphId, i2c_DigitalFilter_t digitalFilter );
i2c_RequestState_t      I2c_Get_DigitalFilter           ( i2c_PeriphId_t periphId, i2c_DigitalFilter_t * const digitalFilter );

i2c_RequestState_t      I2c_Set_AddrMode                ( i2c_PeriphId_t periphId, i2c_AddrMode_t addrMode );
i2c_RequestState_t      I2c_Get_AddrMode                ( i2c_PeriphId_t periphId, i2c_AddrMode_t * const addrMode );

i2c_RequestState_t      I2c_Get_BusState                ( i2c_PeriphId_t periphId, i2c_FlagState_t * const busBusy );

/*-------------------- Data handling (DMA / ISR / POLL) ----------------------*/

i2c_RequestState_t      I2c_Set_DataConfig              ( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig );
i2c_RequestState_t      I2c_Get_DataConfig              ( i2c_PeriphId_t periphId, i2c_DataConfig_t * const dataConfig );

i2c_RequestState_t      I2c_Set_XferStart               ( i2c_PeriphId_t periphId, const i2c_XferRequest_t * const xferRequest );
i2c_RequestState_t      I2c_Set_XferStop                ( i2c_PeriphId_t periphId );
i2c_RequestState_t      I2c_Get_XferState               ( i2c_PeriphId_t periphId, i2c_FunctionState_t * const xferState );
i2c_RequestState_t      I2c_Get_XferError               ( i2c_PeriphId_t periphId, i2c_XferErrorId_t * const xferError );

/*------------------------------ Interrupts ----------------------------------*/

i2c_RequestState_t      I2c_Set_IrqPriority             ( i2c_PeriphId_t periphId, i2c_IrqPrio_t irqPrio );
i2c_RequestState_t      I2c_Get_IrqPriority             ( i2c_PeriphId_t periphId, i2c_IrqPrio_t * const irqPrio );

#ifdef __cplusplus
}
#endif

#endif /* I2C_I2C_PORT_H */
