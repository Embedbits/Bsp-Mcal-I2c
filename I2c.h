/**
 * \author Mr.Nobody
 * \file I2c.h
 * \ingroup I2c
 * \brief I2c module private interface shared between I2c.c and data transfer handlers
 *
 * This file is private to the I2c library (not exported through public headers). It
 * connects the module root (I2c.c) with data transfer mode handlers (I2c_Dma.c, I2c_Isr.c,
 * I2c_Poll.c). The transfer sequencing (CR2 programming, reload, repeated START, end of
 * transfer and error handling) and user callbacks are implemented once in I2c.c, the mode
 * handlers only move the data and enable their resources.
 *
 */

#ifndef I2C_I2C_H
#define I2C_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "I2c_Types.h"                      /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief Type representing iteration count of busy-wait loops (register read-back) */
typedef uint32_t i2c_TimeoutCnt_t;


/** \brief Type representing raw value of I2C ISR register (flags snapshot) */
typedef uint32_t i2c_IsrFlags_t;


/** \brief Phase of the master transfer */
typedef enum
{
    I2C_XFER_PHASE_WRITE = 0u, /**< Bytes are written to the slave (also address only transfer) */
    I2C_XFER_PHASE_READ,       /**< Bytes are read from the slave                               */
    I2C_XFER_PHASE_CNT         /**< Count of transfer phases                                    */
}   i2c_XferPhase_t;


/** \brief Runtime context of data handling (one per I2C peripheral) */
typedef struct
{
    i2c_DataConfig_t              Config;         /**< Copy of user data handling configuration             */
    i2c_FunctionState_t           InitState;      /**< Data handling is initialized                         */
    i2c_XferRequest_t             Request;        /**< Copy of the running transfer request                 */
    volatile i2c_DataCnt_t        TxIdx;          /**< Index of the next byte to be written (ISR / POLL)    */
    volatile i2c_DataCnt_t        RxIdx;          /**< Index of the next byte to be read (ISR / POLL)       */
    volatile i2c_DataCnt_t        PhaseRemaining; /**< Bytes of the phase not yet programmed into NBYTES    */
    volatile i2c_XferPhase_t      Phase;          /**< Running transfer phase                               */
    volatile i2c_FunctionState_t  XferState;      /**< Transfer is running (START - STOP)                   */
    volatile i2c_XferErrorId_t    XferError;      /**< Error of the running / last transfer                 */
}   i2c_XferContext_t;


/** \brief Data transfer mode handler interface (one per mode) */
typedef struct
{
    i2c_RequestState_t ( *CheckConfig )( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig ); /**< Mode specific configuration check          */
    i2c_RequestState_t ( *Init        )( i2c_PeriphId_t periphId );                                            /**< Mode resources initialization              */
    i2c_RequestState_t ( *Deinit      )( i2c_PeriphId_t periphId );                                            /**< Mode resources deinitialization            */
    i2c_RequestState_t ( *Start       )( i2c_PeriphId_t periphId );                                            /**< Transfer start (before START condition)    */
    i2c_RequestState_t ( *Stop        )( i2c_PeriphId_t periphId );                                            /**< Transfer stop (resources released)         */
    i2c_RequestState_t ( *CheckDone   )( i2c_PeriphId_t periphId );                                            /**< All data were moved at the end of transfer */
}   i2c_XferModeIf_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Busy-wait iteration budget used for register read-back (common for all I2c module files).
 *  Same order of magnitude and role as USART_TIMEOUT_RAW / ADC_TIMEOUT_RAW. */
#define I2C_TIMEOUT_RAW                     ( (i2c_TimeoutCnt_t)0x84FCBu )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in I2c.c - common services for data transfer handlers */
i2c_RequestState_t I2c_Get_PeriphReg      ( i2c_PeriphId_t periphId, I2C_TypeDef ** const periphReg );
i2c_RequestState_t I2c_Get_PeriphDmaReq   ( i2c_PeriphId_t periphId, gpdma_PeriphReqId_t * const txRequest, gpdma_PeriphReqId_t * const rxRequest );
i2c_RequestState_t I2c_Get_XferContext    ( i2c_PeriphId_t periphId, i2c_XferContext_t ** const xferContext );

i2c_RequestState_t I2c_Set_XferDataStep   ( i2c_PeriphId_t periphId, i2c_IsrFlags_t isrFlags );
i2c_RequestState_t I2c_Set_XferEvents     ( i2c_PeriphId_t periphId, i2c_IsrFlags_t isrFlags );
i2c_RequestState_t I2c_Set_XferError      ( i2c_PeriphId_t periphId, i2c_XferErrorId_t errorId );

#ifdef __cplusplus
}
#endif

#endif /* I2C_I2C_H */
