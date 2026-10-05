/**
 * \author Mr.Nobody
 * \file I2c_Poll.c
 * \ingroup I2c
 * \brief I2c module polling data transfer handler
 *
 * I2C_XFER_MODE_POLL - no interrupt is used. I2c_Task() calls I2c_Poll_Task() which polls I2C
 * flags of the running transfer: one data byte is moved (TXIS / RXNE) and sequencing events and
 * errors are processed by I2c.c (I2c_Set_XferEvents) per call.
 *
 * \note  Master stretches SCL while TXDR is empty / RXDR is full - a slow task only slows the
 *        transfer down, no data is lost.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I2c_Poll.h"                       /* Self include                   */
#include "I2c.h"                            /* Module private interface       */
#include "Stm32_i2c.h"                      /* I2C RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks polling related part of the data handling configuration
 *
 * \note  Polling mode has no mode specific resources - every peripheral is supported.
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Poll_Check_Config( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT > periphId   ) &&
        ( I2C_NULL_PTR  != dataConfig )    )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes polling data transfer (no HW resource is needed)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Poll_XferInit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes polling data transfer (no HW resource is used)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Poll_XferDeinit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts polling data transfer (bytes are moved by I2c_Task())
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Poll_XferStart( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops polling data transfer (I2c_Task() stops polling when the transfer state in I2c.c
 *        is inactive, no HW resource is used)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Poll_XferStop( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Polls I2C flags of one peripheral and moves the running polling transfer (called from
 *        I2c_Task())
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Poll_Task( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *      periphReg = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( I2C_REQUEST_OK == retState )
    {
        const i2c_IsrFlags_t isrFlags = LL_I2C_ReadReg( periphReg, SR1 );

        retState = I2c_Set_XferEvents( periphId, isrFlags );
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
