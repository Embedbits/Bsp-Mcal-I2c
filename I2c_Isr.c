/**
 * \author Mr.Nobody
 * \file I2c_Isr.c
 * \ingroup I2c
 * \brief I2c module interrupt data transfer handler
 *
 * I2C_XFER_MODE_ISR - data bytes are moved by the I2C event interrupt (TXIS / RXNE), transfer
 * sequencing events (TC / TCR / NACKF / STOPF) and bus errors (BERR / ARLO) are processed by
 * I2c.c (I2c_Set_XferEvents). Event and error interrupt share one handler.
 *
 * The handler is used in I2C_XFER_MODE_DMA too - data are moved by DMA there, the interrupt
 * processes only the sequencing events and errors.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I2c_Isr.h"                        /* Self include                   */
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
 * \brief Checks interrupt related part of the data handling configuration
 *
 * \note  Interrupt mode has no mode specific resources - every peripheral is supported.
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Check_Config( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId   ) &&
        ( I2C_NULL_PTR   != dataConfig )    )
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
 * \brief Initializes interrupt data transfer - all I2C interrupt sources are disabled until
 *        the transfer start (NVIC is configured by I2c.c)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_XferInit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Isr_Set_ItInactive( periphId, I2C_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Deinitializes interrupt data transfer - all I2C interrupt sources are disabled
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_XferDeinit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Isr_Set_ItInactive( periphId, I2C_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Starts interrupt data transfer - data (TXIS / RXNE), sequencing and error interrupts
 *        are enabled (TXIS / RXNE are set by HW only in the corresponding phase)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_XferStart( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Isr_Set_ItActive( periphId, I2C_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Stops interrupt data transfer - all I2C interrupt sources are disabled
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_XferStop( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Isr_Set_ItInactive( periphId, I2C_ISR_IT_ALL );

    return ( retState );
}


/**
 * \brief Enables I2C interrupt sources (I2C_CR1 enable bits)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param itMask   [in]: Bit mask of \ref i2c_IsrIt_t values
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Set_ItActive( i2c_PeriphId_t periphId, i2c_IsrItMask_t itMask )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *      periphReg = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( ( I2C_REQUEST_OK == retState                                          ) &&
        ( 0u             == ( itMask & ~( (i2c_IsrItMask_t)I2C_ISR_IT_ALL ) ) )    )
    {
        SET_BIT( periphReg->CR1, itMask );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = READ_BIT( periphReg->CR1, itMask );

            if( itMask == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Interrupt enable has not yet been applied, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables I2C interrupt sources (I2C_CR1 enable bits)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param itMask   [in]: Bit mask of \ref i2c_IsrIt_t values
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Set_ItInactive( i2c_PeriphId_t periphId, i2c_IsrItMask_t itMask )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *      periphReg = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( ( I2C_REQUEST_OK == retState                                          ) &&
        ( 0u             == ( itMask & ~( (i2c_IsrItMask_t)I2C_ISR_IT_ALL ) ) )    )
    {
        CLEAR_BIT( periphReg->CR1, itMask );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = READ_BIT( periphReg->CR1, itMask );

            if( 0u == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Interrupt disable has not yet been applied, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief I2C event / error interrupt processing - data byte is moved in ISR mode, sequencing
 *        events and errors are processed in ISR and DMA mode
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Handler( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t  retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *       periphReg = I2C_NULL_PTR;
    i2c_XferContext_t * xferCtx   = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Get_XferContext( periphId, &xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( I2C_REQUEST_OK == retState )
    {
        const i2c_IsrFlags_t isrFlags = LL_I2C_ReadReg( periphReg, ISR );

        if( I2C_XFER_MODE_ISR == xferCtx->Config.XferMode )
        {
            retState = I2c_Set_XferDataStep( periphId, isrFlags );
        }
        else
        {
            /* Data are moved by DMA */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_XferEvents( periphId, isrFlags );
        }
        else
        {
            /* Data step failed */
        }
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
