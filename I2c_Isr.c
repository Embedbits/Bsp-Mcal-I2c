/**
 * \author Mr.Nobody
 * \file I2c_Isr.c
 * \ingroup I2c
 * \brief I2c module interrupt data transfer handler
 *
 * In I2C_XFER_MODE_ISR the event and error interrupt are enabled for the whole transfer, the
 * buffer interrupt (TXE / RXNE) is enabled by the common sequencing (I2c.c) only while a data
 * byte is expected. Every interrupt passes a snapshot of SR1 to I2c_Set_XferEvents() which
 * handles addressing, data, end of transfer and errors.
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
 * \brief Checks configuration of I2C_XFER_MODE_ISR (no mode specific parameters)
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if the parameters are valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Check_Config( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
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
 * \brief Initializes interrupt data handling - all I2C interrupt sources are disabled until the
 *        transfer start
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
 * \brief Deinitializes interrupt data handling - all I2C interrupt sources are disabled
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
 * \brief Enables event and error interrupt before the START condition (buffer interrupt is
 *        controlled by the sequencing)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_XferStart( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Isr_Set_ItActive( periphId, I2C_ISR_IT_EVENTS );

    return ( retState );
}


/**
 * \brief Disables all I2C interrupt sources at the end of the transfer
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
 * \brief Enables I2C interrupt sources (CR2) with read-back verification
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param itMask   [in]: Interrupt sources, combination of \ref i2c_IsrIt_t values
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Set_ItActive( i2c_PeriphId_t periphId, i2c_IsrItMask_t itMask )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *      periphReg = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( ( I2C_REQUEST_OK == retState                                    ) &&
        ( 0u             == ( itMask & ~( (i2c_IsrItMask_t)I2C_ISR_IT_ALL ) ) )    )
    {
        SET_BIT( periphReg->CR2, itMask );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = READ_BIT( periphReg->CR2, itMask );

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
 * \brief Disables I2C interrupt sources (CR2) with read-back verification
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param itMask   [in]: Interrupt sources, combination of \ref i2c_IsrIt_t values
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Set_ItInactive( i2c_PeriphId_t periphId, i2c_IsrItMask_t itMask )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *      periphReg = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( ( I2C_REQUEST_OK == retState                                    ) &&
        ( 0u             == ( itMask & ~( (i2c_IsrItMask_t)I2C_ISR_IT_ALL ) ) )    )
    {
        CLEAR_BIT( periphReg->CR2, itMask );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = READ_BIT( periphReg->CR2, itMask );

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
 * \brief I2C event and error interrupt handler (ISR and DMA mode) - SR1 snapshot is processed by
 *        the common sequencing
 *
 * \note  SR1 is read once - the read is the first step of the clearing sequences of SB, ADDR,
 *        ADD10 and BTF, which are completed by the sequencing.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Isr_Handler( i2c_PeriphId_t periphId )
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
