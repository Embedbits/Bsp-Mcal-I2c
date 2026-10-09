/**
 * \author Mr.Nobody
 * \file I2c_Dma.c
 * \ingroup I2c
 * \brief I2c module DMA data transfer handler
 *
 * I2C_XFER_MODE_DMA - data bytes are moved between the request buffers and I2C data registers
 * by two DMA channels (8-bit, normal mode, fixed request mapping of STM32L4 / DMAMUX1 of STM32L4+)
 * selected by the items of the lists \ref i2c_TxDma_t / \ref i2c_RxDma_t (DMA peripheral, channel and
 * request selection are decoded from the item):
 * - Transmission: memory -> TXDR, TxSize bytes (requested by TXIS)
 * - Reception:    RXDR -> memory, RxSize bytes (requested by RXNE)
 *
 * Both channels are armed at the transfer start. Transfer sequencing (reload, repeated START,
 * NACK, STOP) and bus errors are processed by the I2C interrupt (I2c_Isr.c -> I2c.c), DMA
 * interrupts report only DMA transfer errors. At the end of a successful transfer all bytes must
 * have been moved by the channels (I2c_Dma_Check_Done).
 *
 * \note  The DMA channel stays enabled after a normal mode transfer - it is disabled before it
 *        is armed again and by the stop of the transfer.
 *
 * DMA callbacks have no parameter, so every peripheral has its own set of handlers generated
 * by I2C_DMA_DEFINE_HANDLERS().
 *
 */
/* ============================== INCLUDES ================================== */
#include "I2c_Dma.h"                        /* Self include                   */
#include "I2c_Isr.h"                        /* I2C interrupt handler          */
#include "I2c.h"                            /* Module private interface       */
#include "Dma_Port.h"                       /* DMA Mcal layer include         */
#include "Stm32_i2c.h"                      /* I2C RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/** \brief Data transfer direction handled by a DMA channel */
typedef enum
{
    I2C_DMA_DIR_TX = 0u, /**< Transmission (memory to TXDR) */
    I2C_DMA_DIR_RX,      /**< Reception (RXDR to memory)    */
    I2C_DMA_DIR_CNT      /**< Count of directions           */
}   i2c_DmaDir_t;


/** \brief DMA handlers of one peripheral (registered in DMA module) */
typedef struct
{
    dma_IsrCallback TxErrorIsr; /**< Transmission transfer error handler */
    dma_IsrCallback RxErrorIsr; /**< Reception transfer error handler    */
}   i2c_DmaIsrConfig_t;


/** \brief DMA channel ownership */
typedef struct
{
    i2c_FunctionState_t Initialized; /**< DMA channel was initialized for the direction */
    i2c_DmaPeriphId_t   PeriphId;    /**< Initialized DMA peripheral                    */
    i2c_DmaChannelId_t  ChannelId;   /**< Initialized DMA channel                       */
}   i2c_DmaChannelState_t;

/* =============================== MACROS =================================== */

/**
 * \brief Declares DMA handlers of one I2C peripheral
 *
 * \param name [in]: Peripheral name used in the handler names (e.g. I2c1)
 */
#define I2C_DMA_DECLARE_HANDLERS( name )                                                \
    static void I2c_Dma_##name##_TxError ( void );                                      \
    static void I2c_Dma_##name##_RxError ( void )

/**
 * \brief Defines DMA handlers of one I2C peripheral - every handler forwards the event with the
 *        peripheral identification
 *
 * \param name     [in]: Peripheral name used in the handler names (e.g. I2c1)
 * \param periphId [in]: I2C peripheral identification
 */
#define I2C_DMA_DEFINE_HANDLERS( name, periphId )                                                          \
    static void I2c_Dma_##name##_TxError( void ) { (void)I2c_Dma_XferError( periphId ); }                  \
    static void I2c_Dma_##name##_RxError( void ) { (void)I2c_Dma_XferError( periphId ); }

/**
 * \brief Initializer of \ref i2c_DmaIsrConfig_t of one I2C peripheral
 *
 * \param name [in]: Peripheral name used in the handler names (e.g. I2c1)
 */
#define I2C_DMA_ISR_CONFIG( name )                                                      \
    { .TxErrorIsr = I2c_Dma_##name##_TxError, .RxErrorIsr = I2c_Dma_##name##_RxError }

/* ======================== FORWARD DECLARATIONS ============================ */

static i2c_RequestState_t I2c_Dma_Check_Code      ( i2c_PeriphId_t periphId, i2c_DmaCode_t dmaCode );
static i2c_RequestState_t I2c_Dma_Set_ChannelInit ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_ChannelOff  ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_Transfer    ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_Request     ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, i2c_FunctionState_t reqState );
static i2c_RequestState_t I2c_Dma_Set_Stop        ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Get_Remaining   ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, dma_DataCount_t * const remaining );
static i2c_RequestState_t I2c_Dma_XferError       ( i2c_PeriphId_t periphId );

#ifdef I2C1
I2C_DMA_DECLARE_HANDLERS( I2c1 );
#endif /* I2C1 */
#ifdef I2C2
I2C_DMA_DECLARE_HANDLERS( I2c2 );
#endif /* I2C2 */
#ifdef I2C3
I2C_DMA_DECLARE_HANDLERS( I2c3 );
#endif /* I2C3 */
#ifdef I2C4
I2C_DMA_DECLARE_HANDLERS( I2c4 );
#endif /* I2C4 */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief i2c_PeriphId_t -> DMA handlers */
static const i2c_DmaIsrConfig_t i2c_DmaIsrConfig[ ] =
{
#ifdef I2C1
    I2C_DMA_ISR_CONFIG( I2c1 ),
#endif /* I2C1 */
#ifdef I2C2
    I2C_DMA_ISR_CONFIG( I2c2 ),
#endif /* I2C2 */
#ifdef I2C3
    I2C_DMA_ISR_CONFIG( I2c3 ),
#endif /* I2C3 */
#ifdef I2C4
    I2C_DMA_ISR_CONFIG( I2c4 ),
#endif /* I2C4 */
};

_Static_assert( I2C_PERIPH_CNT == ( sizeof(i2c_DmaIsrConfig) / sizeof(i2c_DmaIsrConfig_t) ), "I2c: i2c_DmaIsrConfig has incorrect size." );


/** \brief DMA channel ownership per peripheral and direction */
static i2c_DmaChannelState_t i2c_DmaChannelState[ I2C_PERIPH_CNT ][ I2C_DMA_DIR_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the data handling configuration
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if the DMA channels are items of the lists \ref i2c_TxDma_t /
 *         \ref i2c_RxDma_t of the I2C peripheral, the priorities are valid and the transmit channel
 *         differs from the receive channel. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_Check_Config( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId   ) &&
        ( I2C_NULL_PTR   != dataConfig )    )
    {
        const i2c_DmaCode_t      txCode  = (i2c_DmaCode_t)dataConfig->TxDma;
        const i2c_DmaCode_t      rxCode  = (i2c_DmaCode_t)dataConfig->RxDma;
        const i2c_RequestState_t txState = I2c_Dma_Check_Code( periphId, txCode );
        const i2c_RequestState_t rxState = I2c_Dma_Check_Code( periphId, rxCode );

        if( ( I2C_REQUEST_OK             == txState                            ) &&
            ( (uint32_t)DMA_PRIORITY_CNT >  (uint32_t)dataConfig->TxDmaPriority ) &&
            ( I2C_REQUEST_OK             == rxState                            ) &&
            ( (uint32_t)DMA_PRIORITY_CNT >  (uint32_t)dataConfig->RxDmaPriority )    )
        {
            if( I2C_DMA_BIT_MASK_DECODE_DMA( txCode ) != I2C_DMA_BIT_MASK_DECODE_DMA( rxCode ) )
            {
                retState = I2C_REQUEST_OK;
            }
            else if( I2C_DMA_BIT_MASK_DECODE_CHANNEL( txCode ) != I2C_DMA_BIT_MASK_DECODE_CHANNEL( rxCode ) )
            {
                retState = I2C_REQUEST_OK;
            }
            else
            {
                /* Both directions share one channel */
                retState = I2C_REQUEST_ERROR;
            }
        }
        else
        {
            /* DMA configuration is invalid */
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA data transfer - DMA channels of both directions, DMA requests and I2C
 *        interrupt sources are disabled until the transfer start
 *
 * \pre   Transfer context of the peripheral contains the data handling configuration.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_XferInit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Dma_Set_ChannelInit( periphId, I2C_DMA_DIR_TX );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Dma_Set_ChannelInit( periphId, I2C_DMA_DIR_RX );
    }
    else
    {
        /* Transmit channel initialization failed */
    }

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Dma_XferStop( periphId );
    }
    else
    {
        /* Receive channel initialization failed */
    }

    return ( retState );
}


/**
 * \brief Deinitializes DMA data transfer - DMA channels and their interrupts, DMA requests and
 *        I2C interrupt sources are disabled. All steps are executed, any failure is reported.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_XferDeinit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    const i2c_RequestState_t stopState = I2c_Dma_XferStop( periphId );
    const i2c_RequestState_t txState   = I2c_Dma_Set_ChannelOff( periphId, I2C_DMA_DIR_TX );
    const i2c_RequestState_t rxState   = I2c_Dma_Set_ChannelOff( periphId, I2C_DMA_DIR_RX );

    if( ( I2C_REQUEST_OK == stopState ) &&
        ( I2C_REQUEST_OK == txState   ) &&
        ( I2C_REQUEST_OK == rxState   )    )
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
 * \brief Starts DMA data transfer - DMA channels of the used directions are armed, I2C DMA
 *        requests and sequencing / error interrupts are enabled
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_XferStart( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t  retState = I2C_REQUEST_ERROR;
    i2c_XferContext_t * xferCtx  = I2C_NULL_PTR;

    retState = I2c_Get_XferContext( periphId, &xferCtx );

    if( ( I2C_REQUEST_OK == retState                ) &&
        ( 0u              < xferCtx->Request.TxSize )    )
    {
        retState = I2c_Dma_Set_Transfer( periphId, I2C_DMA_DIR_TX );

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Dma_Set_Request( periphId, I2C_DMA_DIR_TX, I2C_FUNCTION_ACTIVE );
        }
        else
        {
            /* Transmit channel could not be armed */
        }
    }
    else
    {
        /* Invalid peripheral identification or no write phase */
    }

    if( ( I2C_REQUEST_OK == retState                ) &&
        ( 0u              < xferCtx->Request.RxSize )    )
    {
        retState = I2c_Dma_Set_Transfer( periphId, I2C_DMA_DIR_RX );

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Dma_Set_Request( periphId, I2C_DMA_DIR_RX, I2C_FUNCTION_ACTIVE );
        }
        else
        {
            /* Receive channel could not be armed */
        }
    }
    else
    {
        /* Previous step failed or no read phase */
    }

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Isr_Set_ItActive( periphId, I2C_ISR_IT_EVENTS );
    }
    else
    {
        /* DMA could not be started */
    }

    return ( retState );
}


/**
 * \brief Stops DMA data transfer - DMA channels, I2C DMA requests and I2C interrupt sources
 *        are disabled. All steps are executed, any failure is reported.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_XferStop( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    const i2c_RequestState_t itState = I2c_Isr_Set_ItInactive( periphId, I2C_ISR_IT_ALL );
    const i2c_RequestState_t txState = I2c_Dma_Set_Stop( periphId, I2C_DMA_DIR_TX );
    const i2c_RequestState_t rxState = I2c_Dma_Set_Stop( periphId, I2C_DMA_DIR_RX );

    if( ( I2C_REQUEST_OK == itState ) &&
        ( I2C_REQUEST_OK == txState ) &&
        ( I2C_REQUEST_OK == rxState )    )
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
 * \brief Checks that DMA channels moved all bytes of the transfer request (remaining data count
 *        of the used directions is zero)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Returns \ref I2C_REQUEST_OK if all bytes were moved. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_Check_Done( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t  retState    = I2C_REQUEST_ERROR;
    i2c_XferContext_t * xferCtx     = I2C_NULL_PTR;
    dma_DataCount_t     txRemaining = 0u;
    dma_DataCount_t     rxRemaining = 0u;

    retState = I2c_Get_XferContext( periphId, &xferCtx );

    if( ( I2C_REQUEST_OK == retState                ) &&
        ( 0u              < xferCtx->Request.TxSize )    )
    {
        retState = I2c_Dma_Get_Remaining( periphId, I2C_DMA_DIR_TX, &txRemaining );
    }
    else
    {
        /* Invalid peripheral identification or no write phase */
    }

    if( ( I2C_REQUEST_OK == retState                ) &&
        ( 0u              < xferCtx->Request.RxSize )    )
    {
        retState = I2c_Dma_Get_Remaining( periphId, I2C_DMA_DIR_RX, &rxRemaining );
    }
    else
    {
        /* Previous step failed or no read phase */
    }

    if( ( I2C_REQUEST_OK == retState    ) &&
        ( 0u             == txRemaining ) &&
        ( 0u             == rxRemaining )    )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks an item of the DMA channel lists - the item has to belong to the I2C peripheral and its
 *        DMA peripheral and channel have to exist
 *
 * The request mapping of the channel (STM32L4 DMA_CSELR) is checked by Dma_Init().
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaCode [in]: Item of \ref i2c_TxDma_t / \ref i2c_RxDma_t (encoded DMA channel)
 *
 * \return Returns \ref I2C_REQUEST_OK if the item is a DMA channel of the I2C peripheral. Otherwise
 *         returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Check_Code( i2c_PeriphId_t periphId, i2c_DmaCode_t dmaCode )
{
    i2c_RequestState_t retState    = I2C_REQUEST_ERROR;
    const uint32_t     codePeriph  = I2C_DMA_BIT_MASK_DECODE_PERIPH( dmaCode );
    const uint32_t     codeDmaId   = I2C_DMA_BIT_MASK_DECODE_DMA( dmaCode );
    const uint32_t     codeChannel = I2C_DMA_BIT_MASK_DECODE_CHANNEL( dmaCode );

    if( ( (uint32_t)periphId           == codePeriph  ) &&
        ( (uint32_t)I2C_DMA_PERIPH_CNT  > codeDmaId   ) &&
        ( (uint32_t)I2C_DMA_CHANNEL_CNT > codeChannel )    )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        /* Item of another peripheral, unused item or invalid code */
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes the DMA channel of one direction and enables its error interrupt (only
 *        the error handler is registered)
 *
 * - Transmission: memory (increment) -> TXDR (static), DMA request I2Cx_TX
 * - Reception:    RXDR (static) -> memory (increment), DMA request I2Cx_RX
 *
 * A channel already initialized for the direction with the same DMA peripheral / channel is
 * reused (priority updated), otherwise it is initialized by Dma_Init().
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref i2c_DmaDir_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_ChannelInit( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t  retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *       periphReg = I2C_NULL_PTR;
    i2c_XferContext_t * xferCtx   = I2C_NULL_PTR;
    dma_PeriphReqId_t   txRequest = DMA_REQ_I2C1_TX; /* Overwritten by I2c_Get_PeriphDmaReq() */
    dma_PeriphReqId_t   rxRequest = DMA_REQ_I2C1_RX; /* Overwritten by I2c_Get_PeriphDmaReq() */

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
        retState = I2c_Get_PeriphDmaReq( periphId, &txRequest, &rxRequest );
    }
    else
    {
        /* Transfer context is not available */
    }

    if( ( I2C_REQUEST_OK  == retState ) &&
        ( I2C_DMA_DIR_CNT  > dmaDir   )    )
    {
        i2c_DmaChannelState_t * const    chState    = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        const i2c_DmaIsrConfig_t * const isrConfig  = &i2c_DmaIsrConfig[ periphId ];
        i2c_DmaCode_t                    dmaCode    = (i2c_DmaCode_t)xferCtx->Config.TxDma;
        i2c_DmaPriority_t                dmaPrio    = xferCtx->Config.TxDmaPriority;
        dma_ConfigStruct_t               dmaConfig;
        dma_RequestState_t               dmaState   = Dma_Get_DefaultConfig( &dmaConfig );

        if( I2C_DMA_DIR_RX == dmaDir )
        {
            dmaCode = (i2c_DmaCode_t)xferCtx->Config.RxDma;
            dmaPrio = xferCtx->Config.RxDmaPriority;
        }
        else
        {
            /* Transmission channel */
        }

        const i2c_DmaPeriphId_t  dmaPeriph  = (i2c_DmaPeriphId_t)I2C_DMA_BIT_MASK_DECODE_DMA( dmaCode );
        const i2c_DmaChannelId_t dmaChannel = (i2c_DmaChannelId_t)I2C_DMA_BIT_MASK_DECODE_CHANNEL( dmaCode );

        /* --- DMA channel initialization or reuse (address and count are set by every start) --- */
        if( ( I2C_FUNCTION_ACTIVE == chState->Initialized ) &&
            ( dmaPeriph           == chState->PeriphId    ) &&
            ( dmaChannel          == chState->ChannelId   )    )
        {
            /* Channel is already configured for this direction - priority is updated */
            dmaState = Dma_Set_Priority( (dma_PeriphId_t)dmaPeriph, (dma_ChannelId_t)dmaChannel, (dma_Priority_t)dmaPrio );
        }
        else if( DMA_REQUEST_OK == dmaState )
        {
            dmaConfig.DmaPeriphId              = (dma_PeriphId_t)dmaPeriph;
            dmaConfig.DmaChannel               = (dma_ChannelId_t)dmaChannel;
            dmaConfig.TransferMode             = DMA_TRANSFER_MODE_NORMAL;
            dmaConfig.PeriphAddrIncrement      = DMA_PERIPH_ADDR_STATIC;
            dmaConfig.MemoryAddrIncrement      = DMA_MEMORY_ADDR_INCREMENT;
            dmaConfig.PeriphTransferSize       = DMA_TRANSFER_SIZE_8BIT;
            dmaConfig.MemoryTransferSize       = DMA_TRANSFER_SIZE_8BIT;
            dmaConfig.MemoryAddress            = 0u;
            dmaConfig.DataCount                = 0u;
            dmaConfig.Priority                 = (dma_Priority_t)dmaPrio;
            dmaConfig.TransferCompleteCallback = DMA_NULL_PTR;
            dmaConfig.HalfTransferCallback     = DMA_NULL_PTR;

            if( I2C_DMA_DIR_TX == dmaDir )
            {
                dmaConfig.Direction             = DMA_DIR_MEMORY_TO_PERIPH;
                dmaConfig.PeripheralReqId       = txRequest;
                dmaConfig.PeriphAddress         = (dma_PeriphAddr_t)LL_I2C_DMA_GetRegAddr( periphReg, LL_I2C_DMA_REG_DATA_TRANSMIT );
                dmaConfig.TransferErrorCallback = isrConfig->TxErrorIsr;
            }
            else
            {
                dmaConfig.Direction             = DMA_DIR_PERIPH_TO_MEMORY;
                dmaConfig.PeripheralReqId       = rxRequest;
                dmaConfig.PeriphAddress         = (dma_PeriphAddr_t)LL_I2C_DMA_GetRegAddr( periphReg, LL_I2C_DMA_REG_DATA_RECEIVE );
                dmaConfig.TransferErrorCallback = isrConfig->RxErrorIsr;
            }

            dmaState = Dma_Init( &dmaConfig );

            if( DMA_REQUEST_OK == dmaState )
            {
                chState->Initialized = I2C_FUNCTION_ACTIVE;
                chState->PeriphId    = dmaPeriph;
                chState->ChannelId   = dmaChannel;
            }
            else
            {
                /* DMA channel initialization failed */
            }
        }
        else
        {
            /* Default configuration is not available */
        }

        /* Only the transfer error is reported by the DMA interrupt */
        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferErrorIrqActive( (dma_PeriphId_t)dmaPeriph, (dma_ChannelId_t)dmaChannel );
        }
        else
        {
            /* DMA channel configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_InterruptActive( (dma_PeriphId_t)dmaPeriph, (dma_ChannelId_t)dmaChannel );
        }
        else
        {
            /* Error interrupt could not be enabled */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables the DMA channel of one direction and its interrupts (channel ownership is
 *        released)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref i2c_DmaDir_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems (also if no channel was initialized). Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_ChannelOff( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId ) &&
        ( I2C_DMA_DIR_CNT > dmaDir   )    )
    {
        i2c_DmaChannelState_t * const chState = &i2c_DmaChannelState[ periphId ][ dmaDir ];

        if( I2C_FUNCTION_ACTIVE == chState->Initialized )
        {
            const dma_PeriphId_t  dmaPeriph = (dma_PeriphId_t)chState->PeriphId;
            const dma_ChannelId_t dmaChan   = (dma_ChannelId_t)chState->ChannelId;
            dma_RequestState_t    dmaState  = Dma_Set_TransferInactive( dmaPeriph, dmaChan );

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_TransferErrorIrqInactive( dmaPeriph, dmaChan );
            }
            else
            {
                /* Channel could not be disabled */
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                dmaState = Dma_Set_InterruptInactive( dmaPeriph, dmaChan );
            }
            else
            {
                /* Error interrupt could not be disabled */
            }

            if( DMA_REQUEST_OK == dmaState )
            {
                chState->Initialized = I2C_FUNCTION_INACTIVE;
                retState             = I2C_REQUEST_OK;
            }
            else
            {
                retState = I2C_REQUEST_ERROR;
            }
        }
        else
        {
            /* No DMA channel was initialized for the direction */
            retState = I2C_REQUEST_OK;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Arms the DMA channel of one direction for the running request - the channel is disabled
 *        (it stays enabled after a normal mode transfer), memory address and data count are
 *        written and the channel is enabled
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref i2c_DmaDir_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Transfer( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t  retState = I2C_REQUEST_ERROR;
    i2c_XferContext_t * xferCtx  = I2C_NULL_PTR;

    retState = I2c_Get_XferContext( periphId, &xferCtx );

    if( ( I2C_REQUEST_OK      == retState                                              ) &&
        ( I2C_DMA_DIR_CNT      > dmaDir                                                ) &&
        ( I2C_FUNCTION_ACTIVE == i2c_DmaChannelState[ periphId ][ dmaDir ].Initialized )    )
    {
        const i2c_DmaChannelState_t * const chState   = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        const dma_PeriphId_t                dmaPeriph = (dma_PeriphId_t)chState->PeriphId;
        const dma_ChannelId_t               dmaChan   = (dma_ChannelId_t)chState->ChannelId;
        dma_MemoryAddr_t                    memAddr   = (dma_MemoryAddr_t)xferCtx->Request.TxData;
        dma_DataCount_t                     dataCnt   = (dma_DataCount_t)xferCtx->Request.TxSize;
        dma_RequestState_t                  dmaState  = Dma_Set_TransferInactive( dmaPeriph, dmaChan );

        if( I2C_DMA_DIR_RX == dmaDir )
        {
            memAddr = (dma_MemoryAddr_t)xferCtx->Request.RxData;
            dataCnt = (dma_DataCount_t)xferCtx->Request.RxSize;
        }
        else
        {
            /* Transmission data */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryAddr( dmaPeriph, dmaChan, memAddr );
        }
        else
        {
            /* Channel could not be disabled */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_DataCount( dmaPeriph, dmaChan, dataCnt );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferActive( dmaPeriph, dmaChan );
        }
        else
        {
            /* Data count configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables / disables I2C DMA requests of one direction (TXDMAEN / RXDMAEN)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref i2c_DmaDir_t
 * \param reqState [in]: Required state, value from \ref i2c_FunctionState_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Request( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, i2c_FunctionState_t reqState )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *      periphReg = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( ( I2C_REQUEST_OK  == retState ) &&
        ( I2C_DMA_DIR_CNT  > dmaDir   )    )
    {
        uint32_t expected = 0u;

        if( I2C_DMA_DIR_TX == dmaDir )
        {
            if( I2C_FUNCTION_ACTIVE == reqState )
            {
                LL_I2C_EnableDMAReq_TX( periphReg );
                expected = 1u;
            }
            else
            {
                LL_I2C_DisableDMAReq_TX( periphReg );
                expected = 0u;
            }
        }
        else if( I2C_FUNCTION_ACTIVE == reqState )
        {
            LL_I2C_EnableDMAReq_RX( periphReg );
            expected = 1u;
        }
        else
        {
            LL_I2C_DisableDMAReq_RX( periphReg );
            expected = 0u;
        }

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t regValue = 0u;

            if( I2C_DMA_DIR_TX == dmaDir )
            {
                regValue = LL_I2C_IsEnabledDMAReq_TX( periphReg );
            }
            else
            {
                regValue = LL_I2C_IsEnabledDMAReq_RX( periphReg );
            }

            if( expected == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* DMA request state has not yet been applied, keep return state as error */
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
 * \brief Stops one direction - DMA channel and I2C DMA requests are disabled. Both steps are
 *        executed, any failure is reported.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Direction, value from \ref i2c_DmaDir_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Stop( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId ) &&
        ( I2C_DMA_DIR_CNT > dmaDir   )    )
    {
        const i2c_DmaChannelState_t * const chState  = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        dma_RequestState_t                  dmaState = DMA_REQUEST_OK;

        if( I2C_FUNCTION_ACTIVE == chState->Initialized )
        {
            dmaState = Dma_Set_TransferInactive( (dma_PeriphId_t)chState->PeriphId, (dma_ChannelId_t)chState->ChannelId );
        }
        else
        {
            /* No DMA channel was initialized for the direction */
        }

        const i2c_RequestState_t reqState = I2c_Dma_Set_Request( periphId, dmaDir, I2C_FUNCTION_INACTIVE );

        if( ( DMA_REQUEST_OK == dmaState ) &&
            ( I2C_REQUEST_OK == reqState )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns remaining data count of the DMA channel of one direction
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir     [in]: Direction, value from \ref i2c_DmaDir_t
 * \param remaining [out]: Pointer to store the remaining count of bytes. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Get_Remaining( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, dma_DataCount_t * const remaining )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT       > periphId                                              ) &&
        ( I2C_DMA_DIR_CNT      > dmaDir                                                ) &&
        ( I2C_NULL_PTR        != remaining                                             ) &&
        ( I2C_FUNCTION_ACTIVE == i2c_DmaChannelState[ periphId ][ dmaDir ].Initialized )    )
    {
        const i2c_DmaChannelState_t * const chState  = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        const dma_RequestState_t            dmaState = Dma_Get_DataCount( (dma_PeriphId_t)chState->PeriphId, (dma_ChannelId_t)chState->ChannelId, remaining );

        if( DMA_REQUEST_OK == dmaState )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reports DMA transfer error of a peripheral - the transfer is aborted and
 *        \ref I2C_XFER_ERROR_DMA_TRANSFER is reported
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_XferError( i2c_PeriphId_t periphId )
{
    return ( I2c_Set_XferError( periphId, I2C_XFER_ERROR_DMA_TRANSFER ) );
}

/* =========================== INTERRUPT HANDLERS =========================== */

#ifdef I2C1
I2C_DMA_DEFINE_HANDLERS( I2c1, I2C_PERIPH_1 )
#endif /* I2C1 */
#ifdef I2C2
I2C_DMA_DEFINE_HANDLERS( I2c2, I2C_PERIPH_2 )
#endif /* I2C2 */
#ifdef I2C3
I2C_DMA_DEFINE_HANDLERS( I2c3, I2C_PERIPH_3 )
#endif /* I2C3 */
#ifdef I2C4
I2C_DMA_DEFINE_HANDLERS( I2c4, I2C_PERIPH_4 )
#endif /* I2C4 */

/* ================================ TASKS =================================== */
