/**
 * \author Mr.Nobody
 * \file I2c_Dma.c
 * \ingroup I2c
 * \brief I2c module DMA data transfer handler
 *
 * I2C_XFER_MODE_DMA - data bytes are moved between the request buffers and I2C data registers
 * by two GPDMA channels (8-bit, one block per phase):
 * - Transmission: memory -> TXDR, TxSize bytes (requested by TXIS)
 * - Reception:    RXDR -> memory, RxSize bytes (requested by RXNE)
 *
 * Both channels are armed at the transfer start. Transfer sequencing (reload, repeated START,
 * NACK, STOP) and bus errors are processed by the I2C interrupt (I2c_Isr.c -> I2c.c), GPDMA
 * interrupts report only DMA errors. At the end of a successful transfer all bytes must have
 * been moved by the channels (I2c_Dma_Check_Done).
 *
 * GPDMA callbacks have no parameter, so every peripheral has its own set of handlers generated
 * by I2C_DMA_DEFINE_HANDLERS().
 *
 */
/* ============================== INCLUDES ================================== */
#include "I2c_Dma.h"                        /* Self include                   */
#include "I2c_Isr.h"                        /* I2C interrupt handler          */
#include "I2c.h"                            /* Module private interface       */
#include "Gpdma_Port.h"                     /* GPDMA Mcal layer include       */
#include "Stm32_i2c.h"                      /* I2C RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/** \brief Data transfer direction handled by a GPDMA channel */
typedef enum
{
    I2C_DMA_DIR_TX = 0u, /**< Transmission (memory to TXDR) */
    I2C_DMA_DIR_RX,      /**< Reception (RXDR to memory)    */
    I2C_DMA_DIR_CNT      /**< Count of directions           */
}   i2c_DmaDir_t;


/** \brief GPDMA handlers of one peripheral (registered in GPDMA module) */
typedef struct
{
    gpdma_IsrErrCallback *TxErrorIsr; /**< Transmission transfer error handler */
    gpdma_IsrErrCallback *RxErrorIsr; /**< Reception transfer error handler    */
}   i2c_DmaIsrConfig_t;


/** \brief GPDMA channel ownership (GPDMA channel can not be de-initialized separately) */
typedef struct
{
    i2c_FunctionState_t Initialized; /**< GPDMA channel was initialized for the direction */
    i2c_DmaPeriphId_t   PeriphId;    /**< Initialized GPDMA peripheral                    */
    i2c_DmaChannelId_t  ChannelId;   /**< Initialized GPDMA channel                       */
}   i2c_DmaChannelState_t;


/** \brief GPDMA error bit reporting */
typedef struct
{
    gpdma_ErrorMaskId_t DmaError; /**< GPDMA error bit            */
    i2c_XferErrorId_t   ErrorId;  /**< Error reported to the user */
}   i2c_DmaErrorConfig_t;


/** \brief Index of \ref i2c_DmaErrorConfig_t items */
typedef enum
{
    I2C_DMA_ERR_IDX_TRANSFER = 0u,  /**< Transfer error           */
    I2C_DMA_ERR_IDX_CONFIG,         /**< Configuration error      */
    I2C_DMA_ERR_IDX_CONFIG_UPDATE,  /**< Configuration update     */
    I2C_DMA_ERR_IDX_TRIG_OVERRUN,   /**< Trigger overrun          */
    I2C_DMA_ERR_IDX_CNT             /**< Count of reported errors */
}   i2c_DmaErrIdx_t;

/* =============================== MACROS =================================== */

/**
 * \brief Declares GPDMA handlers of one I2C peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. I2c1)
 */
#define I2C_DMA_DECLARE_HANDLERS( name )                                                \
    static void I2c_Dma_##name##_TxError ( gpdma_ErrorMaskId_t errorMask );             \
    static void I2c_Dma_##name##_RxError ( gpdma_ErrorMaskId_t errorMask )

/**
 * \brief Defines GPDMA handlers of one I2C peripheral - every handler forwards the event with the
 *        peripheral identification to the common processing function
 *
 * \param name     [in]: Peripheral name used in handler names (e.g. I2c1)
 * \param periphId [in]: Peripheral identification, value from \ref i2c_PeriphId_t
 */
#define I2C_DMA_DEFINE_HANDLERS( name, periphId )                                                          \
    static void I2c_Dma_##name##_TxError( gpdma_ErrorMaskId_t errorMask ) { (void)I2c_Dma_XferError( periphId, errorMask ); } \
    static void I2c_Dma_##name##_RxError( gpdma_ErrorMaskId_t errorMask ) { (void)I2c_Dma_XferError( periphId, errorMask ); }

/**
 * \brief Handler table entry of one I2C peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. I2c1)
 */
#define I2C_DMA_ISR_CONFIG( name )                                                      \
    { .TxErrorIsr = I2c_Dma_##name##_TxError, .RxErrorIsr = I2c_Dma_##name##_RxError }

/* ======================== FORWARD DECLARATIONS ============================ */

static i2c_RequestState_t I2c_Dma_Set_ChannelInit ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_ChannelOff  ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_Transfer    ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_Request     ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, i2c_FunctionState_t reqState );
static i2c_RequestState_t I2c_Dma_Set_Stop        ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Get_Remaining   ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, gpdma_BlockSize_t * const remaining );
static i2c_RequestState_t I2c_Dma_XferError       ( i2c_PeriphId_t periphId, gpdma_ErrorMaskId_t errorMask );

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

/** Count of data items transferred per DMA request (single transfer) */
#define I2C_DMA_BURST_LEN            ( 1u )

/** Count of transfers in GPDMA transfer list (one block) */
#define I2C_DMA_TRANSFERS_CNT        ( 1u )

/** GPDMA errors reported to the user */
#define I2C_DMA_ERROR_MASK           ( GPDMA_ERROR_TRANSFER      | \
                                       GPDMA_ERROR_CONFIG_UPDATE | \
                                       GPDMA_ERROR_CONFIG_ERROR  | \
                                       GPDMA_ERROR_TRIG_OVERRUN    )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief i2c_PeriphId_t -> GPDMA handlers */
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


/** \brief GPDMA error bits and reported errors */
static const i2c_DmaErrorConfig_t i2c_DmaErrorConfig[ I2C_DMA_ERR_IDX_CNT ] =
{
    [I2C_DMA_ERR_IDX_TRANSFER]      = { .DmaError = GPDMA_ERROR_TRANSFER,      .ErrorId = I2C_XFER_ERROR_DMA_TRANSFER        },
    [I2C_DMA_ERR_IDX_CONFIG]        = { .DmaError = GPDMA_ERROR_CONFIG_ERROR,  .ErrorId = I2C_XFER_ERROR_DMA_CONFIG          },
    [I2C_DMA_ERR_IDX_CONFIG_UPDATE] = { .DmaError = GPDMA_ERROR_CONFIG_UPDATE, .ErrorId = I2C_XFER_ERROR_DMA_CONFIG_UPDATE   },
    [I2C_DMA_ERR_IDX_TRIG_OVERRUN]  = { .DmaError = GPDMA_ERROR_TRIG_OVERRUN,  .ErrorId = I2C_XFER_ERROR_DMA_TRIGGER_OVERRUN },
};


/** \brief GPDMA transfer lists (must be static, used by GPDMA HW) */
static gpdma_XferList_t i2c_DmaXferList[ I2C_PERIPH_CNT ][ I2C_DMA_DIR_CNT ];


/** \brief GPDMA channel ownership per peripheral and direction */
static i2c_DmaChannelState_t i2c_DmaChannelState[ I2C_PERIPH_CNT ][ I2C_DMA_DIR_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the data handling configuration
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if DMA identifications and priorities are valid and the
 *         transmit channel differs from the receive channel. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_Check_Config( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT > periphId   ) &&
        ( I2C_NULL_PTR  != dataConfig )    )
    {
        if( ( I2C_DMA_PERIPH_CNT            > dataConfig->TxDmaPeriphId           ) &&
            ( I2C_DMA_CHANNEL_CNT           > dataConfig->TxDmaChannelId          ) &&
            ( (uint32_t)GPDMA_PRIORITY_CNT  > (uint32_t)dataConfig->TxDmaPriority ) &&
            ( I2C_DMA_PERIPH_CNT            > dataConfig->RxDmaPeriphId           ) &&
            ( I2C_DMA_CHANNEL_CNT           > dataConfig->RxDmaChannelId          ) &&
            ( (uint32_t)GPDMA_PRIORITY_CNT  > (uint32_t)dataConfig->RxDmaPriority ) &&
            ( ( dataConfig->TxDmaPeriphId  != dataConfig->RxDmaPeriphId  ) ||
              ( dataConfig->TxDmaChannelId != dataConfig->RxDmaChannelId )    )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            /* DMA configuration is invalid or both directions share one channel */
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
 * \brief Initializes DMA data transfer - GPDMA channels of both directions, DMA requests and I2C
 *        interrupt sources are disabled until the transfer start
 *
 * \note  A GPDMA channel already initialized for the direction by a previous initialization is
 *        reused (GPDMA module does not support de-initialization of a single channel).
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
 * \brief Deinitializes DMA data transfer - GPDMA channels and their interrupts, DMA requests and
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
 * \brief Starts DMA data transfer - GPDMA channels of the used directions are armed, I2C DMA
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

    if( ( I2C_REQUEST_OK == retState               ) &&
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

    if( ( I2C_REQUEST_OK == retState               ) &&
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
 * \brief Stops DMA data transfer - GPDMA channels, I2C DMA requests and I2C interrupt sources
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
 * \brief Checks that GPDMA channels moved all bytes of the transfer request (remaining block size
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
    gpdma_BlockSize_t   txRemaining = 0u;
    gpdma_BlockSize_t   rxRemaining = 0u;

    retState = I2c_Get_XferContext( periphId, &xferCtx );

    if( ( I2C_REQUEST_OK == retState               ) &&
        ( 0u              < xferCtx->Request.TxSize )    )
    {
        retState = I2c_Dma_Get_Remaining( periphId, I2C_DMA_DIR_TX, &txRemaining );
    }
    else
    {
        /* Invalid peripheral identification or no write phase */
    }

    if( ( I2C_REQUEST_OK == retState               ) &&
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
 * \brief Initializes (or reuses) the GPDMA channel of one direction and enables its interrupt
 *        (only error handler is registered)
 *
 * - Transmission: memory (increment) -> TXDR (static)
 * - Reception:    RXDR (static) -> memory (increment)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_ChannelInit( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t  retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *       periphReg = I2C_NULL_PTR;
    i2c_XferContext_t * xferCtx   = I2C_NULL_PTR;
    gpdma_PeriphReqId_t txRequest = GPDMA_REQ_I2C1_TX; /* Overwritten by I2c_Get_PeriphDmaReq() */
    gpdma_PeriphReqId_t rxRequest = GPDMA_REQ_I2C1_RX; /* Overwritten by I2c_Get_PeriphDmaReq() */

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

    if( ( I2C_REQUEST_OK == retState ) && ( I2C_DMA_DIR_CNT > dmaDir ) )
    {
        i2c_DmaChannelState_t * const    chState    = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        const i2c_DmaIsrConfig_t * const isrConfig  = &i2c_DmaIsrConfig[ periphId ];
        gpdma_ConfigStruct_t             dmaConfig  = { 0u };
        gpdma_TransferConfig_t           xferConfig = { 0u };
        i2c_DmaPeriphId_t                dmaPeriph  = xferCtx->Config.TxDmaPeriphId;
        i2c_DmaChannelId_t               dmaChannel = xferCtx->Config.TxDmaChannelId;
        i2c_DmaPriority_t                dmaPrio    = xferCtx->Config.TxDmaPriority;
        gpdma_RequestState_t             dmaState   = GPDMA_REQUEST_ERROR;

        /* --- Common transfer parameters (address and block size are set by every start) --- */
        xferConfig.EventMode              = GPDMA_TRANSFER_EVENT_BLOCK;
        xferConfig.TriggerType            = GPDMA_TRG_NOT_USED;
        xferConfig.TriggerMode            = GPDMA_TRIGGER_BLOCK;
        xferConfig.RequestMode            = GPDMA_PERIPH_REQ_SINGLE;
        xferConfig.BlockRepetitionCount   = 0u;
        xferConfig.BlockSize              = 0u;
        xferConfig.SourceDataSize         = GPDMA_DATA_SIZE_8BITS;
        xferConfig.SourceBurstLength      = I2C_DMA_BURST_LEN;
        xferConfig.SourcePortId           = GPDMA_PORT_DEFAULT;
        xferConfig.SourceDataOp           = GPDMA_SRC_DATA_PRESERVE;
        xferConfig.DestinationDataSize    = GPDMA_DATA_SIZE_8BITS;
        xferConfig.DestinationBurstLength = I2C_DMA_BURST_LEN;
        xferConfig.DestinationPortId      = GPDMA_PORT_DEFAULT;
        xferConfig.DestinationDataOp      = GPDMA_DEST_DATA_PRESERVE;

        dmaState = Gpdma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.TransferExecMode    = GPDMA_XFER_EXEC_CONTINUOUS;
        dmaConfig.TransferConfig      = &xferConfig;
        dmaConfig.TransfersCount      = I2C_DMA_TRANSFERS_CNT;
        dmaConfig.XferListAccessMode  = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
        dmaConfig.XferList            = &i2c_DmaXferList[ periphId ][ dmaDir ];
        dmaConfig.TransferLockState   = GPDMA_TRANSFER_LIST_LOCKED;
        dmaConfig.ErrorMask           = I2C_DMA_ERROR_MASK;
        dmaConfig.TransferCompleteIsr = GPDMA_NULL_PTR;
        dmaConfig.HalfTransferIsr     = GPDMA_NULL_PTR;

        if( I2C_DMA_DIR_TX == dmaDir )
        {
            xferConfig.Direction           = GPDMA_DIR_MEMORY_TO_PERIPH;
            xferConfig.RequestSource       = txRequest;
            xferConfig.SourceAddr          = 0u;
            xferConfig.SourceAddrMode      = GPDMA_ADDR_INCREMENT;
            xferConfig.DestinationAddr     = (gpdma_DstAddr_t)LL_I2C_DMA_GetRegAddr( periphReg, LL_I2C_DMA_REG_DATA_TRANSMIT );
            xferConfig.DestinationAddrMode = GPDMA_ADDR_STATIC;

            dmaConfig.ErrorIsr             = isrConfig->TxErrorIsr;
        }
        else
        {
            dmaPeriph  = xferCtx->Config.RxDmaPeriphId;
            dmaChannel = xferCtx->Config.RxDmaChannelId;
            dmaPrio    = xferCtx->Config.RxDmaPriority;

            xferConfig.Direction           = GPDMA_DIR_PERIPH_TO_MEMORY;
            xferConfig.RequestSource       = rxRequest;
            xferConfig.SourceAddr          = (gpdma_SrcAddr_t)LL_I2C_DMA_GetRegAddr( periphReg, LL_I2C_DMA_REG_DATA_RECEIVE );
            xferConfig.SourceAddrMode      = GPDMA_ADDR_STATIC;
            xferConfig.DestinationAddr     = 0u;
            xferConfig.DestinationAddrMode = GPDMA_ADDR_INCREMENT;

            dmaConfig.ErrorIsr             = isrConfig->RxErrorIsr;
        }

        dmaConfig.PeriphId    = (gpdma_PeriphId_t)dmaPeriph;
        dmaConfig.ChannelId   = (gpdma_ChannelId_t)dmaChannel;
        dmaConfig.ChannelPrio = (gpdma_Priority_t)dmaPrio;

        /* --- GPDMA channel initialization or reuse --- */
        if( ( I2C_FUNCTION_ACTIVE == chState->Initialized ) &&
            ( dmaPeriph           == chState->PeriphId    ) &&
            ( dmaChannel          == chState->ChannelId   )    )
        {
            /* Channel is already configured for this direction - priority is updated */
            dmaState = Gpdma_Set_Priority( dmaConfig.PeriphId, dmaConfig.ChannelId, dmaConfig.ChannelPrio );
        }
        else if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Init( &dmaConfig );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                chState->Initialized = I2C_FUNCTION_ACTIVE;
                chState->PeriphId    = dmaPeriph;
                chState->ChannelId   = dmaChannel;
            }
            else
            {
                /* GPDMA channel initialization failed */
            }
        }
        else
        {
            /* Default configuration is not available */
        }

        /* GPDMA module does not enable the channel interrupt in NVIC by itself */
        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_InterruptActive( dmaConfig.PeriphId, dmaConfig.ChannelId );
        }
        else
        {
            /* GPDMA channel configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief Disables the GPDMA channel of one direction and its interrupt (channel configuration is
 *        kept, see I2c_Dma_Set_ChannelInit())
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems (also if no channel was initialized). Otherwise
 *         returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_ChannelOff( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT > periphId ) && ( I2C_DMA_DIR_CNT > dmaDir ) )
    {
        const i2c_DmaChannelState_t * const chState = &i2c_DmaChannelState[ periphId ][ dmaDir ];

        if( I2C_FUNCTION_ACTIVE == chState->Initialized )
        {
            const gpdma_PeriphId_t  dmaPeriph = (gpdma_PeriphId_t)chState->PeriphId;
            const gpdma_ChannelId_t dmaChan   = (gpdma_ChannelId_t)chState->ChannelId;
            gpdma_RequestState_t    dmaState  = Gpdma_Set_ChannelInactive( dmaPeriph, dmaChan );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_InterruptInactive( dmaPeriph, dmaChan );
            }
            else
            {
                /* Channel could not be disabled */
            }

            if( GPDMA_REQUEST_OK == dmaState )
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
            /* No GPDMA channel was initialized for the direction */
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
 * \brief Arms the GPDMA channel of one direction (block size, memory address, enable)
 *
 * - Transmission: TxSize bytes from TxData
 * - Reception:    RxSize bytes into RxData
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Transfer( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t  retState = I2C_REQUEST_ERROR;
    i2c_XferContext_t * xferCtx  = I2C_NULL_PTR;

    retState = I2c_Get_XferContext( periphId, &xferCtx );

    if( ( I2C_REQUEST_OK      == retState                                          ) &&
        ( I2C_DMA_DIR_CNT      > dmaDir                                            ) &&
        ( I2C_FUNCTION_ACTIVE == i2c_DmaChannelState[ periphId ][ dmaDir ].Initialized ) )
    {
        const i2c_DmaChannelState_t * const chState   = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        const gpdma_PeriphId_t              dmaPeriph = (gpdma_PeriphId_t)chState->PeriphId;
        const gpdma_ChannelId_t             dmaChan   = (gpdma_ChannelId_t)chState->ChannelId;
        gpdma_RequestState_t                dmaState  = GPDMA_REQUEST_ERROR;

        if( I2C_DMA_DIR_TX == dmaDir )
        {
            dmaState = Gpdma_Set_BlockSize( dmaPeriph, dmaChan, (gpdma_BlockSize_t)xferCtx->Request.TxSize );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_SourceAddr( dmaPeriph, dmaChan, (gpdma_SrcAddr_t)xferCtx->Request.TxData );
            }
            else
            {
                /* Block size configuration failed */
            }
        }
        else
        {
            dmaState = Gpdma_Set_BlockSize( dmaPeriph, dmaChan, (gpdma_BlockSize_t)xferCtx->Request.RxSize );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_DestinationAddr( dmaPeriph, dmaChan, (gpdma_DstAddr_t)xferCtx->Request.RxData );
            }
            else
            {
                /* Block size configuration failed */
            }
        }

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_ChannelActive( dmaPeriph, dmaChan );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief Enables / disables I2C DMA request of one direction (TXDMAEN / RXDMAEN)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
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

    if( ( I2C_REQUEST_OK == retState ) && ( I2C_DMA_DIR_CNT > dmaDir ) )
    {
        uint32_t expected = 0u;

        if( ( I2C_DMA_DIR_TX == dmaDir ) && ( I2C_FUNCTION_ACTIVE == reqState ) )
        {
            LL_I2C_EnableDMAReq_TX( periphReg );
            expected = 1u;
        }
        else if( I2C_DMA_DIR_TX == dmaDir )
        {
            LL_I2C_DisableDMAReq_TX( periphReg );
            expected = 0u;
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
 * \brief Stops one direction - GPDMA channel (if initialized) and I2C DMA request are disabled
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Stop( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT > periphId ) && ( I2C_DMA_DIR_CNT > dmaDir ) )
    {
        const i2c_DmaChannelState_t * const chState  = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        gpdma_RequestState_t                dmaState = GPDMA_REQUEST_OK;

        if( I2C_FUNCTION_ACTIVE == chState->Initialized )
        {
            dmaState = Gpdma_Set_ChannelInactive( (gpdma_PeriphId_t)chState->PeriphId, (gpdma_ChannelId_t)chState->ChannelId );
        }
        else
        {
            /* No GPDMA channel was initialized for the direction */
        }

        const i2c_RequestState_t reqState = I2c_Dma_Set_Request( periphId, dmaDir, I2C_FUNCTION_INACTIVE );

        if( ( GPDMA_REQUEST_OK == dmaState ) &&
            ( I2C_REQUEST_OK   == reqState )    )
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
 * \brief Returns count of bytes not yet moved by the GPDMA channel of one direction
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir     [in]: Data transfer direction
 * \param remaining [out]: Pointer to store the remaining block size. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Get_Remaining( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, gpdma_BlockSize_t * const remaining )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT      > periphId                                          ) &&
        ( I2C_DMA_DIR_CNT     > dmaDir                                            ) &&
        ( I2C_NULL_PTR       != remaining                                         ) &&
        ( I2C_FUNCTION_ACTIVE == i2c_DmaChannelState[ periphId ][ dmaDir ].Initialized ) )
    {
        const i2c_DmaChannelState_t * const chState  = &i2c_DmaChannelState[ periphId ][ dmaDir ];
        const gpdma_RequestState_t          dmaState = Gpdma_Get_BlockSize( (gpdma_PeriphId_t)chState->PeriphId, (gpdma_ChannelId_t)chState->ChannelId, remaining );

        if( GPDMA_REQUEST_OK == dmaState )
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
 * \brief DMA error processing - the transfer is aborted and every reported GPDMA error bit is
 *        forwarded as data transfer error
 *
 * \param periphId  [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param errorMask [in]: GPDMA error bit mask
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_XferError( i2c_PeriphId_t periphId, gpdma_ErrorMaskId_t errorMask )
{
    i2c_RequestState_t retState = I2C_REQUEST_OK;

    for( i2c_DmaErrIdx_t errIdx = I2C_DMA_ERR_IDX_TRANSFER; I2C_DMA_ERR_IDX_CNT > errIdx; errIdx ++ )
    {
        if( 0u != ( (uint32_t)errorMask & (uint32_t)i2c_DmaErrorConfig[ errIdx ].DmaError ) )
        {
            /* First error aborts the transfer, further errors are only reported */
            retState = I2c_Set_XferError( periphId, i2c_DmaErrorConfig[ errIdx ].ErrorId );
        }
        else
        {
            /* Error bit is not reported */
        }
    }

    return ( retState );
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
