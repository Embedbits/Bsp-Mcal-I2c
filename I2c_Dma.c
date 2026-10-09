/**
 * \author Mr.Nobody
 * \file I2c_Dma.c
 * \ingroup I2c
 * \brief I2c module DMA data transfer handler
 *
 * I2C_XFER_MODE_DMA - data bytes are moved by DMA1 streams, the transfer is sequenced by the
 * I2C event and error interrupt (I2c_Isr_Handler):
 * - write phase: transmit stream writes DR on TXE requests, the end of the phase is detected
 *   by BTF with empty stream (I2c_Dma_Check_TxDone)
 * - read phase (2 and more bytes): receive stream reads DR on RXNE requests, LAST makes the I2C
 *   send NACK after the last byte, transfer complete interrupt of the stream ends the transfer
 *   (STOP condition, I2c_Set_XferRxDone)
 * - read phase of 1 byte: DMA can not generate NACK of a single byte (EOT-1 event), the byte is
 *   read by the I2C buffer interrupt
 *
 * The stream of each direction is configured once by the data handling initialization (request
 * channel selection, peripheral address, priority, interrupt callbacks), every transfer start
 * only sets memory address and count of bytes.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I2c_Dma.h"                        /* Self include                   */
#include "I2c_Isr.h"                        /* I2C interrupt handler          */
#include "I2c.h"                            /* Module private interface       */
#include "Dma_Port.h"                       /* DMA Mcal layer include         */
#include "Stm32_i2c.h"                      /* I2C RAL functionality          */
/* ============================== TYPEDEFS ================================== */

/** \brief DMA transfer direction */
typedef enum
{
    I2C_DMA_DIR_TX = 0u, /**< Transmission (memory to DR) */
    I2C_DMA_DIR_RX,      /**< Reception (DR to memory)    */
    I2C_DMA_DIR_CNT      /**< Count of directions         */
}   i2c_DmaDir_t;


/** \brief DMA request map entry - stream connected to the I2C request through channel selection */
typedef struct
{
    i2c_PeriphId_t      PeriphId;  /**< I2C peripheral                       */
    i2c_DmaDir_t        Dir;       /**< Transfer direction of the request    */
    i2c_DmaPeriphId_t   DmaId;     /**< DMA peripheral                       */
    i2c_DmaChannelId_t  StreamId;  /**< DMA stream                           */
    dma_PeriphReqId_t   ChannelSel;/**< Channel selection (CHSEL) of stream  */
}   i2c_DmaReqMap_t;


/** \brief Interrupt callbacks of DMA streams of one I2C peripheral */
typedef struct
{
    dma_IsrCallback TxErrorIsr;    /**< Transmission transfer error handler */
    dma_IsrCallback RxErrorIsr;    /**< Reception transfer error handler    */
    dma_IsrCallback RxCompleteIsr; /**< Reception transfer complete handler */
}   i2c_DmaIsrConfig_t;


/** \brief Stream used by one direction of one I2C peripheral */
typedef struct
{
    i2c_FunctionState_t Initialized; /**< Stream was initialized for the direction */
    dma_PeriphId_t      DmaId;       /**< Initialized DMA peripheral               */
    dma_ChannelId_t     StreamId;    /**< Initialized DMA stream                   */
}   i2c_DmaStreamState_t;


/** \brief Type representing type of DMA requests enable bits of CR2 (DMAEN / LAST) */
typedef uint32_t i2c_DmaCr2Mask_t;

/* =============================== MACROS =================================== */

/** Declares DMA interrupt callbacks of one I2C peripheral */
#define I2C_DMA_DECLARE_HANDLERS( name )                                                \
    static void I2c_Dma_##name##_TxError    ( void );                                   \
    static void I2c_Dma_##name##_RxError    ( void );                                   \
    static void I2c_Dma_##name##_RxComplete ( void )

/** Defines DMA interrupt callbacks of one I2C peripheral */
#define I2C_DMA_DEFINE_HANDLERS( name, periphId )                                                                    \
    static void I2c_Dma_##name##_TxError( void )    { (void)I2c_Set_XferError( periphId, I2C_XFER_ERROR_DMA_TRANSFER ); } \
    static void I2c_Dma_##name##_RxError( void )    { (void)I2c_Set_XferError( periphId, I2C_XFER_ERROR_DMA_TRANSFER ); } \
    static void I2c_Dma_##name##_RxComplete( void ) { (void)I2c_Set_XferRxDone( periphId ); }

/** Initializer of DMA interrupt callbacks of one I2C peripheral */
#define I2C_DMA_ISR_CONFIG( name )                                                      \
    { .TxErrorIsr = I2c_Dma_##name##_TxError, .RxErrorIsr = I2c_Dma_##name##_RxError,   \
      .RxCompleteIsr = I2c_Dma_##name##_RxComplete }

/* ======================== FORWARD DECLARATIONS ============================ */

static i2c_RequestState_t I2c_Dma_Get_ChannelSel  ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, i2c_DmaCode_t dmaCode, dma_PeriphReqId_t * const channelSel );
static i2c_RequestState_t I2c_Dma_Set_StreamInit  ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_StreamOff   ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_Transfer    ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Set_Stop        ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir );
static i2c_RequestState_t I2c_Dma_Get_Remaining   ( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, dma_DataCount_t * const remaining );
static i2c_RequestState_t I2c_Dma_Set_Cr2         ( i2c_PeriphId_t periphId, i2c_DmaCr2Mask_t cr2Mask, i2c_FunctionState_t cr2State );

#ifdef I2C1
I2C_DMA_DECLARE_HANDLERS( I2c1 );
#endif /* I2C1 */
#ifdef I2C2
I2C_DMA_DECLARE_HANDLERS( I2c2 );
#endif /* I2C2 */
#ifdef I2C3
I2C_DMA_DECLARE_HANDLERS( I2c3 );
#endif /* I2C3 */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** CR2 bits of DMA requests: DMAEN (TXE / RXNE requests) and LAST (NACK after the last byte) */
#define I2C_DMA_CR2_ALL              ( I2C_CR2_DMAEN | I2C_CR2_LAST )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/**
 * \brief DMA1 request map of I2C peripherals (RM DMA1 request mapping, source: embassy stm32-data)
 */
static const i2c_DmaReqMap_t i2c_DmaReqMap[ ] =
{
#ifdef I2C1
    { .PeriphId = I2C_PERIPH_1, .Dir = I2C_DMA_DIR_RX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_0, .ChannelSel = DMA_REQ_CHANNEL_1 },
    { .PeriphId = I2C_PERIPH_1, .Dir = I2C_DMA_DIR_RX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_5, .ChannelSel = DMA_REQ_CHANNEL_1 },
    { .PeriphId = I2C_PERIPH_1, .Dir = I2C_DMA_DIR_TX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_6, .ChannelSel = DMA_REQ_CHANNEL_1 },
    { .PeriphId = I2C_PERIPH_1, .Dir = I2C_DMA_DIR_TX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_7, .ChannelSel = DMA_REQ_CHANNEL_1 },
#if defined(I2C_AF_MAP_F410_F423)
    { .PeriphId = I2C_PERIPH_1, .Dir = I2C_DMA_DIR_TX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_1, .ChannelSel = DMA_REQ_CHANNEL_0 },
#endif
#endif /* I2C1 */
#ifdef I2C2
    { .PeriphId = I2C_PERIPH_2, .Dir = I2C_DMA_DIR_RX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_2, .ChannelSel = DMA_REQ_CHANNEL_7 },
    { .PeriphId = I2C_PERIPH_2, .Dir = I2C_DMA_DIR_RX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_3, .ChannelSel = DMA_REQ_CHANNEL_7 },
    { .PeriphId = I2C_PERIPH_2, .Dir = I2C_DMA_DIR_TX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_7, .ChannelSel = DMA_REQ_CHANNEL_7 },
#endif /* I2C2 */
#ifdef I2C3
    { .PeriphId = I2C_PERIPH_3, .Dir = I2C_DMA_DIR_RX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_2, .ChannelSel = DMA_REQ_CHANNEL_3 },
    { .PeriphId = I2C_PERIPH_3, .Dir = I2C_DMA_DIR_TX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_4, .ChannelSel = DMA_REQ_CHANNEL_3 },
#if defined(I2C_AF_MAP_F401) || defined(I2C_AF_MAP_F410_F423) || defined(I2C_AF_MAP_F446)
    { .PeriphId = I2C_PERIPH_3, .Dir = I2C_DMA_DIR_RX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_1, .ChannelSel = DMA_REQ_CHANNEL_1 },
#endif
#if defined(I2C_AF_MAP_F401) || defined(I2C_AF_MAP_F410_F423)
    { .PeriphId = I2C_PERIPH_3, .Dir = I2C_DMA_DIR_TX, .DmaId = I2C_DMA_PERIPH_1, .StreamId = I2C_DMA_CHANNEL_5, .ChannelSel = DMA_REQ_CHANNEL_6 },
#endif
#endif /* I2C3 */
};

/** Count of entries of the DMA request map */
#define I2C_DMA_REQ_MAP_CNT          ( sizeof( i2c_DmaReqMap ) / sizeof( i2c_DmaReqMap_t ) )


/** \brief DMA interrupt callbacks per I2C peripheral */
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
};

_Static_assert( I2C_PERIPH_CNT == ( sizeof(i2c_DmaIsrConfig) / sizeof(i2c_DmaIsrConfig_t) ), "I2c: i2c_DmaIsrConfig has incorrect size." );


/** \brief Initialized streams per peripheral and direction */
static i2c_DmaStreamState_t i2c_DmaStreamState[ I2C_PERIPH_CNT ][ I2C_DMA_DIR_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA specific part of data handling configuration - priorities in range and both
 *        streams are items of the lists \ref i2c_TxDma_t / \ref i2c_RxDma_t of the I2C peripheral
 *        (the streams of the I2C requests of their direction, transmission and reception never
 *        share a stream)
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_Check_Config( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT > periphId   ) &&
        ( I2C_NULL_PTR  != dataConfig )    )
    {
        dma_PeriphReqId_t txChannelSel = DMA_REQ_CHANNEL_0;
        dma_PeriphReqId_t rxChannelSel = DMA_REQ_CHANNEL_0;

        if( ( (uint32_t)DMA_PRIORITY_CNT  > (uint32_t)dataConfig->TxDmaPriority ) &&
            ( (uint32_t)DMA_PRIORITY_CNT  > (uint32_t)dataConfig->RxDmaPriority )    )
        {
            retState = I2c_Dma_Get_ChannelSel( periphId, I2C_DMA_DIR_TX, (i2c_DmaCode_t)dataConfig->TxDma, &txChannelSel );
        }
        else
        {
            /* Priority out of range */
            retState = I2C_REQUEST_ERROR;
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Dma_Get_ChannelSel( periphId, I2C_DMA_DIR_RX, (i2c_DmaCode_t)dataConfig->RxDma, &rxChannelSel );
        }
        else
        {
            /* Transmit stream is not connected to the I2C transmit request */
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA data handling - streams of both directions are configured and their
 *        interrupts enabled, I2C DMA requests stay disabled until the transfer start
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_XferInit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Dma_Set_StreamInit( periphId, I2C_DMA_DIR_TX );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Dma_Set_StreamInit( periphId, I2C_DMA_DIR_RX );
    }
    else
    {
        /* Transmit stream initialization failed */
    }

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Dma_XferStop( periphId );
    }
    else
    {
        /* Receive stream initialization failed */
    }

    return ( retState );
}


/**
 * \brief Deinitializes DMA data handling - transfer is stopped, streams are disabled and their
 *        interrupts disabled
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
    const i2c_RequestState_t txState   = I2c_Dma_Set_StreamOff( periphId, I2C_DMA_DIR_TX );
    const i2c_RequestState_t rxState   = I2c_Dma_Set_StreamOff( periphId, I2C_DMA_DIR_RX );

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
 * \brief Arms the streams of the transfer before the START condition - transmit stream for the
 *        write phase, receive stream and LAST for the read phase of 2 and more bytes, DMA
 *        requests (DMAEN) and I2C event / error interrupts are enabled
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_XferStart( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t  retState  = I2C_REQUEST_ERROR;
    i2c_XferContext_t * xferCtx   = I2C_NULL_PTR;
    i2c_FunctionState_t dmaRxUsed = I2C_FUNCTION_INACTIVE;
    i2c_DmaCr2Mask_t    cr2Mask   = 0u;

    retState = I2c_Get_XferContext( periphId, &xferCtx );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Get_DmaRxUsed( periphId, &dmaRxUsed );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( ( I2C_REQUEST_OK == retState               ) &&
        ( 0u              < xferCtx->Request.TxSize )    )
    {
        retState = I2c_Dma_Set_Transfer( periphId, I2C_DMA_DIR_TX );
        cr2Mask |= I2C_CR2_DMAEN;
    }
    else
    {
        /* Previous step failed or no write phase */
    }

    if( ( I2C_REQUEST_OK      == retState  ) &&
        ( I2C_FUNCTION_ACTIVE == dmaRxUsed )    )
    {
        retState = I2c_Dma_Set_Transfer( periphId, I2C_DMA_DIR_RX );
        cr2Mask |= ( I2C_CR2_DMAEN | I2C_CR2_LAST );
    }
    else
    {
        /* Previous step failed or read phase is not moved by DMA */
    }

    if( ( I2C_REQUEST_OK == retState ) &&
        ( 0u             != cr2Mask  )    )
    {
        retState = I2c_Dma_Set_Cr2( periphId, cr2Mask, I2C_FUNCTION_ACTIVE );
    }
    else
    {
        /* Previous step failed or no DMA transfer (address only, single byte read) */
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
 * \brief Stops DMA data handling - I2C interrupts, DMA requests and LAST are disabled and the
 *        streams are stopped
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_XferStop( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    const i2c_RequestState_t itState  = I2c_Isr_Set_ItInactive( periphId, I2C_ISR_IT_ALL );
    const i2c_RequestState_t cr2State = I2c_Dma_Set_Cr2( periphId, I2C_DMA_CR2_ALL, I2C_FUNCTION_INACTIVE );
    const i2c_RequestState_t txState  = I2c_Dma_Set_Stop( periphId, I2C_DMA_DIR_TX );
    const i2c_RequestState_t rxState  = I2c_Dma_Set_Stop( periphId, I2C_DMA_DIR_RX );

    if( ( I2C_REQUEST_OK == itState  ) &&
        ( I2C_REQUEST_OK == cr2State ) &&
        ( I2C_REQUEST_OK == txState  ) &&
        ( I2C_REQUEST_OK == rxState  )    )
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
 * \brief Checks that the transmit stream moved all bytes of the write phase
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Returns \ref I2C_REQUEST_OK if all bytes were written to DR (also if the transfer has
 *         no write phase). Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_Check_TxDone( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t  retState    = I2C_REQUEST_ERROR;
    i2c_XferContext_t * xferCtx     = I2C_NULL_PTR;
    dma_DataCount_t     txRemaining = 0u;

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

    if( ( I2C_REQUEST_OK == retState    ) &&
        ( 0u             == txRemaining )    )
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
 * \brief Checks that the streams moved all data of the transfer (end of transfer check)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Returns \ref I2C_REQUEST_OK if all bytes of the request were moved by DMA. Otherwise
 *         returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Dma_Check_Done( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t  retState    = I2C_REQUEST_ERROR;
    i2c_FunctionState_t dmaRxUsed   = I2C_FUNCTION_INACTIVE;
    dma_DataCount_t     rxRemaining = 0u;

    retState = I2c_Dma_Check_TxDone( periphId );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Get_DmaRxUsed( periphId, &dmaRxUsed );
    }
    else
    {
        /* Write phase is not finished */
    }

    if( ( I2C_REQUEST_OK      == retState  ) &&
        ( I2C_FUNCTION_ACTIVE == dmaRxUsed )    )
    {
        retState = I2c_Dma_Get_Remaining( periphId, I2C_DMA_DIR_RX, &rxRemaining );
    }
    else
    {
        /* Previous step failed or read phase is not moved by DMA */
    }

    if( ( I2C_REQUEST_OK == retState    ) &&
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
 * \brief Finds channel selection of the stream connected to the I2C request of the direction
 *
 * The DMA peripheral and the stream are decoded from the item of the DMA stream list, the item has to
 * belong to the I2C peripheral.
 *
 * \param periphId    [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir      [in]: Transfer direction
 * \param dmaCode     [in]: Item of \ref i2c_TxDma_t / \ref i2c_RxDma_t (encoded DMA stream)
 * \param channelSel [out]: Pointer to store the channel selection. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if the stream is connected to the request. Otherwise
 *         returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Get_ChannelSel( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, i2c_DmaCode_t dmaCode, dma_PeriphReqId_t * const channelSel )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_NULL_PTR != channelSel )
    {
        const uint32_t codePeriph = I2C_DMA_BIT_MASK_DECODE_PERIPH( dmaCode );
        const uint32_t codeDmaId  = I2C_DMA_BIT_MASK_DECODE_DMA( dmaCode );
        const uint32_t codeStream = I2C_DMA_BIT_MASK_DECODE_STREAM( dmaCode );

        for( uint32_t mapIdx = 0u; I2C_DMA_REQ_MAP_CNT > mapIdx; mapIdx ++ )
        {
            const i2c_DmaReqMap_t * const mapEntry = &i2c_DmaReqMap[ mapIdx ];

            if( ( codePeriph == (uint32_t)periphId           ) &&
                ( periphId   == mapEntry->PeriphId           ) &&
                ( dmaDir     == mapEntry->Dir                ) &&
                ( codeDmaId  == (uint32_t)mapEntry->DmaId    ) &&
                ( codeStream == (uint32_t)mapEntry->StreamId )    )
            {
                *channelSel = mapEntry->ChannelSel;
                retState    = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Entry does not match, keep searching */
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
 * \brief Configures the stream of one direction (request channel selection, I2C data register,
 *        8-bit normal transfer, priority, callbacks) and enables its interrupts in DMA and NVIC
 *        (transfer error for both directions, transfer complete for reception)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_StreamInit( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
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

    if( ( I2C_REQUEST_OK == retState ) && ( I2C_DMA_DIR_CNT > dmaDir ) )
    {
        i2c_DmaStreamState_t * const     strState   = &i2c_DmaStreamState[ periphId ][ dmaDir ];
        const i2c_DmaIsrConfig_t * const isrConfig  = &i2c_DmaIsrConfig[ periphId ];
        dma_ConfigStruct_t               dmaConfig;
        dma_PeriphReqId_t                channelSel = DMA_REQ_CHANNEL_0;
        dma_RequestState_t               dmaState   = DMA_REQUEST_ERROR;
        i2c_DmaCode_t                    dmaCode    = (i2c_DmaCode_t)xferCtx->Config.TxDma;
        i2c_DmaPriority_t                dmaPrio    = xferCtx->Config.TxDmaPriority;

        dmaState = Dma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.TransferMode        = DMA_TRANSFER_MODE_NORMAL;
        dmaConfig.PeriphAddress       = (dma_PeriphAddr_t)LL_I2C_DMA_GetRegAddr( periphReg );
        dmaConfig.MemoryAddress       = 0u;
        dmaConfig.PeriphAddrIncrement = DMA_PERIPH_ADDR_STATIC;
        dmaConfig.MemoryAddrIncrement = DMA_MEMORY_ADDR_INCREMENT;
        dmaConfig.PeriphTransferSize  = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.MemoryTransferSize  = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.DataCount           = 0u;
        dmaConfig.HalfTransferCallback = DMA_NULL_PTR;

        if( I2C_DMA_DIR_TX == dmaDir )
        {
            dmaConfig.Direction                = DMA_DIR_MEMORY_TO_PERIPH;
            dmaConfig.TransferCompleteCallback = DMA_NULL_PTR;
            dmaConfig.TransferErrorCallback    = isrConfig->TxErrorIsr;
        }
        else
        {
            dmaCode = (i2c_DmaCode_t)xferCtx->Config.RxDma;
            dmaPrio = xferCtx->Config.RxDmaPriority;

            dmaConfig.Direction                = DMA_DIR_PERIPH_TO_MEMORY;
            dmaConfig.TransferCompleteCallback = isrConfig->RxCompleteIsr;
            dmaConfig.TransferErrorCallback    = isrConfig->RxErrorIsr;
        }

        const i2c_RequestState_t mapState = I2c_Dma_Get_ChannelSel( periphId, dmaDir, dmaCode, &channelSel );

        dmaConfig.DmaPeriphId     = (dma_PeriphId_t)I2C_DMA_BIT_MASK_DECODE_DMA( dmaCode );
        dmaConfig.DmaChannel      = (dma_ChannelId_t)I2C_DMA_BIT_MASK_DECODE_STREAM( dmaCode );
        dmaConfig.PeripheralReqId = channelSel;
        dmaConfig.Priority        = (dma_Priority_t)dmaPrio;

        if( ( DMA_REQUEST_OK == dmaState ) &&
            ( I2C_REQUEST_OK == mapState )    )
        {
            dmaState = Dma_Init( &dmaConfig );
        }
        else
        {
            /* Default configuration is not available or stream is not connected to the request */
            dmaState = DMA_REQUEST_ERROR;
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            strState->Initialized = I2C_FUNCTION_ACTIVE;
            strState->DmaId       = dmaConfig.DmaPeriphId;
            strState->StreamId    = dmaConfig.DmaChannel;

            dmaState = Dma_Set_TransferErrorIrqActive( strState->DmaId, strState->StreamId );
        }
        else
        {
            /* Stream initialization failed */
        }

        if( ( DMA_REQUEST_OK == dmaState ) &&
            ( I2C_DMA_DIR_RX == dmaDir   )    )
        {
            dmaState = Dma_Set_TransferCompleteIrqActive( strState->DmaId, strState->StreamId );
        }
        else
        {
            /* Previous step failed or end of transmission is detected by I2C BTF */
        }

        /* DMA module does not enable the stream interrupt in NVIC by itself */
        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_InterruptActive( strState->DmaId, strState->StreamId );
        }
        else
        {
            /* Stream interrupt configuration failed */
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
 * \brief Disables the stream of one direction and its interrupts (DMA and NVIC)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems (also if no stream was initialized). Otherwise
 *         returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_StreamOff( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT > periphId ) && ( I2C_DMA_DIR_CNT > dmaDir ) )
    {
        i2c_DmaStreamState_t * const strState = &i2c_DmaStreamState[ periphId ][ dmaDir ];

        if( I2C_FUNCTION_ACTIVE == strState->Initialized )
        {
            const dma_RequestState_t xferState = Dma_Set_TransferInactive( strState->DmaId, strState->StreamId );
            const dma_RequestState_t teState   = Dma_Set_TransferErrorIrqInactive( strState->DmaId, strState->StreamId );
            const dma_RequestState_t tcState   = Dma_Set_TransferCompleteIrqInactive( strState->DmaId, strState->StreamId );
            const dma_RequestState_t nvicState = Dma_Set_InterruptInactive( strState->DmaId, strState->StreamId );

            strState->Initialized = I2C_FUNCTION_INACTIVE;

            if( ( DMA_REQUEST_OK == xferState ) &&
                ( DMA_REQUEST_OK == teState   ) &&
                ( DMA_REQUEST_OK == tcState   ) &&
                ( DMA_REQUEST_OK == nvicState )    )
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
            /* No stream was initialized for the direction */
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
 * \brief Arms the stream of one direction with the buffer of the running request and enables it
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Transfer( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t  retState = I2C_REQUEST_ERROR;
    i2c_XferContext_t * xferCtx  = I2C_NULL_PTR;

    retState = I2c_Get_XferContext( periphId, &xferCtx );

    if( ( I2C_REQUEST_OK      == retState                                            ) &&
        ( I2C_DMA_DIR_CNT      > dmaDir                                              ) &&
        ( I2C_FUNCTION_ACTIVE == i2c_DmaStreamState[ periphId ][ dmaDir ].Initialized )    )
    {
        const i2c_DmaStreamState_t * const strState = &i2c_DmaStreamState[ periphId ][ dmaDir ];
        dma_MemoryAddr_t                   memAddr  = 0u;
        dma_DataCount_t                    dataCnt  = 0u;
        dma_RequestState_t                 dmaState = DMA_REQUEST_ERROR;

        if( I2C_DMA_DIR_TX == dmaDir )
        {
            memAddr = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Request.TxData;
            dataCnt = (dma_DataCount_t)xferCtx->Request.TxSize;
        }
        else
        {
            memAddr = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Request.RxData;
            dataCnt = (dma_DataCount_t)xferCtx->Request.RxSize;
        }

        dmaState = Dma_Set_MemoryAddr( strState->DmaId, strState->StreamId, memAddr );

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_DataCount( strState->DmaId, strState->StreamId, dataCnt );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferActive( strState->DmaId, strState->StreamId );
        }
        else
        {
            /* Count of data configuration failed */
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
 * \brief Stops the stream of one direction (no callback of the aborted transfer)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir   [in]: Transfer direction
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems (also if no stream was initialized). Otherwise
 *         returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Stop( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT > periphId ) && ( I2C_DMA_DIR_CNT > dmaDir ) )
    {
        const i2c_DmaStreamState_t * const strState = &i2c_DmaStreamState[ periphId ][ dmaDir ];
        dma_RequestState_t                 dmaState = DMA_REQUEST_OK;

        if( I2C_FUNCTION_ACTIVE == strState->Initialized )
        {
            dmaState = Dma_Set_TransferInactive( strState->DmaId, strState->StreamId );
        }
        else
        {
            /* No stream was initialized for the direction */
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
 * \brief Reads count of data not yet moved by the stream of one direction
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dmaDir     [in]: Transfer direction
 * \param remaining [out]: Pointer to store the count. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Get_Remaining( i2c_PeriphId_t periphId, i2c_DmaDir_t dmaDir, dma_DataCount_t * const remaining )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT      > periphId                                            ) &&
        ( I2C_DMA_DIR_CNT     > dmaDir                                              ) &&
        ( I2C_NULL_PTR       != remaining                                           ) &&
        ( I2C_FUNCTION_ACTIVE == i2c_DmaStreamState[ periphId ][ dmaDir ].Initialized )    )
    {
        const i2c_DmaStreamState_t * const strState = &i2c_DmaStreamState[ periphId ][ dmaDir ];
        const dma_RequestState_t           dmaState = Dma_Get_DataCount( strState->DmaId, strState->StreamId, remaining );

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
 * \brief Sets / clears DMA request bits of CR2 (DMAEN, LAST) with read-back verification
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param cr2Mask  [in]: Bits to be changed (subset of I2C_DMA_CR2_ALL)
 * \param cr2State [in]: \ref I2C_FUNCTION_ACTIVE - bits are set, otherwise cleared
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Dma_Set_Cr2( i2c_PeriphId_t periphId, i2c_DmaCr2Mask_t cr2Mask, i2c_FunctionState_t cr2State )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    I2C_TypeDef *      periphReg = I2C_NULL_PTR;

    retState = I2c_Get_PeriphReg( periphId, &periphReg );

    if( ( I2C_REQUEST_OK == retState                              ) &&
        ( 0u             == ( cr2Mask & ~(i2c_DmaCr2Mask_t)I2C_DMA_CR2_ALL ) )    )
    {
        uint32_t expected = 0u;

        if( I2C_FUNCTION_ACTIVE == cr2State )
        {
            SET_BIT( periphReg->CR2, cr2Mask );
            expected = cr2Mask;
        }
        else
        {
            CLEAR_BIT( periphReg->CR2, cr2Mask );
            expected = 0u;
        }

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = READ_BIT( periphReg->CR2, cr2Mask );

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

/* ================================ TASKS =================================== */
