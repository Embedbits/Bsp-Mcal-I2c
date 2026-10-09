/**
 * \author Mr.Nobody
 * \file ItTest_I2c.c
 * \ingroup I2c
 * \brief Integration tests of I2C module on target.
 *
 * I2c module runs on the MCU together with real RCC, NVIC, GPIO and GPDMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): peripheral and pin initialization, timing
 * (SCL frequency) calculation from the kernel clock, filters, idle bus state,
 * master transfer to a not present slave (address NACK) in polling, interrupt
 * and DMA mode and deinitialization.
 *
 * No slave device and no external wiring is used - SCL / SDA lines are pulled
 * up by internal pull-up resistors (PinPull), every address is not acknowledged.
 *
 * Used resources: I2C1 on board specific pins (see board configuration below,
 * not connected on the board); GPDMA1 channels 5 / 6 (DMA mode).
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "I2c_Port.h"                       /* Module under test              */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void It_I2c_Init                 ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq );
static void It_I2c_Wait_XferEnd         ( void );
static void It_I2c_Check_NackTransfer   ( const i2c_XferRequest_t * const xferRequest );

static void It_I2c_XferCompleteCallback ( void );
static void It_I2c_ErrorCallback        ( i2c_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32U535xE) || \
    defined(IT_BOARD_STM32U545xE)

    /* NUCLEO-U545RE-Q */

    /** I2C1 on PB6 (SCL) / PB7 (SDA) - Arduino D15 / D14, not connected without shield */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB6 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB7 )

#elif defined(IT_BOARD_STM32U575xI) || \
      defined(IT_BOARD_STM32U585xI) || \
      defined(IT_BOARD_STM32U595xJ) || \
      defined(IT_BOARD_STM32U599xJ) || \
      defined(IT_BOARD_STM32U5A5xJ) || \
      defined(IT_BOARD_STM32U5A9xJ)

    /* NUCLEO-U575ZI-Q, NUCLEO-U5A5ZJ-Q (PB7 is LED LD2 blue) */

    /** I2C1 on PB8 (SCL) / PB9 (SDA) - Arduino D15 / D14, not connected without shield */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )

#else
    #error "Board of I2c integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** I2C peripheral under test */
#define IT_I2C_PERIPH                       ( I2C_PERIPH_1 )

/** Address of a not present slave (EEPROM address range) */
#define IT_I2C_SLAVE_ADDR                   ( 0x50u )

/** Standard-mode and Fast-mode SCL frequency [Hz] */
#define IT_I2C_FREQ_STANDARD_HZ             ( 100000u )
#define IT_I2C_FREQ_FAST_HZ                 ( 400000u )

/** Tolerance of the SCL frequency read back from timing register [%] */
#define IT_I2C_FREQ_TOL_PCT                 ( 10u )

/** Maximal count of wait loop iterations (I2c_Task called) */
#define IT_I2C_WAIT_LOOPS                   ( 500000u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Data handling configuration (copied by the module) */
static i2c_DataConfig_t             itI2c_DataConfig;

/** Transmit and receive data */
static const i2c_Data_t             itI2c_TxData[ 2u ] = { 0x00u, 0x10u };
static i2c_Data_t                   itI2c_RxData[ 4u ];

/** Counts of callback calls */
static volatile uint32_t            itI2c_CompleteCnt;
static volatile uint32_t            itI2c_ErrorCnt;

/** Error of the last error callback */
static volatile i2c_XferErrorId_t   itI2c_ErrorId;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itI2c_CompleteCnt = 0u;
    itI2c_ErrorCnt    = 0u;
    itI2c_ErrorId     = I2C_XFER_ERROR_CNT;
}


void tearDown( void )
{
    /* Every test case runs after system reset */
}

/* =============================== TESTS ==================================== */

/*----------------------------- Configuration --------------------------------*/

/**
 * \brief   I2c_Init() on target in standard mode - configuration read back.
 *
 * \details Initializes I2C1 (polling mode, 100 kHz, internal pull-ups on board
 *          specific pins) and reads the configuration back.
 *
 * \par Expected results
 * - Bus frequency 100 kHz +- 10 % (calculated from real kernel clock).
 * - Analog filter enabled, digital filter off, 7-bit addressing.
 * - Peripheral is enabled.
 */
void It_I2c_Init_StandardMode_ConfigurationReadBack( void )
{
    i2c_FreqHz_t        busFreq       = 0u;
    i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_CNT;
    i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_CNT;
    i2c_AddrMode_t      addrMode      = I2C_ADDR_MODE_CNT;
    i2c_FlagState_t     periphState   = I2C_FLAG_INACTIVE;

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_STANDARD_HZ * IT_I2C_FREQ_TOL_PCT ) / 100u, IT_I2C_FREQ_STANDARD_HZ, busFreq );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AnalogFilter( IT_I2C_PERIPH, &analogFilter ) );
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_ENABLED, analogFilter );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DigitalFilter( IT_I2C_PERIPH, &digitalFilter ) );
    TEST_ASSERT_EQUAL( I2C_DIGITAL_FILTER_OFF, digitalFilter );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AddrMode( IT_I2C_PERIPH, &addrMode ) );
    TEST_ASSERT_EQUAL( I2C_ADDR_MODE_7BIT, addrMode );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphState( IT_I2C_PERIPH, &periphState ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, periphState );
}


/**
 * \brief   I2c_Init() on target in fast mode - bus frequency read back.
 *
 * \details Initializes I2C1 with 400 kHz and reads the bus frequency.
 *
 * \par Expected results
 * - Bus frequency 400 kHz +- 10 %.
 */
void It_I2c_Init_FastMode_BusFreqReadBack( void )
{
    i2c_FreqHz_t busFreq = 0u;

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_HZ * IT_I2C_FREQ_TOL_PCT ) / 100u, IT_I2C_FREQ_FAST_HZ, busFreq );
}


/**
 * \brief   Idle bus with pull-ups is reported free.
 *
 * \details Initializes I2C1 (lines pulled up internally, no device) and reads bus state.
 *
 * \par Expected results
 * - Bus busy flag is INACTIVE.
 */
void It_I2c_Get_BusState_IdleBusWithPullUps_BusFree( void )
{
    i2c_FlagState_t busBusy = I2C_FLAG_ACTIVE;

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( IT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, busBusy );
}


/**
 * \brief   I2c_Init() on target rejects invalid configurations.
 *
 * \details Calls I2c_Init() with NULL, bus frequency 0 and invalid peripheral.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases.
 */
void It_I2c_Init_InvalidConfig_ReturnsError( void )
{
    i2c_Config_t config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( I2C_NULL_PTR ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = IT_I2C_PERIPH;
    config.BusFreq  = 0u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = I2C_PERIPH_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
}


/**
 * \brief   I2c_Deinit() on target disables the peripheral.
 *
 * \details Initializes I2C1 in ISR mode, deinitializes it and tries to start an
 *          address-only transfer.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, peripheral state INACTIVE.
 * - Transfer start is rejected (I2C_REQUEST_ERROR).
 */
void It_I2c_Deinit_InitializedPeripheral_PeripheralInactive( void )
{
    i2c_FlagState_t   periphState = I2C_FLAG_ACTIVE;
    i2c_XferRequest_t request     = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( IT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphState( IT_I2C_PERIPH, &periphState ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, periphState );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
}

/*--------------------------- Transfers (NACK) -------------------------------*/

/**
 * \brief   Polling address-only transfer to absent slave reports NACK.
 *
 * \details Initializes polling mode (100 kHz) and starts address-only transfer to
 *          slave 0x50 (no device on the bus), polls until the transfer ends.
 *
 * \par Expected results
 * - I2c_Set_XferStart() returns I2C_REQUEST_OK.
 * - Error callback called once with I2C_XFER_ERROR_NACK, no complete callback.
 * - I2c_Get_XferError() returns NACK, transfer state is INACTIVE.
 * - Bus is free (STOP generated after NACK).
 */
void It_I2c_Set_XferStart_PollAddressOnly_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );
}


/**
 * \brief   ISR mode write to absent slave reports NACK.
 *
 * \details Initializes ISR mode (400 kHz) and starts 2 byte write to slave 0x50.
 *
 * \par Expected results
 * - I2c_Set_XferStart() returns I2C_REQUEST_OK.
 * - Error callback called once with I2C_XFER_ERROR_NACK, no complete callback.
 * - I2c_Get_XferError() returns NACK, transfer state is INACTIVE.
 * - Bus is free (STOP generated after NACK).
 */
void It_I2c_Set_XferStart_IsrWrite_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_SLAVE_ADDR, itI2c_TxData, 2u, I2C_NULL_PTR, 0u };

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_FAST_HZ );

    It_I2c_Check_NackTransfer( &request );
}


/**
 * \brief   ISR mode write + read to absent slave reports NACK.
 *
 * \details Initializes ISR mode (100 kHz) and starts 1 byte write + 4 byte read
 *          from slave 0x50.
 *
 * \par Expected results
 * - I2c_Set_XferStart() returns I2C_REQUEST_OK.
 * - Error callback called once with I2C_XFER_ERROR_NACK, no complete callback.
 * - I2c_Get_XferError() returns NACK, transfer state is INACTIVE.
 * - Bus is free (STOP generated after NACK).
 */
void It_I2c_Set_XferStart_IsrWriteRead_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_SLAVE_ADDR, itI2c_TxData, 1u, itI2c_RxData, 4u };

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );
}


/**
 * \brief   DMA mode read from absent slave reports NACK.
 *
 * \details Initializes DMA mode (GPDMA1, 100 kHz) and starts 4 byte read from
 *          slave 0x50.
 *
 * \par Expected results
 * - I2c_Set_XferStart() returns I2C_REQUEST_OK.
 * - Error callback called once with I2C_XFER_ERROR_NACK, no complete callback.
 * - I2c_Get_XferError() returns NACK, transfer state is INACTIVE.
 * - Bus is free (STOP generated after NACK).
 */
void It_I2c_Set_XferStart_DmaRead_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, itI2c_RxData, 4u };

    It_I2c_Init( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );
}


/**
 * \brief   Transfer after NACK can be started and reports NACK again.
 *
 * \details Initializes ISR mode, starts 2 byte write to slave 0x50 twice (error
 *          counters cleared between the transfers).
 *
 * \par Expected results
 * - Both transfers: start OK, NACK error reported once, no complete callback,
 *   transfer INACTIVE and bus free.
 */
void It_I2c_Set_XferStart_RepeatedAfterNack_NackReportedAgain( void )
{
    const i2c_XferRequest_t request = { IT_I2C_SLAVE_ADDR, itI2c_TxData, 2u, I2C_NULL_PTR, 0u };

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );

    itI2c_ErrorCnt = 0u;
    itI2c_ErrorId  = I2C_XFER_ERROR_CNT;
    It_I2c_Check_NackTransfer( &request );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes I2C1 with internal pull-ups and data handling.
 *
 * \param xferMode [in]: Data transfer mode
 * \param busFreq  [in]: SCL frequency [Hz]
 */
static void It_I2c_Init( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq )
{
    i2c_Config_t config;

    itI2c_DataConfig.XferMode             = xferMode;
    itI2c_DataConfig.TxDmaPeriphId        = I2C_DMA_PERIPH_1;
    itI2c_DataConfig.TxDmaChannelId       = I2C_DMA_CHANNEL_5;
    itI2c_DataConfig.TxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.RxDmaPeriphId        = I2C_DMA_PERIPH_1;
    itI2c_DataConfig.RxDmaChannelId       = I2C_DMA_CHANNEL_6;
    itI2c_DataConfig.RxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.IrqPriority          = 5u;
    itI2c_DataConfig.XferCompleteCallback = It_I2c_XferCompleteCallback;
    itI2c_DataConfig.ErrorCallback        = It_I2c_ErrorCallback;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId      = IT_I2C_PERIPH;
    config.BusFreq       = busFreq;
    config.AnalogFilter  = I2C_ANALOG_FILTER_ENABLED;
    config.DigitalFilter = I2C_DIGITAL_FILTER_OFF;
    config.AddrMode      = I2C_ADDR_MODE_7BIT;
    config.DataConfig    = &itI2c_DataConfig;
    config.SclPin        = IT_I2C_SCL_PIN;
    config.SdaPin        = IT_I2C_SDA_PIN;
    config.PinPull       = I2C_PIN_PULL_UP;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
}


/**
 * \brief Waits until a callback is called or timeout (I2c_Task called - polling mode).
 */
static void It_I2c_Wait_XferEnd( void )
{
    for( uint32_t loopIdx = 0u; ( IT_I2C_WAIT_LOOPS > loopIdx ) && ( 0u == ( itI2c_CompleteCnt + itI2c_ErrorCnt ) ); loopIdx++ )
    {
        I2c_Task();
    }
}


/**
 * \brief Starts transfer to not present slave and checks NACK error and free bus.
 *
 * \param xferRequest [in]: Transfer request
 */
static void It_I2c_Check_NackTransfer( const i2c_XferRequest_t * const xferRequest )
{
    i2c_FunctionState_t xferState = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t   xferError = I2C_XFER_ERROR_NONE;
    i2c_FlagState_t     busBusy   = I2C_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, xferRequest ) );
    It_I2c_Wait_XferEnd();

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_ErrorCnt, "NACK error not reported" );
    TEST_ASSERT_EQUAL_UINT32( 0u, itI2c_CompleteCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, itI2c_ErrorId );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( IT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, xferError );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( IT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );

    /* STOP condition was generated after NACK - bus is released */
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( IT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, busBusy );
}


/** \brief Transfer complete callback */
static void It_I2c_XferCompleteCallback( void )
{
    itI2c_CompleteCnt++;
}


/**
 * \brief Transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void It_I2c_ErrorCallback( i2c_XferErrorId_t errorId )
{
    itI2c_ErrorId = errorId;
    itI2c_ErrorCnt++;
}
