/**
 * \author Mr.Nobody
 * \file Test_I2c.c
 * \ingroup I2c
 * \brief Unit tests of Inter-Integrated Circuit (I2C) module (STM32F4, I2C v1).
 *
 * I2c.c, I2c_Isr.c, I2c_Poll.c and I2c_Dma.c are compiled unchanged with real
 * LL drivers. I2C registers are emulated by RegMem, RCC, NVIC, GPIO and DMA
 * modules are mocked by CMock. I2C ISR registered in NVIC and DMA callbacks
 * passed to Dma_Init() are captured by stubs and called directly.
 *
 * \note Emulated registers are plain memory:
 *       - SR1 / SR2 flags are not changed by data register accesses, tests
 *         preset SR1 before each sequencing step,
 *       - DR returns the last written value - tests preset received bytes,
 *       - CR1.START / CR1.STOP are not cleared by hardware, tests clear them
 *         (\ref Ut_I2c_Set_StartSent, \ref Ut_I2c_Set_StopSent).
 *
 * \note Data handling context of the module is static - setUp() releases the
 *       data handling of the previous test (I2c_Deinit with ignored mocks)
 *       and re-initializes the mocks.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "CmsisHost.h"                      /* Core intrinsics emulation      */
#include "I2c_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_i2c.h"                      /* I2C registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_I2c_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static gpio_RequestState_t  Ut_I2c_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static dma_RequestState_t   Ut_I2c_DmaInitStub          ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_I2c_DmaGetCountStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt );
static void                 Ut_I2c_Ignore_PeriphMocks   ( void );
static void                 Ut_I2c_Release              ( void );
static void                 Ut_I2c_Reset_Mocks          ( void );
static i2c_Config_t         Ut_I2c_Get_Config           ( void );
static i2c_DataConfig_t     Ut_I2c_Get_DataConfig       ( i2c_XferMode_t xferMode );
static void                 Ut_I2c_Init                 ( const i2c_DataConfig_t * const dataConfig );
static void                 Ut_I2c_Task                 ( uint32_t sr1Flags );
static void                 Ut_I2c_Call_Isr             ( uint32_t sr1Flags );
static void                 Ut_I2c_Set_StartSent        ( void );
static void                 Ut_I2c_Set_StopSent         ( void );
static void                 Ut_I2c_Check_XferEnd        ( i2c_XferErrorId_t expError );

static void                 Ut_I2c_XferCompleteCallback ( void );
static void                 Ut_I2c_ErrorCallback        ( i2c_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** I2C peripheral used by tests (available on all supported MCUs) */
#define UT_I2C_PERIPH                       ( I2C_PERIPH_1 )
#define UT_I2C_REG                          ( I2C1 )
#define UT_I2C_RCC                          ( RCC_PERIPH_I2C1 )
#define UT_I2C_NVIC_EV                      ( NVIC_PERIPH_IRQ_I2C1_EV )
#define UT_I2C_NVIC_ER                      ( NVIC_PERIPH_IRQ_I2C1_ER )

/** PCLK1 frequency returned by RCC mock by default [Hz] (STM32F407 at 168 MHz) */
#define UT_I2C_CLK_HZ                       ( 42000000u )

/** Interrupt priority of test configurations */
#define UT_I2C_PRIO                         ( 5u )

/** Slave address of test transfers (7-bit) and its address bytes */
#define UT_I2C_SLAVE_ADDR                   ( 0x50u )
#define UT_I2C_ADDR_WRITE                   ( 0xA0u )
#define UT_I2C_ADDR_READ                    ( 0xA1u )

/** 10-bit slave address of test transfers, its headers and second address byte */
#define UT_I2C_SLAVE_ADDR10                 ( 0x2A5u )
#define UT_I2C_HEADER10_WRITE               ( 0xF4u )
#define UT_I2C_HEADER10_READ                ( 0xF5u )
#define UT_I2C_ADDR10_LOW                   ( 0xA5u )

/** DMA streams of I2C1 used by tests (TX stream 6, RX stream 0, channel selection 1) */
#define UT_I2C_DMA_TX                       ( I2C_TX_DMA_I2C1_DMA1_STREAM6 )
#define UT_I2C_DMA_RX                       ( I2C_RX_DMA_I2C1_DMA1_STREAM0 )

/** Encoded DMA stream from the peripheral, DMA peripheral index, stream number and channel selection number
 *  (bit-fields written independently of I2C_DMA_ENCODE) */
#define UT_I2C_DMA_CODE( PERIPH, DMA, STREAM, CHSEL )   ( ( (PERIPH) << 15u ) | ( (DMA) << 10u ) | ( (STREAM) << 5u ) | (CHSEL) )

/** Count of transmit / receive buffer bytes */
#define UT_I2C_BUF_SIZE                     ( 8u )

/** Value of OAR1 bit 14 required by RM */
#define UT_I2C_OAR1_BIT14                   ( 0x4000u )

/** CR2 interrupt enable bits */
#define UT_I2C_CR2_IT_ALL                   ( I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC for the I2C (event and error interrupt) */
static nvic_IsrCallback_t       utI2c_Isr;

/** GPIO configurations of Gpio_Init calls (SCL, SDA) */
static gpio_Config_t            utI2c_GpioConfig[ 2u ];
static uint32_t                 utI2c_GpioInitCnt;

/** DMA configurations of Dma_Init calls (TX, RX) */
static dma_ConfigStruct_t       utI2c_DmaConfig[ 2u ];
static uint32_t                 utI2c_DmaInitCnt;

/** Count of data not moved by DMA returned by Dma_Get_DataCount stub */
static dma_DataCount_t          utI2c_DmaRemaining;

/** PCLK1 frequency returned by RCC stub */
static rcc_FreqHz_t             utI2c_ClkHz;

/** Transmit data */
static i2c_Data_t               utI2c_TxBuf[ UT_I2C_BUF_SIZE ];

/** Receive buffer */
static i2c_Data_t               utI2c_RxBuf[ UT_I2C_BUF_SIZE ];

/** Counts of callback calls */
static uint32_t                 utI2c_CompleteCnt;
static uint32_t                 utI2c_ErrorCnt;

/** Parameter of the last error callback */
static i2c_XferErrorId_t        utI2c_LastError;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    utI2c_ClkHz = UT_I2C_CLK_HZ;

    Ut_I2c_Release();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utI2c_Isr          = NULL;
    utI2c_GpioInitCnt  = 0u;
    utI2c_DmaInitCnt   = 0u;
    utI2c_DmaRemaining = 0u;
    utI2c_CompleteCnt  = 0u;
    utI2c_ErrorCnt     = 0u;
    utI2c_LastError    = I2C_XFER_ERROR_CNT;

    for( uint32_t byteIdx = 0u; UT_I2C_BUF_SIZE > byteIdx; byteIdx++ )
    {
        utI2c_TxBuf[ byteIdx ] = (i2c_Data_t)( 0x10u + byteIdx );
        utI2c_RxBuf[ byteIdx ] = 0u;
    }
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   I2c_Get_ModuleVersion() returns version of the module.
 *
 * \details Reads the module version structure.
 *
 * \par Expected results
 * - Version is 1.0.0 (Major 1, Minor 0, Patch 0).
 */
void Ut_I2c_Get_ModuleVersion_ReturnsVersion( void )
{
    const i2c_ModuleVersion_t version = I2c_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   I2c_Get_DefaultConfig() fills default configuration.
 *
 * \details Reads default configuration, then calls the function with NULL.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, I2C1, PCLK clock source, 100 kHz, analog filter enabled
 *   (disabled on devices without FLTR register), digital filter off, 7-bit
 *   addressing, no data configuration, SCL / SDA pins unused, no pull.
 * - NULL pointer: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Get_DefaultConfig_ReturnsDefaults( void )
{
    i2c_Config_t config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( I2C_PERIPH_1,              config.PeriphId );
    TEST_ASSERT_EQUAL( I2C_CLK_SRC_PCLK,          config.ClkSrc );
    TEST_ASSERT_EQUAL_UINT32( 100000u,            config.BusFreq );
#if defined(I2C_FLTR_ANOFF)
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_ENABLED, config.AnalogFilter );
#else
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_DISABLED, config.AnalogFilter );
#endif
    TEST_ASSERT_EQUAL( I2C_DIGITAL_FILTER_OFF,    config.DigitalFilter );
    TEST_ASSERT_EQUAL( I2C_ADDR_MODE_7BIT,        config.AddrMode );
    TEST_ASSERT_NULL( config.DataConfig );
    TEST_ASSERT_EQUAL( I2C_SCL_PIN_UNUSED,        config.SclPin );
    TEST_ASSERT_EQUAL( I2C_SDA_PIN_UNUSED,        config.SdaPin );
    TEST_ASSERT_EQUAL( I2C_PIN_PULL_NONE,         config.PinPull );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DefaultConfig( NULL ) );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   I2c_Init() rejects invalid configuration.
 *
 * \details Calls I2c_Init() with NULL and with valid configuration where one item
 *          is invalid: peripheral, clock source, bus frequency 0 and above 400 kHz,
 *          analog / digital filter, address mode, pin pull. On devices without
 *          FLTR register also enabled analog filter and digital filter length 1.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR is returned in all cases.
 * - RCC, GPIO and NVIC are not called (strict mocks without expectations).
 */
void Ut_I2c_Init_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( NULL ) );

    config.PeriphId = I2C_PERIPH_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.ClkSrc = I2C_CLK_SRC_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.BusFreq = 0u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.BusFreq = I2C_BUS_FREQ_MAX_HZ + 1u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.AnalogFilter = I2C_ANALOG_FILTER_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.DigitalFilter = I2C_DIGITAL_FILTER_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.AddrMode = I2C_ADDR_MODE_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.PinPull = I2C_PIN_PULL_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

#if !defined(I2C_FLTR_ANOFF)
    config = Ut_I2c_Get_Config();
    config.AnalogFilter = I2C_ANALOG_FILTER_ENABLED;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.DigitalFilter = I2C_DIGITAL_FILTER_1;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
#endif
}


/**
 * \brief   I2c_Init() rejects pin of other I2C peripheral.
 *
 * \details Initializes I2C1 with SCL pin of I2C2 (PB10).
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, no other module is called.
 */
void Ut_I2c_Init_PinOfOtherPeripheral_ReturnsErrorWithoutAccess( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.SclPin = I2C_SCL_PIN_I2C2_PB10;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
}


/**
 * \brief   I2c_Init() activates clock, resets and configures the peripheral.
 *
 * \details Initializes I2C1 with default configuration (100 kHz, no pins, no data
 *          handling), PCLK1 42 MHz.
 *
 * \par Expected results
 * - RCC: clock activated, reset pulse (active, inactive) for RCC_PERIPH_I2C1.
 * - I2C_REQUEST_OK, CR1.PE = 1, CR2.FREQ = 42, CCR = 210 (Standard-mode),
 *   TRISE = 43, OAR1 bit 14 = 1, 7-bit addressing.
 */
void Ut_I2c_Init_DefaultConfig_ClockTimingAndEnable( void )
{
    i2c_Config_t   config   = Ut_I2c_Get_Config();
    i2c_AddrMode_t addrMode = I2C_ADDR_MODE_CNT;

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 );
    TEST_ASSERT_EQUAL_UINT32( 42u, UT_I2C_REG->CR2 & I2C_CR2_FREQ );
    TEST_ASSERT_EQUAL_HEX32( 210u, UT_I2C_REG->CCR );
    TEST_ASSERT_EQUAL_UINT32( 43u, UT_I2C_REG->TRISE );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_OAR1_BIT14, UT_I2C_REG->OAR1 );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AddrMode( UT_I2C_PERIPH, &addrMode ) );
    TEST_ASSERT_EQUAL( I2C_ADDR_MODE_7BIT, addrMode );
}


/**
 * \brief   I2c_Init() stops on clock activation error.
 *
 * \details Rcc_Set_PeriphActive() returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, peripheral is not enabled (PE = 0).
 */
void Ut_I2c_Init_ClockError_ReturnsErrorPeripheralNotEnabled( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   I2c_Init() rejects PCLK1 out of the I2C range.
 *
 * \details RCC returns PCLK1 0 Hz, 1 MHz (below 2 MHz) and 60 MHz (above 50 MHz).
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, peripheral is not enabled (PE = 0), CCR not written.
 */
void Ut_I2c_Init_ClockOutOfRange_ReturnsErrorPeripheralNotEnabled( void )
{
    const rcc_FreqHz_t clkLut[ ] = { 0u, 1000000u, 60000000u };
    i2c_Config_t       config    = Ut_I2c_Get_Config();

    for( uint32_t clkIdx = 0u; ( sizeof( clkLut ) / sizeof( clkLut[ 0u ] ) ) > clkIdx; clkIdx++ )
    {
        Ut_I2c_Ignore_PeriphMocks();
        utI2c_ClkHz = clkLut[ clkIdx ];

        TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

        TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
        TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CCR );
    }
}


/**
 * \brief   I2c_Init() configures SCL / SDA pins as open-drain alternate function.
 *
 * \details Initializes I2C1 on PB6 (SCL) / PB7 (SDA) with internal pull-up (both pins exist
 *          on every device line).
 *
 * \par Expected results
 * - Gpio_Init() called twice: PB6 and PB7, alternate function 4, open-drain,
 *   pull-up, active level high.
 */
void Ut_I2c_Init_Pins_GpioOpenDrainAlternateWithPull( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.SclPin  = I2C_SCL_PIN_I2C1_PB6;
    config.SdaPin  = I2C_SDA_PIN_I2C1_PB7;
    config.PinPull = I2C_PIN_PULL_UP;

    Ut_I2c_Ignore_PeriphMocks();
    Gpio_Init_StubWithCallback( Ut_I2c_GpioInitStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_GpioInitCnt );

    TEST_ASSERT_EQUAL( GPIO_PORT_B,               utI2c_GpioConfig[ 0u ].PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_6,             utI2c_GpioConfig[ 0u ].PinId );
    TEST_ASSERT_EQUAL( GPIO_PORT_B,               utI2c_GpioConfig[ 1u ].PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_7,             utI2c_GpioConfig[ 1u ].PinId );

    for( uint32_t pinIdx = 0u; 2u > pinIdx; pinIdx++ )
    {
        TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE,   utI2c_GpioConfig[ pinIdx ].PinMode );
        TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_4,           utI2c_GpioConfig[ pinIdx ].PinAltFunction );
        TEST_ASSERT_EQUAL( GPIO_PIN_OUTPUT_OPENDRAIN, utI2c_GpioConfig[ pinIdx ].PinOutType );
        TEST_ASSERT_EQUAL( GPIO_PIN_PULL_UP,          utI2c_GpioConfig[ pinIdx ].PinPull );
        TEST_ASSERT_EQUAL( GPIO_PIN_LEVEL_HIGH,       utI2c_GpioConfig[ pinIdx ].PinActiveLevel );
    }
}


/**
 * \brief   I2c_Init() with ISR data handling configures I2C interrupts in NVIC.
 *
 * \details Initializes I2C1 with ISR transfer mode.
 *
 * \par Expected results
 * - ISR registered for event and error IRQ, priority 5 for both, both IRQs enabled.
 * - I2C interrupt sources (CR2 ITEVTEN / ITBUFEN / ITERREN) stay disabled.
 * - I2c_Get_DataConfig() returns the configuration.
 */
void Ut_I2c_Init_IsrDataConfig_InterruptsConfigured( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    i2c_Config_t           config     = Ut_I2c_Get_Config();
    i2c_DataConfig_t       readConfig;

    config.DataConfig = &dataConfig;

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_EV, UT_I2C_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_ER, UT_I2C_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_I2C_NVIC_EV, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_I2C_NVIC_ER, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_NOT_NULL( utI2c_Isr );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( I2C_XFER_MODE_ISR, readConfig.XferMode );
    TEST_ASSERT_EQUAL_PTR( Ut_I2c_XferCompleteCallback, readConfig.XferCompleteCallback );
}


/**
 * \brief   I2c_Deinit() disables the peripheral, resets it and disables its clock.
 *
 * \details Initializes I2C1 with ISR data handling, then deinitializes it.
 *
 * \par Expected results
 * - NVIC event and error IRQ disabled, RCC reset pulse and clock disabled.
 * - I2C_REQUEST_OK, PE = 0, data handling is not initialized any more.
 * - Invalid peripheral: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Deinit_InitializedPeripheral_DisabledResetAndClockOff( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    i2c_DataConfig_t       readConfig;

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_I2C_NVIC_EV, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_I2C_NVIC_ER, NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Deinit( I2C_PERIPH_CNT ) );
}

/* ======================= PERIPHERAL CONFIGURATION ========================= */

/**
 * \brief   I2c_Set_PeriphActive() / I2c_Set_PeriphInactive() control PE.
 *
 * \details Initializes I2C1, disables and enables it, reads the state.
 *
 * \par Expected results
 * - PE follows the requests, I2c_Get_PeriphState() reports it.
 */
void Ut_I2c_Set_PeriphActive_EnabledAndReported( void )
{
    i2c_FlagState_t periphState = I2C_FLAG_INACTIVE;

    Ut_I2c_Init( NULL );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphState( UT_I2C_PERIPH, &periphState ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, periphState );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphActive( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphState( UT_I2C_PERIPH, &periphState ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, periphState );
}


/**
 * \brief   Peripheral state functions reject invalid arguments.
 *
 * \details Calls the functions with invalid peripheral and NULL pointer.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases.
 */
void Ut_I2c_Set_PeriphState_InvalidArgs_ReturnsError( void )
{
    i2c_FlagState_t periphState = I2C_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_PeriphActive( I2C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_PeriphInactive( I2C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_PeriphState( I2C_PERIPH_CNT, &periphState ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_PeriphState( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusState( I2C_PERIPH_CNT, &periphState ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusState( UT_I2C_PERIPH, NULL ) );
}


/**
 * \brief   I2c_Set_PeriphInactive() is rejected while START is pending.
 *
 * \details Sets CR1.START of the enabled peripheral and disables it.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, PE stays 1.
 */
void Ut_I2c_Set_PeriphInactive_StartPending_ReturnsErrorStaysEnabled( void )
{
    Ut_I2c_Init( NULL );

    UT_I2C_REG->CR1 |= I2C_CR1_START;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   I2c_Get_BusState() reports SR2.BUSY.
 *
 * \details Reads the bus state with BUSY = 0 and BUSY = 1.
 *
 * \par Expected results
 * - INACTIVE, then ACTIVE.
 */
void Ut_I2c_Get_BusState_BusyFlag_Reported( void )
{
    i2c_FlagState_t busBusy = I2C_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( UT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, busBusy );

    UT_I2C_REG->SR2 = I2C_SR2_BUSY;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( UT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, busBusy );
}


/**
 * \brief   I2c_Set_BusFreq() calculates FREQ, CCR and TRISE (PCLK1 42 MHz).
 *
 * \details Disables the peripheral and configures 100, 50, 250 and 400 kHz.
 *
 * \par Expected results
 * - 100 kHz: CCR = 210 (Standard-mode), TRISE = 43, read back 100 kHz.
 * - 50 kHz:  CCR = 420, read back 50 kHz.
 * - 250 kHz: CCR = FS | 56 (duty 2), TRISE = 13, read back 250 kHz.
 * - 400 kHz: CCR = FS | 35 (duty 2), TRISE = 13, read back 400 kHz.
 * - FREQ = 42 in all cases.
 */
void Ut_I2c_Set_BusFreq_StandardAndFast_CcrTriseAndReadBack( void )
{
    const i2c_FreqHz_t freqLut[ ]  = { 100000u, 50000u,  250000u,            400000u            };
    const uint32_t     ccrLut[ ]   = { 210u,    420u,    I2C_CCR_FS | 56u,   I2C_CCR_FS | 35u   };
    const uint32_t     triseLut[ ] = { 43u,     43u,     13u,                13u                };
    i2c_FreqHz_t       busFreq     = 0u;

    Ut_I2c_Init( NULL );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );

    for( uint32_t freqIdx = 0u; ( sizeof( freqLut ) / sizeof( freqLut[ 0u ] ) ) > freqIdx; freqIdx++ )
    {
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, freqLut[ freqIdx ] ) );

        TEST_ASSERT_EQUAL_UINT32( 42u, UT_I2C_REG->CR2 & I2C_CR2_FREQ );
        TEST_ASSERT_EQUAL_HEX32( ccrLut[ freqIdx ], UT_I2C_REG->CCR );
        TEST_ASSERT_EQUAL_UINT32( triseLut[ freqIdx ], UT_I2C_REG->TRISE );

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( UT_I2C_PERIPH, &busFreq ) );
        TEST_ASSERT_EQUAL_UINT32( freqLut[ freqIdx ], busFreq );
    }
}


/**
 * \brief   I2c_Set_BusFreq() selects the Fast-mode duty cycle with the smaller error.
 *
 * \details 400 kHz with PCLK1 10 MHz (duty 16/9 exact) and 16 MHz (duty 2 closer).
 *
 * \par Expected results
 * - 10 MHz: CCR = FS | DUTY | 1, TRISE = 4, read back 400 kHz.
 * - 16 MHz: CCR = FS | 14, TRISE = 5, read back 380952 Hz (within 10 %).
 */
void Ut_I2c_Set_BusFreq_FastModeDutyCycle_SmallerErrorSelected( void )
{
    i2c_FreqHz_t busFreq = 0u;

    Ut_I2c_Init( NULL );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );

    utI2c_ClkHz = 10000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CCR_FS | I2C_CCR_DUTY | 1u, UT_I2C_REG->CCR );
    TEST_ASSERT_EQUAL_UINT32( 4u, UT_I2C_REG->TRISE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( UT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_EQUAL_UINT32( 400000u, busFreq );

    utI2c_ClkHz = 16000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CCR_FS | 14u, UT_I2C_REG->CCR );
    TEST_ASSERT_EQUAL_UINT32( 5u, UT_I2C_REG->TRISE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( UT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_EQUAL_UINT32( 380952u, busFreq );
}


/**
 * \brief   I2c_Set_BusFreq() respects the minimum SCL low / high period.
 *
 * \details Standard-mode 100 kHz with PCLK1 2 MHz and 3 MHz.
 *
 * \par Expected results
 * - SCL low / high period (CCR x tPCLK1) is at least 4.7 us in both cases.
 */
void Ut_I2c_Set_BusFreq_StandardModeLowPeriodRespected( void )
{
    const rcc_FreqHz_t clkLut[ ] = { 2000000u, 3000000u };

    Ut_I2c_Init( NULL );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );

    for( uint32_t clkIdx = 0u; ( sizeof( clkLut ) / sizeof( clkLut[ 0u ] ) ) > clkIdx; clkIdx++ )
    {
        utI2c_ClkHz = clkLut[ clkIdx ];

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

        const uint64_t lowNs = ( (uint64_t)( UT_I2C_REG->CCR & I2C_CCR_CCR ) * 1000000000u ) / utI2c_ClkHz;

        TEST_ASSERT_GREATER_OR_EQUAL_UINT64( 4700u, lowNs );
    }
}


/**
 * \brief   I2c_Set_BusFreq() rejects invalid requests without register write.
 *
 * \details 1. Enabled peripheral, 2. frequency 0 and above 400 kHz, 3. Fast-mode
 *          with PCLK1 3 MHz, 4. frequency below the CCR range (5 kHz at 42 MHz),
 *          5. invalid peripheral.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, CCR keeps the initial value (210).
 */
void Ut_I2c_Set_BusFreq_InvalidOrEnabled_ReturnsErrorWithoutWrite( void )
{
    Ut_I2c_Init( NULL );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 0u ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, I2C_BUS_FREQ_MAX_HZ + 1u ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 5000u ) );

    utI2c_ClkHz = 3000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( I2C_PERIPH_CNT, 100000u ) );

    TEST_ASSERT_EQUAL_HEX32( 210u, UT_I2C_REG->CCR );
}


/**
 * \brief   I2c_Get_BusFreq() rejects invalid arguments and not configured timing.
 *
 * \details Invalid peripheral, NULL pointer, CCR = 0 (peripheral not initialized).
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases.
 */
void Ut_I2c_Get_BusFreq_InvalidArgs_ReturnsError( void )
{
    i2c_FreqHz_t busFreq = 0u;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusFreq( I2C_PERIPH_CNT, &busFreq ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusFreq( UT_I2C_PERIPH, NULL ) );

    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusFreq( UT_I2C_PERIPH, &busFreq ) );
}


/**
 * \brief   Noise filters are configured according to the device capability.
 *
 * \details Devices with FLTR register: disables / enables the analog filter and
 *          configures digital filter OFF ... 15. Devices without FLTR register:
 *          configures filters off and on.
 *
 * \par Expected results
 * - FLTR: ANOFF = 1 / DISABLED and ANOFF = 0 / ENABLED read back, DNF equals the
 *   enumeration value and is read back.
 * - No FLTR: analog DISABLED and digital OFF accepted and read back, analog ENABLED
 *   and digital filter 1 rejected.
 */
void Ut_I2c_Set_Filters_DeviceCapability_RegisterAndReadBack( void )
{
    i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_CNT;
    i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_CNT;

    Ut_I2c_Init( NULL );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );

#if defined(I2C_FLTR_ANOFF)
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_DISABLED ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_FLTR_ANOFF, UT_I2C_REG->FLTR & I2C_FLTR_ANOFF );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AnalogFilter( UT_I2C_PERIPH, &analogFilter ) );
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_DISABLED, analogFilter );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_ENABLED ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->FLTR & I2C_FLTR_ANOFF );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AnalogFilter( UT_I2C_PERIPH, &analogFilter ) );
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_ENABLED, analogFilter );

    for( i2c_DigitalFilter_t filterIdx = I2C_DIGITAL_FILTER_OFF; I2C_DIGITAL_FILTER_CNT > filterIdx; filterIdx++ )
    {
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DigitalFilter( UT_I2C_PERIPH, filterIdx ) );
        TEST_ASSERT_EQUAL_UINT32( (uint32_t)filterIdx, UT_I2C_REG->FLTR & I2C_FLTR_DNF );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DigitalFilter( UT_I2C_PERIPH, &digitalFilter ) );
        TEST_ASSERT_EQUAL( filterIdx, digitalFilter );
    }
#else
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK,    I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_DISABLED ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_ENABLED ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK,    I2c_Set_DigitalFilter( UT_I2C_PERIPH, I2C_DIGITAL_FILTER_OFF ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DigitalFilter( UT_I2C_PERIPH, I2C_DIGITAL_FILTER_1 ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AnalogFilter( UT_I2C_PERIPH, &analogFilter ) );
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_DISABLED, analogFilter );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DigitalFilter( UT_I2C_PERIPH, &digitalFilter ) );
    TEST_ASSERT_EQUAL( I2C_DIGITAL_FILTER_OFF, digitalFilter );
#endif
}


/**
 * \brief   Filter functions reject invalid requests.
 *
 * \details 1. Enabled peripheral, 2. invalid values, 3. invalid peripheral and NULL.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases.
 */
void Ut_I2c_Set_Filters_InvalidOrEnabled_ReturnsError( void )
{
    i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_CNT;
    i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_CNT;

    Ut_I2c_Init( NULL );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_DISABLED ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DigitalFilter( UT_I2C_PERIPH, I2C_DIGITAL_FILTER_OFF ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DigitalFilter( UT_I2C_PERIPH, I2C_DIGITAL_FILTER_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AnalogFilter( I2C_PERIPH_CNT, I2C_ANALOG_FILTER_DISABLED ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DigitalFilter( I2C_PERIPH_CNT, I2C_DIGITAL_FILTER_OFF ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_AnalogFilter( I2C_PERIPH_CNT, &analogFilter ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_AnalogFilter( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DigitalFilter( I2C_PERIPH_CNT, &digitalFilter ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DigitalFilter( UT_I2C_PERIPH, NULL ) );
}


/**
 * \brief   I2c_Set_AddrMode() stores 7-bit / 10-bit addressing.
 *
 * \details Configures 10-bit and 7-bit mode, invalid values.
 *
 * \par Expected results
 * - Read back equals the request; invalid mode, peripheral or NULL: error.
 */
void Ut_I2c_Set_AddrMode_7And10Bit_ReadBack( void )
{
    i2c_AddrMode_t addrMode = I2C_ADDR_MODE_CNT;

    Ut_I2c_Init( NULL );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AddrMode( UT_I2C_PERIPH, &addrMode ) );
    TEST_ASSERT_EQUAL( I2C_ADDR_MODE_10BIT, addrMode );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_7BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AddrMode( UT_I2C_PERIPH, &addrMode ) );
    TEST_ASSERT_EQUAL( I2C_ADDR_MODE_7BIT, addrMode );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AddrMode( I2C_PERIPH_CNT, I2C_ADDR_MODE_7BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_AddrMode( UT_I2C_PERIPH, NULL ) );
}

/* ============================== INTERRUPTS ================================ */

/**
 * \brief   I2c_Set_IrqPriority() configures event and error IRQ.
 *
 * \details 1. Both NVIC calls succeed. 2. Error IRQ priority fails. 3. Invalid peripheral.
 *
 * \par Expected results
 * 1. I2C_REQUEST_OK, priority 5 set for event and error IRQ.
 * 2. I2C_REQUEST_ERROR.
 * 3. I2C_REQUEST_ERROR, NVIC not called.
 */
void Ut_I2c_Set_IrqPriority_EventAndErrorLines( void )
{
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_EV, UT_I2C_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_ER, UT_I2C_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_IrqPriority( UT_I2C_PERIPH, UT_I2C_PRIO ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_EV, UT_I2C_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_ER, UT_I2C_PRIO, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_IrqPriority( UT_I2C_PERIPH, UT_I2C_PRIO ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_IrqPriority( I2C_PERIPH_CNT, UT_I2C_PRIO ) );
}


/**
 * \brief   I2c_Get_IrqPriority() returns priority of event IRQ.
 *
 * \details NVIC mock returns priority 5 of event IRQ, then NULL pointer is used.
 *
 * \par Expected results
 * - I2C_REQUEST_OK and priority 5; NULL pointer: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Get_IrqPriority_ReadFromEventLine( void )
{
    static nvic_IrqPrio_t prio = UT_I2C_PRIO;
    i2c_IrqPrio_t         read = 0u;

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_EV, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &prio );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_IrqPriority( UT_I2C_PERIPH, &read ) );
    TEST_ASSERT_EQUAL_UINT32( UT_I2C_PRIO, read );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_IrqPriority( UT_I2C_PERIPH, NULL ) );
}

/* ======================= DATA HANDLING CONFIGURATION ====================== */

/**
 * \brief   I2c_Set_DataConfig() rejects invalid configuration.
 *
 * \details Calls I2c_Set_DataConfig() with NULL, invalid peripheral and invalid
 *          transfer mode; I2c_Get_DataConfig() before initialization.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, NVIC / DMA not called.
 */
void Ut_I2c_Set_DataConfig_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( I2C_PERIPH_CNT, &dataConfig ) );

    dataConfig.XferMode = I2C_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, NULL ) );
}


/**
 * \brief   DMA data configuration rejects streams out of the DMA stream lists.
 *
 * \details DMA mode with: unused streams, the RX stream item (I2C1 RX request) as TX stream, the
 *          TX stream item (I2C1 TX request) as RX stream, stream 3 (no I2C1 request), the I2C2
 *          stream items, DMA peripheral and stream out of range, priority out of range.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, DMA / NVIC not called.
 */
void Ut_I2c_Set_DataConfig_DmaInvalidStreams_ReturnsErrorWithoutAccess( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    dataConfig.TxDma = I2C_TX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = I2C_RX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDma = (i2c_TxDma_t)I2C_RX_DMA_I2C1_DMA1_STREAM0;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = (i2c_RxDma_t)I2C_TX_DMA_I2C1_DMA1_STREAM6;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = (i2c_RxDma_t)I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_3, 1u );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

#if defined(I2C2)
    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDma = I2C_TX_DMA_I2C2_DMA1_STREAM7;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = I2C_RX_DMA_I2C2_DMA1_STREAM2;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
#endif /* I2C2 */

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDma = (i2c_TxDma_t)I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_CNT, 1u );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = (i2c_RxDma_t)I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_CNT, I2C_DMA_CHANNEL_0, 1u );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = (i2c_RxDma_t)I2C_DMA_ENCODE( I2C_PERIPH_CNT, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_0, 1u );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDmaPriority = (i2c_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDmaPriority = (i2c_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
}


/**
 * \brief   DMA data configuration initializes the streams of both directions.
 *
 * \details Initializes I2C1 in DMA mode (TX stream 6, RX stream 0).
 *
 * \par Expected results
 * - Dma_Init() twice: TX stream 6 memory to peripheral, RX stream 0 peripheral to
 *   memory, channel 1, 8-bit, normal mode, peripheral address = DR, configured
 *   priorities; RX has transfer complete callback, both have error callback.
 * - CR2: DMAEN, LAST and I2C interrupt sources disabled.
 */
void Ut_I2c_Set_DataConfig_Dma_StreamsInitialized( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );

    TEST_ASSERT_EQUAL( DMA_PERIPH_1,               utI2c_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_STREAM_6,               utI2c_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH,   utI2c_DmaConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW,           utI2c_DmaConfig[ 0u ].Priority );
    TEST_ASSERT_NULL( utI2c_DmaConfig[ 0u ].TransferCompleteCallback );

    TEST_ASSERT_EQUAL( DMA_PERIPH_1,               utI2c_DmaConfig[ 1u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_STREAM_0,               utI2c_DmaConfig[ 1u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY,   utI2c_DmaConfig[ 1u ].Direction );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH,          utI2c_DmaConfig[ 1u ].Priority );
    TEST_ASSERT_NOT_NULL( utI2c_DmaConfig[ 1u ].TransferCompleteCallback );

    for( uint32_t dirIdx = 0u; 2u > dirIdx; dirIdx++ )
    {
        TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_1,          utI2c_DmaConfig[ dirIdx ].PeripheralReqId );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,   utI2c_DmaConfig[ dirIdx ].TransferMode );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,     utI2c_DmaConfig[ dirIdx ].PeriphTransferSize );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,     utI2c_DmaConfig[ dirIdx ].MemoryTransferSize );
        TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC,     utI2c_DmaConfig[ dirIdx ].PeriphAddrIncrement );
        TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT,  utI2c_DmaConfig[ dirIdx ].MemoryAddrIncrement );
        TEST_ASSERT_EQUAL_HEX32( (dma_PeriphAddr_t)LL_I2C_DMA_GetRegAddr( UT_I2C_REG ), utI2c_DmaConfig[ dirIdx ].PeriphAddress );
        TEST_ASSERT_NOT_NULL( utI2c_DmaConfig[ dirIdx ].TransferErrorCallback );
    }

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & ( I2C_CR2_DMAEN | I2C_CR2_LAST | UT_I2C_CR2_IT_ALL ) );
}

/* ============================ TRANSFER START ============================== */

/**
 * \brief   I2c_Set_XferStart() rejects invalid requests.
 *
 * \details NULL request, 7-bit address 0x80, TxSize without buffer, RxSize without
 *          buffer, invalid peripheral.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, START is not generated, transfer state INACTIVE.
 */
void Ut_I2c_Set_XferStart_InvalidRequest_ReturnsErrorWithoutStart( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    i2c_XferRequest_t      request    = { .SlaveAddr = 0x80u, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t    xferState  = I2C_FUNCTION_ACTIVE;

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    request.SlaveAddr = UT_I2C_SLAVE_ADDR;
    request.TxData    = NULL;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    request.TxSize = 0u;
    request.RxSize = 1u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    request.RxData = utI2c_RxBuf;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( I2C_PERIPH_CNT, &request ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_START );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
}


/**
 * \brief   I2c_Set_XferStart() is rejected when the peripheral is not ready.
 *
 * \details 1. Data handling not initialized, 2. transfer mode NONE, 3. peripheral
 *          disabled, 4. bus busy, 5. STOP of a previous transfer still pending.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, START is not generated.
 */
void Ut_I2c_Set_XferStart_NotReady_ReturnsErrorWithoutStart( void )
{
    const i2c_DataConfig_t  noneConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_NONE );
    const i2c_DataConfig_t  pollConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( NULL );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DataConfig( UT_I2C_PERIPH, &noneConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DataConfig( UT_I2C_PERIPH, &pollConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphActive( UT_I2C_PERIPH ) );
    UT_I2C_REG->SR2 = I2C_SR2_BUSY;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    UT_I2C_REG->SR2 = 0u;
    UT_I2C_REG->CR1 |= I2C_CR1_STOP;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_START );
}


/**
 * \brief   Second transfer start is rejected while a transfer is running.
 *
 * \details Starts a write, then starts it again; data configuration change and
 *          address mode change are requested during the transfer.
 *
 * \par Expected results
 * - First start OK (CR1.START, ACK / POS cleared, error flags of SR1 cleared),
 *   second start / configuration changes I2C_REQUEST_ERROR.
 */
void Ut_I2c_Set_XferStart_RunningTransfer_SecondRequestRejected( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    UT_I2C_REG->SR1  = I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR | I2C_SR1_OVR;
    UT_I2C_REG->CR1 |= I2C_CR1_ACK | I2C_CR1_POS;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_START, UT_I2C_REG->CR1 & ( I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_POS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->SR1 );
    Ut_I2c_Set_StartSent();

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
}

/* ========================== POLLING TRANSFERS ============================= */

/**
 * \brief   Polling write: address, data bytes, STOP after BTF.
 *
 * \details Starts 2 byte write to 0x50 and calls I2c_Task() with SB, ADDR, TXE,
 *          TXE, TXE (last byte in shift register), TXE + BTF.
 *
 * \par Expected results
 * - SB: DR = 0xA0 (address, write).
 * - TXE: DR = 0x10, then 0x11; TXE without BTF after the last byte: no action.
 * - BTF: STOP generated, complete callback once, transfer INACTIVE, error NONE.
 * - Task without running transfer does not call the callback again.
 */
void Ut_I2c_Task_PollWrite_BytesWrittenAndStopAfterBtf( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_WRITE, UT_I2C_REG->DR );

    Ut_I2c_Task( I2C_SR1_ADDR );

    Ut_I2c_Task( I2C_SR1_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x10u, UT_I2C_REG->DR );

    Ut_I2c_Task( I2C_SR1_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_I2C_REG->DR );

    Ut_I2c_Task( I2C_SR1_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_I2C_REG->DR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );

    Ut_I2c_Task( I2C_SR1_TXE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );

    /* Task without running transfer does nothing */
    Ut_I2c_Task( I2C_SR1_TXE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
}


/**
 * \brief   Polling write + read uses repeated START and BTF reception of 3 bytes.
 *
 * \details Starts 1 byte write + 3 byte read:
 * 1. SB, ADDR, TXE - byte written; TXE + BTF - read phase; TXE + BTF again (flags
 *    of the write phase before the repeated START is sent),
 * 2. SB - read address, ADDR (3 bytes),
 * 3. BTF (DR 0x21) - first byte, BTF (DR 0x22) - last two bytes.
 *
 * \par Expected results
 * 1. DR = 0x10, then repeated START (CR1.START), no STOP; stale TXE + BTF are not
 *    processed as received data (no STOP, no ACK change, RX buffer untouched).
 * 2. DR = 0xA1, ACK = 1 after ADDR.
 * 3. ACK = 0 after the first byte; STOP and RX buffer { 0x21, 0x22, 0x22 },
 *    complete callback once, ACK / POS cleared.
 */
void Ut_I2c_Task_PollWriteRead_RepeatedStartAndBtfReception( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_WRITE, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_ADDR );
    Ut_I2c_Task( I2C_SR1_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x10u, UT_I2C_REG->DR );

    Ut_I2c_Task( I2C_SR1_TXE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_START, UT_I2C_REG->CR1 & ( I2C_CR1_START | I2C_CR1_STOP ) );

    /* TXE / BTF of the write phase stay set until the repeated START is sent - not read data */
    Ut_I2c_Task( I2C_SR1_TXE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_START, UT_I2C_REG->CR1 & ( I2C_CR1_START | I2C_CR1_STOP | I2C_CR1_ACK ) );
    TEST_ASSERT_EQUAL_HEX8( 0u, utI2c_RxBuf[ 0u ] );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_READ, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_ACK, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_POS ) );

    /* RXNE of the first byte alone - BTF reception of the last 3 bytes waits */
    UT_I2C_REG->DR = 0x21u;
    Ut_I2c_Task( I2C_SR1_RXNE );
    TEST_ASSERT_EQUAL_HEX8( 0u, utI2c_RxBuf[ 0u ] );

    Ut_I2c_Task( I2C_SR1_RXNE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX8( 0x21u, utI2c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_ACK );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );

    UT_I2C_REG->DR = 0x22u;
    Ut_I2c_Task( I2C_SR1_RXNE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    TEST_ASSERT_EQUAL_HEX8( 0x22u, utI2c_RxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x22u, utI2c_RxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_HEX8( 0u,    utI2c_RxBuf[ 3u ] );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_POS ) );
}


/**
 * \brief   Polling read of 1 byte: NACK and STOP on ADDR.
 *
 * \details Starts 1 byte read: SB, ADDR, RXNE (DR 0x5A).
 *
 * \par Expected results
 * - SB: DR = 0xA1. ADDR: ACK = 0, STOP generated, no callback yet.
 * - RXNE: RX buffer[0] = 0x5A, complete callback once.
 */
void Ut_I2c_Task_PollReadOneByte_NackAndStopOnAddr( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 1u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_READ, UT_I2C_REG->DR );

    Ut_I2c_Task( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_POS | I2C_CR1_STOP ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );

    UT_I2C_REG->DR = 0x5Au;
    Ut_I2c_Task( I2C_SR1_RXNE );
    TEST_ASSERT_EQUAL_HEX8( 0x5Au, utI2c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0u,    utI2c_RxBuf[ 1u ] );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   Polling read of 2 bytes: POS on ADDR, both bytes on BTF.
 *
 * \details Starts 2 byte read: SB, ADDR, RXNE (ignored), RXNE + BTF (DR 0x6B).
 *
 * \par Expected results
 * - ADDR: POS = 1, ACK = 0.
 * - RXNE alone: no byte read.
 * - BTF: STOP, RX buffer { 0x6B, 0x6B }, complete callback, POS cleared.
 */
void Ut_I2c_Task_PollReadTwoBytes_PosAndBtf( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 2u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    Ut_I2c_Task( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_POS, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_POS ) );

    UT_I2C_REG->DR = 0x6Bu;
    Ut_I2c_Task( I2C_SR1_RXNE );
    TEST_ASSERT_EQUAL_HEX8( 0u, utI2c_RxBuf[ 0u ] );

    Ut_I2c_Task( I2C_SR1_RXNE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    TEST_ASSERT_EQUAL_HEX8( 0x6Bu, utI2c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x6Bu, utI2c_RxBuf[ 1u ] );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_POS );
}


/**
 * \brief   Polling read of 5 bytes: RXNE bytes, then BTF procedure of the last 3.
 *
 * \details RXNE (0x31), RXNE (0x32), RXNE alone (waits), BTF (0x33), BTF (0x34).
 *
 * \par Expected results
 * - RX buffer { 0x31, 0x32, 0x33, 0x34, 0x34 }, ACK cleared before byte N-2 is read,
 *   STOP before the last two bytes, complete callback once.
 */
void Ut_I2c_Task_PollReadFiveBytes_RxneThenBtfProcedure( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 5u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    Ut_I2c_Task( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_ACK, UT_I2C_REG->CR1 & I2C_CR1_ACK );

    UT_I2C_REG->DR = 0x31u;
    Ut_I2c_Task( I2C_SR1_RXNE );
    UT_I2C_REG->DR = 0x32u;
    Ut_I2c_Task( I2C_SR1_RXNE );
    UT_I2C_REG->DR = 0x33u;
    Ut_I2c_Task( I2C_SR1_RXNE );
    TEST_ASSERT_EQUAL_HEX8( 0u, utI2c_RxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_ACK, UT_I2C_REG->CR1 & I2C_CR1_ACK );

    Ut_I2c_Task( I2C_SR1_RXNE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_STOP ) );

    UT_I2C_REG->DR = 0x34u;
    Ut_I2c_Task( I2C_SR1_RXNE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );

    TEST_ASSERT_EQUAL_HEX8( 0x31u, utI2c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x32u, utI2c_RxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x33u, utI2c_RxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x34u, utI2c_RxBuf[ 3u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x34u, utI2c_RxBuf[ 4u ] );
    TEST_ASSERT_EQUAL_HEX8( 0u,    utI2c_RxBuf[ 5u ] );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   Polling address only transfer ends with STOP on ADDR.
 *
 * \details Starts address only transfer: SB, ADDR.
 *
 * \par Expected results
 * - DR = 0xA0, STOP on ADDR, complete callback once.
 */
void Ut_I2c_Task_PollAddressOnly_StopOnAddr( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_WRITE, UT_I2C_REG->DR );

    Ut_I2c_Task( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   Polling NACK ends the transfer with STOP and NACK error.
 *
 * \details Starts write, SB, then AF (address not acknowledged); a second transfer
 *          is started after STOP was sent.
 *
 * \par Expected results
 * - AF cleared, STOP generated, error callback with I2C_XFER_ERROR_NACK, no complete
 *   callback, transfer INACTIVE.
 * - Next transfer can be started after STOP was sent.
 */
void Ut_I2c_Task_PollNack_StopAndNackError( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    Ut_I2c_Task( I2C_SR1_AF );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->SR1 & I2C_SR1_AF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NACK );

    Ut_I2c_Set_StopSent();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
}


/**
 * \brief   Arbitration lost ends the transfer without STOP.
 *
 * \details Starts write, SB, then ARLO.
 *
 * \par Expected results
 * - ARLO cleared, no STOP, error callback with I2C_XFER_ERROR_ARBITRATION_LOST.
 */
void Ut_I2c_Task_PollArbitrationLost_ErrorWithoutStop( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    Ut_I2c_Task( I2C_SR1_ARLO | I2C_SR1_TXE );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->SR1 & I2C_SR1_ARLO );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_ARBITRATION_LOST );
}


/**
 * \brief   Bus error in master mode is ignored (errata), outside master mode reported.
 *
 * \details 1. BERR with SR2.MSL = 1 during write, 2. BERR with MSL = 0.
 *
 * \par Expected results
 * 1. BERR cleared, no callback, transfer continues (TXE writes the first byte).
 * 2. Error callback with I2C_XFER_ERROR_BUS, peripheral reset by SWRST
 *    (SWRST released, PE = 1, CCR / TRISE / FREQ restored), transfer INACTIVE.
 */
void Ut_I2c_Task_PollBusError_IgnoredInMasterModeReportedOtherwise( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    Ut_I2c_Task( I2C_SR1_SB );
    Ut_I2c_Task( I2C_SR1_ADDR );

    UT_I2C_REG->SR2 = I2C_SR2_MSL;
    Ut_I2c_Task( I2C_SR1_BERR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->SR1 & I2C_SR1_BERR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );

    Ut_I2c_Task( I2C_SR1_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x10u, UT_I2C_REG->DR );

    UT_I2C_REG->SR2 = 0u;
    Ut_I2c_Task( I2C_SR1_BERR );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_BUS );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & ( I2C_CR1_PE | I2C_CR1_SWRST ) );
    TEST_ASSERT_EQUAL_HEX32( 210u, UT_I2C_REG->CCR );
    TEST_ASSERT_EQUAL_UINT32( 43u, UT_I2C_REG->TRISE );
    TEST_ASSERT_EQUAL_UINT32( 42u, UT_I2C_REG->CR2 & I2C_CR2_FREQ );
}


/**
 * \brief   10-bit write: header with address bits 9:8, second address byte on ADD10.
 *
 * \details 10-bit mode, 1 byte write to 0x2A5: SB, ADD10, ADDR, TXE, TXE + BTF.
 *
 * \par Expected results
 * - SB: DR = 0xF4 (11110 10 0), ADD10: DR = 0xA5, TXE: DR = 0x10, STOP, complete.
 */
void Ut_I2c_Task_Poll10BitWrite_HeaderAndSecondAddressByte( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR10, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_HEADER10_WRITE, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_ADD10 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR10_LOW, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_ADDR );
    Ut_I2c_Task( I2C_SR1_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x10u, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_TXE | I2C_SR1_BTF );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   10-bit read: address in write direction, repeated START, read header.
 *
 * \details 10-bit mode, 1 byte read from 0x2A5: SB, ADD10, ADDR, SB, ADDR, RXNE.
 *
 * \par Expected results
 * - SB: DR = 0xF4, ADD10: DR = 0xA5, ADDR: repeated START (no STOP).
 * - SB: DR = 0xF5 (read header), ADDR: NACK + STOP, RXNE: byte stored, complete.
 */
void Ut_I2c_Task_Poll10BitRead_RepeatedStartWithReadHeader( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR10, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 1u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_HEADER10_WRITE, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_ADD10 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR10_LOW, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_START, UT_I2C_REG->CR1 & ( I2C_CR1_START | I2C_CR1_STOP ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Task( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_HEADER10_READ, UT_I2C_REG->DR );
    Ut_I2c_Task( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_STOP ) );

    UT_I2C_REG->DR = 0x77u;
    Ut_I2c_Task( I2C_SR1_RXNE );
    TEST_ASSERT_EQUAL_HEX8( 0x77u, utI2c_RxBuf[ 0u ] );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   I2c_Set_XferStop() aborts running transfer by software reset.
 *
 * \details Starts write, SB, then I2c_Set_XferStop(); I2c_Set_XferStop() without
 *          transfer and with invalid peripheral.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, no callback, transfer INACTIVE, error NONE.
 * - SWRST released, PE = 1, configuration (FREQ, CCR, TRISE, OAR1) restored.
 * - Without transfer: OK; invalid peripheral: error.
 */
void Ut_I2c_Set_XferStop_RunningTransfer_AbortedBySoftwareReset( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t       xferError  = I2C_XFER_ERROR_CNT;

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    Ut_I2c_Task( I2C_SR1_SB );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStop( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt + utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( UT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NONE, xferError );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & ( I2C_CR1_PE | I2C_CR1_SWRST ) );
    TEST_ASSERT_EQUAL_UINT32( 42u, UT_I2C_REG->CR2 & I2C_CR2_FREQ );
    TEST_ASSERT_EQUAL_HEX32( 210u, UT_I2C_REG->CCR );
    TEST_ASSERT_EQUAL_UINT32( 43u, UT_I2C_REG->TRISE );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_OAR1_BIT14, UT_I2C_REG->OAR1 );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStop( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStop( I2C_PERIPH_CNT ) );
}


/**
 * \brief   Transfer state and error getters reject invalid arguments.
 *
 * \details Invalid peripheral and NULL pointers.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases.
 */
void Ut_I2c_Get_XferStateError_InvalidArgs_ReturnsError( void )
{
    i2c_FunctionState_t xferState = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t   xferError = I2C_XFER_ERROR_CNT;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_XferState( I2C_PERIPH_CNT, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_XferState( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_XferError( I2C_PERIPH_CNT, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_XferError( UT_I2C_PERIPH, NULL ) );
}

/* ============================ ISR TRANSFERS =============================== */

/**
 * \brief   ISR write: event / error interrupts during the transfer, buffer
 *          interrupt only while data bytes are written.
 *
 * \details ISR mode, 2 byte write: start, SB, ADDR, TXE, TXE, TXE + BTF.
 *
 * \par Expected results
 * - After start: ITEVTEN | ITERREN, ITBUFEN off.
 * - After ADDR: ITBUFEN on; after the last byte: ITBUFEN off.
 * - BTF: STOP, complete callback, all interrupt sources disabled.
 */
void Ut_I2c_Isr_Write_BufferInterruptOnlyForData( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_ITEVTEN | I2C_CR2_ITERREN, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_WRITE, UT_I2C_REG->DR );

    Ut_I2c_Call_Isr( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2_IT_ALL, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );

    Ut_I2c_Call_Isr( I2C_SR1_TXE );
    Ut_I2c_Call_Isr( I2C_SR1_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_I2C_REG->DR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_ITEVTEN | I2C_CR2_ITERREN, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );

    Ut_I2c_Call_Isr( I2C_SR1_TXE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   ISR read of 4 bytes: buffer interrupt until 3 bytes remain.
 *
 * \details ISR mode, 4 byte read: SB, ADDR, RXNE (0x41), BTF (0x42), BTF (0x43).
 *
 * \par Expected results
 * - After ADDR: ITBUFEN on, ACK = 1.
 * - After the first byte: ITBUFEN off (BTF procedure).
 * - RX buffer { 0x41, 0x42, 0x43, 0x43 }, complete callback, interrupts disabled.
 */
void Ut_I2c_Isr_ReadFourBytes_BufferInterruptUntilBtfProcedure( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 4u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_READ, UT_I2C_REG->DR );

    Ut_I2c_Call_Isr( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2_IT_ALL, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_ACK, UT_I2C_REG->CR1 & I2C_CR1_ACK );

    UT_I2C_REG->DR = 0x41u;
    Ut_I2c_Call_Isr( I2C_SR1_RXNE );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_ITEVTEN | I2C_CR2_ITERREN, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );

    UT_I2C_REG->DR = 0x42u;
    Ut_I2c_Call_Isr( I2C_SR1_RXNE | I2C_SR1_BTF );
    UT_I2C_REG->DR = 0x43u;
    Ut_I2c_Call_Isr( I2C_SR1_RXNE | I2C_SR1_BTF );

    TEST_ASSERT_EQUAL_HEX8( 0x41u, utI2c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x42u, utI2c_RxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x43u, utI2c_RxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x43u, utI2c_RxBuf[ 3u ] );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & UT_I2C_CR2_IT_ALL );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   ISR without running transfer ignores the flags.
 *
 * \details ISR mode initialized, ISR called with SB, ADDR, AF without transfer.
 *
 * \par Expected results
 * - No callback, DR not written.
 */
void Ut_I2c_Isr_NoTransfer_FlagsIgnored( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );

    Ut_I2c_Init( &dataConfig );

    Ut_I2c_Call_Isr( I2C_SR1_SB | I2C_SR1_ADDR | I2C_SR1_AF );

    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt + utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->DR );
}

/* ============================ DMA TRANSFERS =============================== */

/**
 * \brief   DMA write + read: streams armed, BTF ends the write phase, stream
 *          transfer complete ends the read phase.
 *
 * \details DMA mode, 2 byte write + 3 byte read:
 * 1. start - TX and RX stream armed, DMAEN | LAST, event / error interrupts,
 * 2. SB, ADDR, BTF with 1 byte remaining in TX stream (ignored), BTF with empty
 *    stream - repeated START,
 * 3. SB, ADDR, RX stream transfer complete callback.
 *
 * \par Expected results
 * 1. Dma_Set_MemoryAddr / DataCount / TransferActive for both streams, CR2 bits set.
 * 2. Buffer interrupt stays disabled, no STOP while TX stream not empty, then START.
 * 3. DR = 0xA1, ACK = 1; STOP, complete callback, DMAEN / LAST / interrupts cleared.
 */
void Ut_I2c_Dma_WriteRead_StreamsArmedAndRxCompleteEndsTransfer( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Get_DataCount_StubWithCallback( Ut_I2c_DmaGetCountStub );

    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_6, (dma_MemoryAddr_t)(uintptr_t)utI2c_TxBuf, DMA_REQUEST_OK );
    Dma_Set_DataCount_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_6, 2u, DMA_REQUEST_OK );
    Dma_Set_TransferActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_6, DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_0, (dma_MemoryAddr_t)(uintptr_t)utI2c_RxBuf, DMA_REQUEST_OK );
    Dma_Set_DataCount_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_0, 3u, DMA_REQUEST_OK );
    Dma_Set_TransferActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_0, DMA_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_DMAEN | I2C_CR2_LAST | I2C_CR2_ITEVTEN | I2C_CR2_ITERREN,
                             UT_I2C_REG->CR2 & ( I2C_CR2_DMAEN | I2C_CR2_LAST | UT_I2C_CR2_IT_ALL ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_SR1_SB );
    Ut_I2c_Call_Isr( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_ITBUFEN );

    utI2c_DmaRemaining = 1u;
    Ut_I2c_Call_Isr( I2C_SR1_TXE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_START | I2C_CR1_STOP ) );

    utI2c_DmaRemaining = 0u;
    Ut_I2c_Call_Isr( I2C_SR1_TXE | I2C_SR1_BTF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_START, UT_I2C_REG->CR1 & ( I2C_CR1_START | I2C_CR1_STOP ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_SR1_SB );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_ADDR_READ, UT_I2C_REG->DR );
    Ut_I2c_Call_Isr( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_ACK, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_POS ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );

    TEST_ASSERT_NOT_NULL( utI2c_DmaConfig[ 1u ].TransferCompleteCallback );
    utI2c_DmaConfig[ 1u ].TransferCompleteCallback();

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & I2C_CR1_STOP );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & ( I2C_CR2_DMAEN | I2C_CR2_LAST | UT_I2C_CR2_IT_ALL ) );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   DMA read of 1 byte is done by the I2C buffer interrupt.
 *
 * \details DMA mode, 1 byte read: start, SB, ADDR, RXNE (0x3C).
 *
 * \par Expected results
 * - No stream is armed, DMAEN / LAST not set.
 * - ADDR: NACK + STOP, ITBUFEN on; RXNE: byte stored, complete callback.
 */
void Ut_I2c_Dma_ReadOneByte_ByBufferInterrupt( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 1u };

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();
    Ut_I2c_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & ( I2C_CR2_DMAEN | I2C_CR2_LAST ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_SR1_SB );
    Ut_I2c_Call_Isr( I2C_SR1_ADDR );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_STOP, UT_I2C_REG->CR1 & ( I2C_CR1_ACK | I2C_CR1_STOP ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_ITBUFEN, UT_I2C_REG->CR2 & I2C_CR2_ITBUFEN );

    UT_I2C_REG->DR = 0x3Cu;
    Ut_I2c_Call_Isr( I2C_SR1_RXNE );
    TEST_ASSERT_EQUAL_HEX8( 0x3Cu, utI2c_RxBuf[ 0u ] );
    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_NONE );
}


/**
 * \brief   DMA transfer error aborts the transfer.
 *
 * \details DMA mode, write: start, SB, ADDR, transfer error callback of TX stream;
 *          RX transfer complete callback without transfer.
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_DMA_TRANSFER, peripheral reset (PE = 1,
 *   SWRST released), transfer INACTIVE.
 * - RX transfer complete without transfer: no callback.
 */
void Ut_I2c_Dma_TransferError_TransferAborted( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();
    Ut_I2c_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    Ut_I2c_Call_Isr( I2C_SR1_SB );
    Ut_I2c_Call_Isr( I2C_SR1_ADDR );

    TEST_ASSERT_NOT_NULL( utI2c_DmaConfig[ 0u ].TransferErrorCallback );
    utI2c_DmaConfig[ 0u ].TransferErrorCallback();

    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_DMA_TRANSFER );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & ( I2C_CR1_PE | I2C_CR1_SWRST ) );

    utI2c_DmaConfig[ 1u ].TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
}

/* ===================== DATA HANDLING RELEASE / INSTANCES ================== */

/** Count of Dma_Set_InterruptInactive() calls (DMA stream released) */
static uint32_t             utI2c_DmaIrqOffCnt;

/** Return value of Dma_Set_InterruptInactive() stub */
static dma_RequestState_t   utI2c_DmaIrqOffState;

/** ISR registered in NVIC for any I2C peripheral */
static nvic_IsrCallback_t   utI2c_AnyIsr;

/** Dma_Set_InterruptInactive() stub counting the calls */
static dma_RequestState_t Ut_I2c_DmaIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utI2c_DmaIrqOffCnt++;

    return ( utI2c_DmaIrqOffState );
}


/** RCC peripheral clock stub of any I2C peripheral - returns PCLK1 ef utI2c_ClkHz */
static rcc_RequestState_t Ut_I2c_RccAnyClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *periphClk = utI2c_ClkHz;

    return ( RCC_REQUEST_OK );
}


/** NVIC handler registration stub of any I2C peripheral */
static nvic_RequestState_t Ut_I2c_NvicAnyHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utI2c_AnyIsr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief   I2c_Deinit() releases both DMA streams of DMA mode.
 *
 * \details I2C1 in DMA mode (TX stream 6, RX stream 0), I2c_Deinit() twice.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, Dma_Set_InterruptInactive() called for both streams (2x),
 *   CR2: DMAEN, LAST and interrupt sources cleared.
 * - Second I2c_Deinit(): I2C_REQUEST_OK, streams not touched again.
 */
void Ut_I2c_Dma_Deinit_StreamsReleased( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    Ut_I2c_Init( &dataConfig );
    UT_I2C_REG->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST | UT_I2C_CR2_IT_ALL;

    utI2c_DmaIrqOffCnt   = 0u;
    utI2c_DmaIrqOffState = DMA_REQUEST_OK;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & ( I2C_CR2_DMAEN | I2C_CR2_LAST | UT_I2C_CR2_IT_ALL ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
}


/**
 * \brief   Failure of DMA stream release is reported by I2c_Deinit().
 *
 * \details I2C1 in DMA mode, Dma_Set_InterruptInactive() returns DMA_REQUEST_ERROR.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, both streams released anyway (2 calls), data handling not
 *   initialized any more.
 */
void Ut_I2c_Dma_Deinit_StreamReleaseError_ReturnsError( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    i2c_DataConfig_t       readConfig;

    Ut_I2c_Init( &dataConfig );

    utI2c_DmaIrqOffCnt   = 0u;
    utI2c_DmaIrqOffState = DMA_REQUEST_ERROR;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
}


/**
 * \brief   I2c_Set_DataConfig() releases the previous data handling.
 *
 * \details I2C1 in DMA mode, reconfigured to POLL mode, then to ISR mode, then
 *          I2c_Deinit().
 *
 * \par Expected results
 * - DMA -> POLL: I2C_REQUEST_OK, both DMA streams released, I2C interrupts disabled.
 * - POLL -> ISR: I2C_REQUEST_OK, read back configuration is ISR mode.
 * - I2c_Deinit(): I2C_REQUEST_OK.
 */
void Ut_I2c_Set_DataConfig_Reconfigured_PreviousModeReleased( void )
{
    const i2c_DataConfig_t dmaConfig  = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_DataConfig_t pollConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_DataConfig_t isrConfig  = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    i2c_DataConfig_t       readConfig;

    Ut_I2c_Init( &dmaConfig );

    utI2c_DmaIrqOffCnt   = 0u;
    utI2c_DmaIrqOffState = DMA_REQUEST_OK;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DataConfig( UT_I2C_PERIPH, &pollConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & ( I2C_CR2_DMAEN | UT_I2C_CR2_IT_ALL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( I2C_XFER_MODE_POLL, readConfig.XferMode );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DataConfig( UT_I2C_PERIPH, &isrConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( I2C_XFER_MODE_ISR, readConfig.XferMode );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );
}


/**
 * \brief   DMA receive error aborts the running transfer.
 *
 * \details DMA mode, 3 byte read: start, SB, ADDR, transfer error callback of the
 *          RX stream.
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_DMA_TRANSFER, transfer INACTIVE, peripheral
 *   enabled after the software reset.
 */
void Ut_I2c_Dma_RxTransferError_TransferAborted( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    Ut_I2c_Call_Isr( I2C_SR1_SB );
    Ut_I2c_Call_Isr( I2C_SR1_ADDR );

    TEST_ASSERT_NOT_NULL( utI2c_DmaConfig[ 1u ].TransferErrorCallback );
    utI2c_DmaConfig[ 1u ].TransferErrorCallback();

    Ut_I2c_Check_XferEnd( I2C_XFER_ERROR_DMA_TRANSFER );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & ( I2C_CR1_PE | I2C_CR1_SWRST ) );
}


/**
 * \brief   Every I2C peripheral uses its own ISR and DMA callbacks on every stream pair.
 *
 * \details Every item of the DMA stream lists of every I2C peripheral of the MCU (DMA1 request
 *          table of the device line), items paired by the I2C peripheral, in DMA mode, no
 *          transfer is running. Captured ISR is called without flags, DMA callbacks (TX error,
 *          RX error, RX complete) of both streams are called.
 *
 * \par Expected results
 * - Streams of the list items are accepted and configured with the channel selections of the
 *   request table, channel selection stored in the items equals the request table.
 * - ISR without transfer: no callback, CR1 not changed.
 * - TX / RX error: error callback with I2C_XFER_ERROR_DMA_TRANSFER (transfer not
 *   running - only reported), RX complete: no callback.
 * - I2c_Deinit(): I2C_REQUEST_OK, both streams released.
 */
void Ut_I2c_Dma_AllStreams_OwnPeripheralReported( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const struct
    {
        i2c_PeriphId_t       PeriphId;
        I2C_TypeDef *        PeriphReg;
        i2c_TxDma_t          TxDma;
        i2c_RxDma_t          RxDma;
        i2c_DmaChannelId_t   TxStream;
        i2c_DmaChannelId_t   RxStream;
        dma_PeriphReqId_t    TxSel;
        dma_PeriphReqId_t    RxSel;
    }   periphLut[] =
    {
        { I2C_PERIPH_1, I2C1, I2C_TX_DMA_I2C1_DMA1_STREAM6, I2C_RX_DMA_I2C1_DMA1_STREAM0, I2C_DMA_CHANNEL_6, I2C_DMA_CHANNEL_0, DMA_REQ_CHANNEL_1, DMA_REQ_CHANNEL_1 },
        { I2C_PERIPH_1, I2C1, I2C_TX_DMA_I2C1_DMA1_STREAM7, I2C_RX_DMA_I2C1_DMA1_STREAM5, I2C_DMA_CHANNEL_7, I2C_DMA_CHANNEL_5, DMA_REQ_CHANNEL_1, DMA_REQ_CHANNEL_1 },
#if defined(I2C_AF_MAP_F410_F423)
        { I2C_PERIPH_1, I2C1, I2C_TX_DMA_I2C1_DMA1_STREAM1, I2C_RX_DMA_I2C1_DMA1_STREAM0, I2C_DMA_CHANNEL_1, I2C_DMA_CHANNEL_0, DMA_REQ_CHANNEL_0, DMA_REQ_CHANNEL_1 },
#endif /* I2C_AF_MAP_F410_F423 */
#ifdef I2C2
        { I2C_PERIPH_2, I2C2, I2C_TX_DMA_I2C2_DMA1_STREAM7, I2C_RX_DMA_I2C2_DMA1_STREAM2, I2C_DMA_CHANNEL_7, I2C_DMA_CHANNEL_2, DMA_REQ_CHANNEL_7, DMA_REQ_CHANNEL_7 },
        { I2C_PERIPH_2, I2C2, I2C_TX_DMA_I2C2_DMA1_STREAM7, I2C_RX_DMA_I2C2_DMA1_STREAM3, I2C_DMA_CHANNEL_7, I2C_DMA_CHANNEL_3, DMA_REQ_CHANNEL_7, DMA_REQ_CHANNEL_7 },
#endif /* I2C2 */
#ifdef I2C3
        { I2C_PERIPH_3, I2C3, I2C_TX_DMA_I2C3_DMA1_STREAM4, I2C_RX_DMA_I2C3_DMA1_STREAM2, I2C_DMA_CHANNEL_4, I2C_DMA_CHANNEL_2, DMA_REQ_CHANNEL_3, DMA_REQ_CHANNEL_3 },
#if defined(I2C_AF_MAP_F401)      || \
    defined(I2C_AF_MAP_F410_F423) || \
    defined(I2C_AF_MAP_F446)
        { I2C_PERIPH_3, I2C3, I2C_TX_DMA_I2C3_DMA1_STREAM4, I2C_RX_DMA_I2C3_DMA1_STREAM1, I2C_DMA_CHANNEL_4, I2C_DMA_CHANNEL_1, DMA_REQ_CHANNEL_3, DMA_REQ_CHANNEL_1 },
#endif /* I2C_AF_MAP_F401 OR I2C_AF_MAP_F410_F423 OR I2C_AF_MAP_F446 */
#if defined(I2C_AF_MAP_F401)      || \
    defined(I2C_AF_MAP_F410_F423)
        { I2C_PERIPH_3, I2C3, I2C_TX_DMA_I2C3_DMA1_STREAM5, I2C_RX_DMA_I2C3_DMA1_STREAM2, I2C_DMA_CHANNEL_5, I2C_DMA_CHANNEL_2, DMA_REQ_CHANNEL_6, DMA_REQ_CHANNEL_3 },
#endif /* I2C_AF_MAP_F401 OR I2C_AF_MAP_F410_F423 */
#endif /* I2C3 */
    };

    for( uint32_t idx = 0u; ( sizeof( periphLut ) / sizeof( periphLut[ 0u ] ) ) > idx; idx++ )
    {
        i2c_Config_t config = Ut_I2c_Get_Config();

        dataConfig.TxDma  = periphLut[ idx ].TxDma;
        dataConfig.RxDma  = periphLut[ idx ].RxDma;
        config.PeriphId   = periphLut[ idx ].PeriphId;
        config.DataConfig = &dataConfig;

        Ut_I2c_Reset_Mocks();
        Ut_I2c_Ignore_PeriphMocks();
        Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicAnyHandlerStub );
        Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccAnyClkStub );
        utI2c_AnyIsr     = NULL;
        utI2c_DmaInitCnt = 0u;

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
        TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );
        TEST_ASSERT_EQUAL( DMA_PERIPH_1,                                utI2c_DmaConfig[ 0u ].DmaPeriphId );
        TEST_ASSERT_EQUAL( DMA_PERIPH_1,                                utI2c_DmaConfig[ 1u ].DmaPeriphId );
        TEST_ASSERT_EQUAL( (dma_ChannelId_t)periphLut[ idx ].TxStream,  utI2c_DmaConfig[ 0u ].DmaChannel );
        TEST_ASSERT_EQUAL( (dma_ChannelId_t)periphLut[ idx ].RxStream,  utI2c_DmaConfig[ 1u ].DmaChannel );
        TEST_ASSERT_EQUAL( periphLut[ idx ].TxSel,                      utI2c_DmaConfig[ 0u ].PeripheralReqId );
        TEST_ASSERT_EQUAL( periphLut[ idx ].RxSel,                      utI2c_DmaConfig[ 1u ].PeripheralReqId );
        TEST_ASSERT_NOT_NULL( utI2c_AnyIsr );

        /* Channel selection of the list items equals the one used by the request table */
        TEST_ASSERT_EQUAL_UINT32( (uint32_t)periphLut[ idx ].TxSel >> DMA_SxCR_CHSEL_Pos, I2C_DMA_BIT_MASK_DECODE_CHSEL( periphLut[ idx ].TxDma ) );
        TEST_ASSERT_EQUAL_UINT32( (uint32_t)periphLut[ idx ].RxSel >> DMA_SxCR_CHSEL_Pos, I2C_DMA_BIT_MASK_DECODE_CHSEL( periphLut[ idx ].RxDma ) );

        const uint32_t cr1Value = periphLut[ idx ].PeriphReg->CR1;
        periphLut[ idx ].PeriphReg->SR1 = 0u;
        utI2c_AnyIsr();
        TEST_ASSERT_EQUAL_HEX32( cr1Value, periphLut[ idx ].PeriphReg->CR1 );
        TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );

        utI2c_DmaConfig[ 0u ].TransferErrorCallback();
        utI2c_DmaConfig[ 1u ].TransferErrorCallback();
        utI2c_DmaConfig[ 1u ].TransferCompleteCallback();

        TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_ErrorCnt );
        TEST_ASSERT_EQUAL( I2C_XFER_ERROR_DMA_TRANSFER, utI2c_LastError );
        TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );

        utI2c_DmaIrqOffCnt   = 0u;
        utI2c_DmaIrqOffState = DMA_REQUEST_OK;
        Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( periphLut[ idx ].PeriphId ) );
        TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );

        utI2c_ErrorCnt = 0u;
    }
}


/**
 * \brief   Items of the DMA stream lists carry I2C peripheral, DMA peripheral, stream and channel
 *          selection of the stream.
 *
 * \details Expected values are written as (I2C peripheral, DMA peripheral index, stream number,
 *          channel selection number) taken from the DMA1 request mapping of the STM32F4
 *          reference manuals, independently of the encoding macro.
 *
 * \par Expected results
 * - Every transmit and receive item of the I2C peripherals of the MCU carries the expected
 *   bit-fields.
 * - Unused items of both lists equal I2C_DMA_CODE_UNUSED, the decoding macros return the fields.
 */
void Ut_I2c_DmaLists_Items_EncodePeriphDmaStreamAndChannelSelection( void )
{
    /* I2C1 (DMA1) */
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_1, 0u, 6u, 1u ), I2C_TX_DMA_I2C1_DMA1_STREAM6 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_1, 0u, 7u, 1u ), I2C_TX_DMA_I2C1_DMA1_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_1, 0u, 0u, 1u ), I2C_RX_DMA_I2C1_DMA1_STREAM0 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_1, 0u, 5u, 1u ), I2C_RX_DMA_I2C1_DMA1_STREAM5 );
#if defined(I2C_AF_MAP_F410_F423)
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_1, 0u, 1u, 0u ), I2C_TX_DMA_I2C1_DMA1_STREAM1 );
#endif /* I2C_AF_MAP_F410_F423 */

#if defined(I2C2)
    /* I2C2 (DMA1, channel selection 7) */
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_2, 0u, 7u, 7u ), I2C_TX_DMA_I2C2_DMA1_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_2, 0u, 2u, 7u ), I2C_RX_DMA_I2C2_DMA1_STREAM2 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_2, 0u, 3u, 7u ), I2C_RX_DMA_I2C2_DMA1_STREAM3 );
#endif /* I2C2 */

#if defined(I2C3)
    /* I2C3 (DMA1) */
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_3, 0u, 4u, 3u ), I2C_TX_DMA_I2C3_DMA1_STREAM4 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_3, 0u, 2u, 3u ), I2C_RX_DMA_I2C3_DMA1_STREAM2 );
#if defined(I2C_AF_MAP_F401)      || \
    defined(I2C_AF_MAP_F410_F423)
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_3, 0u, 5u, 6u ), I2C_TX_DMA_I2C3_DMA1_STREAM5 );
#endif /* I2C_AF_MAP_F401 OR I2C_AF_MAP_F410_F423 */
#if defined(I2C_AF_MAP_F401)      || \
    defined(I2C_AF_MAP_F410_F423) || \
    defined(I2C_AF_MAP_F446)
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( I2C_PERIPH_3, 0u, 1u, 1u ), I2C_RX_DMA_I2C3_DMA1_STREAM1 );
#endif /* I2C_AF_MAP_F401 OR I2C_AF_MAP_F410_F423 OR I2C_AF_MAP_F446 */
#endif /* I2C3 */

    /* Decoding of the fields */
    TEST_ASSERT_EQUAL_UINT32( I2C_PERIPH_1,       I2C_DMA_BIT_MASK_DECODE_PERIPH( I2C_RX_DMA_I2C1_DMA1_STREAM5 ) );
    TEST_ASSERT_EQUAL_UINT32( I2C_DMA_PERIPH_1,   I2C_DMA_BIT_MASK_DECODE_DMA( I2C_RX_DMA_I2C1_DMA1_STREAM5 ) );
    TEST_ASSERT_EQUAL_UINT32( I2C_DMA_CHANNEL_5,  I2C_DMA_BIT_MASK_DECODE_STREAM( I2C_RX_DMA_I2C1_DMA1_STREAM5 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u,                 I2C_DMA_BIT_MASK_DECODE_CHSEL( I2C_RX_DMA_I2C1_DMA1_STREAM5 ) );

    /* Unused stream */
    TEST_ASSERT_EQUAL_HEX32( I2C_DMA_CODE_UNUSED, I2C_TX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_HEX32( I2C_DMA_CODE_UNUSED, I2C_RX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_UINT32( I2C_PERIPH_CNT,      I2C_DMA_BIT_MASK_DECODE_PERIPH( I2C_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( I2C_DMA_PERIPH_CNT,  I2C_DMA_BIT_MASK_DECODE_DMA( I2C_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( I2C_DMA_CHANNEL_CNT, I2C_DMA_BIT_MASK_DECODE_STREAM( I2C_TX_DMA_UNUSED ) );
}


/**
 * \brief   Interrupt service routine of every I2C ends with DSB.
 *
 * \details Every I2C of the MCU initialized in ISR mode, captured interrupt called once without
 *          flags and transfer, peripheral deinitialized.
 *
 * \note    Device errata bug AB#670 (ES0182 2.1.3 "Store immediate overlapping exception return
 *          operation might vector to incorrect interrupt", Arm ID 838869): a buffered store with
 *          immediate offset still pending at the exception return may vector to an incorrect
 *          interrupt. Workaround - DSB before the exception return of every handler.
 *
 * \par Expected results
 * - Every ISR executes exactly one DSB.
 */
void Ut_I2c_Isr_AllPeriphs_EndWithDsb( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );

    for( uint32_t periphId = 0u; I2C_PERIPH_CNT > periphId; periphId++ )
    {
        i2c_Config_t config = Ut_I2c_Get_Config();

        config.PeriphId   = (i2c_PeriphId_t)periphId;
        config.DataConfig = &dataConfig;

        Ut_I2c_Reset_Mocks();
        Ut_I2c_Ignore_PeriphMocks();
        Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicAnyHandlerStub );
        Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccAnyClkStub );
        utI2c_AnyIsr = NULL;

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
        TEST_ASSERT_NOT_NULL( utI2c_AnyIsr );

        const uint32_t dsbCnt = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB );

        utI2c_AnyIsr();

        TEST_ASSERT_EQUAL_UINT32( dsbCnt + 1u, CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( (i2c_PeriphId_t)periphId ) );
    }
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief NVIC handler registration stub - stores ISR of the I2C (event and error line).
 */
static nvic_RequestState_t Ut_I2c_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_TRUE( ( UT_I2C_NVIC_EV == irqId ) || ( UT_I2C_NVIC_ER == irqId ) );
    TEST_ASSERT_NOT_NULL( irqHandler );

    utI2c_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief RCC peripheral clock stub - returns PCLK1 \ref utI2c_ClkHz.
 */
static rcc_RequestState_t Ut_I2c_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_I2C_RCC, periphId );

    *periphClk = utI2c_ClkHz;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief GPIO initialization stub - stores the pin configurations.
 */
static gpio_RequestState_t Ut_I2c_GpioInitStub( gpio_Config_t *gpioConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( gpioConfig );
    TEST_ASSERT_LESS_THAN_UINT32( 2u, utI2c_GpioInitCnt );

    utI2c_GpioConfig[ utI2c_GpioInitCnt ] = *gpioConfig;
    utI2c_GpioInitCnt++;

    return ( GPIO_REQUEST_OK );
}


/**
 * \brief DMA initialization stub - stores the stream configurations (TX, RX).
 */
static dma_RequestState_t Ut_I2c_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( dmaConfig );

    utI2c_DmaConfig[ utI2c_DmaInitCnt % 2u ] = *dmaConfig;
    utI2c_DmaInitCnt++;

    return ( DMA_REQUEST_OK );
}


/**
 * \brief DMA count of data stub - returns \ref utI2c_DmaRemaining.
 */
static dma_RequestState_t Ut_I2c_DmaGetCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    *dataCount = utI2c_DmaRemaining;

    return ( DMA_REQUEST_OK );
}


/**
 * \brief Ignores all calls of RCC / NVIC / GPIO / DMA functions used by peripheral
 *        configuration and data handling.
 */
static void Ut_I2c_Ignore_PeriphMocks( void )
{
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Gpio_Init_IgnoreAndReturn( GPIO_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Dma_Get_DefaultConfig_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Init_StubWithCallback( Ut_I2c_DmaInitStub );
    Dma_Set_TransferErrorIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIrqActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferCompleteIrqInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_DataCount_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Get_DataCount_StubWithCallback( Ut_I2c_DmaGetCountStub );
}


/**
 * \brief Releases data handling of the previous test (static module context) and
 *        re-initializes the mocks.
 */
static void Ut_I2c_Release( void )
{
    Ut_I2c_Ignore_PeriphMocks();

    (void)I2c_Deinit( UT_I2C_PERIPH );

    Ut_I2c_Reset_Mocks();
}


/**
 * \brief Removes all expectations and ignores of the mocks.
 */
static void Ut_I2c_Reset_Mocks( void )
{
    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockDma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();
}


/**
 * \brief Returns default configuration of I2C1 without pins and data handling.
 */
static i2c_Config_t Ut_I2c_Get_Config( void )
{
    i2c_Config_t config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = UT_I2C_PERIPH;

    return ( config );
}


/**
 * \brief Returns data handling configuration with test callbacks (DMA: I2C1 TX
 *        stream 6, RX stream 0).
 *
 * \param xferMode [in]: Data transfer mode
 */
static i2c_DataConfig_t Ut_I2c_Get_DataConfig( i2c_XferMode_t xferMode )
{
    i2c_DataConfig_t dataConfig;

    dataConfig.XferMode             = xferMode;
    dataConfig.TxDma                = UT_I2C_DMA_TX;
    dataConfig.TxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    dataConfig.RxDma                = UT_I2C_DMA_RX;
    dataConfig.RxDmaPriority        = I2C_DMA_PRIORITY_HIGH;
    dataConfig.IrqPriority          = UT_I2C_PRIO;
    dataConfig.XferCompleteCallback = Ut_I2c_XferCompleteCallback;
    dataConfig.ErrorCallback        = Ut_I2c_ErrorCallback;

    return ( dataConfig );
}


/**
 * \brief Initializes I2C1 (100 kHz, PCLK1 42 MHz) with ignored RCC / NVIC / GPIO /
 *        DMA calls.
 *
 * \param dataConfig [in]: Data handling configuration (NULL - not initialized)
 */
static void Ut_I2c_Init( const i2c_DataConfig_t * const dataConfig )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.DataConfig = dataConfig;

    Ut_I2c_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
}


/**
 * \brief Calls I2c_Task() with given SR1 flags (polling mode).
 *
 * \param sr1Flags [in]: Value of the SR1 register
 */
static void Ut_I2c_Task( uint32_t sr1Flags )
{
    UT_I2C_REG->SR1 = sr1Flags;
    I2c_Task();
}


/**
 * \brief Calls the captured I2C ISR with given SR1 flags.
 *
 * \param sr1Flags [in]: Value of the SR1 register
 */
static void Ut_I2c_Call_Isr( uint32_t sr1Flags )
{
    TEST_ASSERT_NOT_NULL_MESSAGE( utI2c_Isr, "I2C ISR not registered" );

    UT_I2C_REG->SR1 = sr1Flags;
    utI2c_Isr();
}


/**
 * \brief Emulates hardware clearing of CR1.START after the START condition.
 */
static void Ut_I2c_Set_StartSent( void )
{
    UT_I2C_REG->CR1 &= ~I2C_CR1_START;
}


/**
 * \brief Emulates hardware clearing of CR1.STOP after the STOP condition.
 */
static void Ut_I2c_Set_StopSent( void )
{
    UT_I2C_REG->CR1 &= ~I2C_CR1_STOP;
}


/**
 * \brief Checks the end of the transfer - one callback of the expected result,
 *        transfer INACTIVE and the stored error.
 *
 * \param expError [in]: Expected result of the transfer
 */
static void Ut_I2c_Check_XferEnd( i2c_XferErrorId_t expError )
{
    i2c_FunctionState_t xferState = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t   xferError = I2C_XFER_ERROR_CNT;

    if( I2C_XFER_ERROR_NONE == expError )
    {
        TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
        TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
    }
    else
    {
        TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
        TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
        TEST_ASSERT_EQUAL( expError, utI2c_LastError );
    }

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( UT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( expError, xferError );
}


/** \brief Transfer complete callback */
static void Ut_I2c_XferCompleteCallback( void )
{
    utI2c_CompleteCnt++;
}


/**
 * \brief Transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void Ut_I2c_ErrorCallback( i2c_XferErrorId_t errorId )
{
    utI2c_LastError = errorId;
    utI2c_ErrorCnt++;
}
