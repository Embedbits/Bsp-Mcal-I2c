/**
 * \author Mr.Nobody
 * \file Test_I2c.c
 * \ingroup I2c
 * \brief Unit tests of Inter-Integrated Circuit (I2C) module.
 *
 * I2c.c, I2c_Isr.c, I2c_Poll.c and I2c_Dma.c are compiled unchanged with real
 * LL drivers. I2C registers are emulated by RegMem, RCC, NVIC, GPIO and GPDMA
 * modules are mocked by CMock. I2C ISR registered in NVIC is captured by stub
 * and called directly to test interrupt data handling.
 *
 * \note Emulated registers are plain memory:
 *       - ISR flags are not cleared by ICR writes and data register accesses,
 *         tests preset the flags handled by the module before each step,
 *       - CR2.START is not cleared by hardware after the address phase, tests
 *         clear it before the next request (\ref Ut_I2c_Set_StartSent),
 *       - ISR.TXE is set by the TX flush of the module (write of TXE to ISR).
 *
 * \note Data handling context of the module is static - setUp() releases the
 *       data handling of the previous test (I2c_Deinit with ignored mocks)
 *       and re-initializes the mocks.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "I2c_Port.h"                       /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockGpdma_Port.h"                 /* GPDMA module mock              */
#include "Stm32_i2c.h"                      /* I2C registers definition       */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_I2c_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetClkSrcStub     ( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static gpio_RequestState_t  Ut_I2c_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static void                 Ut_I2c_Ignore_PeriphMocks   ( void );
static void                 Ut_I2c_Release              ( void );
static void                 Ut_I2c_Reset_Mocks          ( void );
static i2c_Config_t         Ut_I2c_Get_Config           ( void );
static i2c_DataConfig_t     Ut_I2c_Get_DataConfig       ( i2c_XferMode_t xferMode );
static void                 Ut_I2c_Init                 ( const i2c_DataConfig_t * const dataConfig );
static void                 Ut_I2c_Call_Isr             ( uint32_t isrFlags );
static void                 Ut_I2c_Set_StartSent        ( void );
static uint32_t             Ut_I2c_Get_Cr2Xfer          ( void );

static void                 Ut_I2c_XferCompleteCallback ( void );
static void                 Ut_I2c_ErrorCallback        ( i2c_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** I2C peripheral used by tests (available on all supported MCUs) */
#define UT_I2C_PERIPH                       ( I2C_PERIPH_1 )
#define UT_I2C_REG                          ( I2C1 )
#define UT_I2C_RCC_PCLK                     ( RCC_PERIPH_I2C1_PCLK1 )
#define UT_I2C_RCC_HSI                      ( RCC_PERIPH_I2C1_HSI )
#define UT_I2C_NVIC_EV                      ( NVIC_PERIPH_IRQ_I2C1_EV )
#define UT_I2C_NVIC_ER                      ( NVIC_PERIPH_IRQ_I2C1_ER )

/** Kernel clock returned by RCC mock [Hz] */
#define UT_I2C_CLK_HZ                       ( 64000000u )

/** Interrupt priority of test configurations */
#define UT_I2C_PRIO                         ( 5u )

/** Slave address of test transfers (7-bit) */
#define UT_I2C_SLAVE_ADDR                   ( 0x50u )

/** Bus frequency tolerance of read back value [%] */
#define UT_I2C_FREQ_TOL_PCT                 ( 10u )

/** Size of long transfer (more than one NBYTES chunk) */
#define UT_I2C_LONG_SIZE                    ( 300u )

/* ============================== MACROS ==================================== */

/** Expected CR2 transfer fields (SADD, NBYTES, AUTOEND / RELOAD, RD_WRN) */
#define UT_I2C_CR2( ADDR, NBYTES, END, RD )     ( ( ( (uint32_t)(ADDR) << 1u ) & I2C_CR2_SADD )                           | \
                                                  ( ( (uint32_t)(NBYTES) << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES )      | \
                                                  (END) | (RD) )

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC for the I2C (event and error interrupt) */
static nvic_IsrCallback_t       utI2c_Isr;

/** GPIO configuration of the last Gpio_Init call */
static gpio_Config_t            utI2c_GpioConfig;

/** Transmit data */
static i2c_Data_t               utI2c_TxBuf[ UT_I2C_LONG_SIZE ];

/** Receive buffer */
static i2c_Data_t               utI2c_RxBuf[ 4u ];

/** Counts of callback calls */
static uint32_t                 utI2c_CompleteCnt;
static uint32_t                 utI2c_ErrorCnt;

/** Parameter of the last error callback */
static i2c_XferErrorId_t        utI2c_LastError;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    Ut_I2c_Release();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utI2c_Isr         = NULL;
    utI2c_CompleteCnt = 0u;
    utI2c_ErrorCnt    = 0u;
    utI2c_LastError   = I2C_XFER_ERROR_CNT;

    for( uint32_t byteIdx = 0u; UT_I2C_LONG_SIZE > byteIdx; byteIdx++ )
    {
        utI2c_TxBuf[ byteIdx ] = (i2c_Data_t)( 0x10u + byteIdx );
    }

    for( uint32_t byteIdx = 0u; sizeof( utI2c_RxBuf ) > byteIdx; byteIdx++ )
    {
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
 * - I2C_REQUEST_OK, I2C1, PCLK clock source, 100 kHz, analog filter enabled,
 *   digital filter off, 7-bit addressing, no data configuration, SCL / SDA pins
 *   unused, no pull.
 * - NULL pointer: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Get_DefaultConfig_ReturnsDefaults( void )
{
    i2c_Config_t config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( I2C_PERIPH_1,              config.PeriphId );
    TEST_ASSERT_EQUAL( I2C_CLK_SRC_PCLK,          config.ClkSrc );
    TEST_ASSERT_EQUAL_UINT32( 100000u,            config.BusFreq );
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_ENABLED, config.AnalogFilter );
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
 *          is invalid: peripheral, clock source, bus frequency 0 and above maximum,
 *          digital filter, address mode, pin pull.
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
    config.DigitalFilter = I2C_DIGITAL_FILTER_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.AddrMode = I2C_ADDR_MODE_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    config = Ut_I2c_Get_Config();
    config.PinPull = I2C_PIN_PULL_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
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
 * \brief   I2c_Init() activates clock, resets and enables the peripheral.
 *
 * \details Initializes I2C1 with default-like configuration (PCLK, 100 kHz, no pins,
 *          no data handling). RCC clock source and frequency (64 MHz) are stubbed.
 *
 * \par Expected results
 * - RCC: clock activated, reset pulse (active, inactive) for RCC_PERIPH_I2C1_PCLK1.
 * - I2C_REQUEST_OK, CR1.PE = 1, ANFOFF = 0, DNF = 0, 7-bit addressing.
 * - TIMINGR is calculated (not zero).
 */
void Ut_I2c_Init_DefaultConfig_ClockTimingAndEnable( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & ( I2C_CR1_PE | I2C_CR1_ANFOFF | I2C_CR1_DNF ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_ADD10 );
    TEST_ASSERT_NOT_EQUAL( 0u, UT_I2C_REG->TIMINGR );
}


/**
 * \brief   I2c_Init() activates the selected HSI kernel clock.
 *
 * \details Initializes I2C1 with HSI clock source.
 *
 * \par Expected results
 * - Clock activation and reset are requested for RCC_PERIPH_I2C1_HSI.
 * - I2C_REQUEST_OK is returned.
 */
void Ut_I2c_Init_HsiClockSource_SelectedClockActivated( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.ClkSrc = I2C_CLK_SRC_HSI;

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
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

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   I2c_Init() rejects zero kernel clock frequency.
 *
 * \details RCC returns kernel clock 0 Hz (timing can not be calculated).
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, peripheral is not enabled (PE = 0).
 */
void Ut_I2c_Init_KernelClockZero_ReturnsErrorPeripheralNotEnabled( void )
{
    static rcc_FreqHz_t zeroClk = 0u;
    i2c_Config_t        config  = Ut_I2c_Get_Config();

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &zeroClk );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   I2c_Init() configures SCL / SDA pins.
 *
 * \details Initializes I2C1 with SCL PB8, SDA PB9 and pull-up. GPIO initialization
 *          is captured by stub.
 *
 * \par Expected results
 * - I2C_REQUEST_OK.
 * - Last configured pin is PB9: alternate mode, open-drain, pull-up, AF4.
 */
void Ut_I2c_Init_Pins_GpioOpenDrainAlternateWithPull( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.SclPin  = I2C_SCL_PIN_I2C1_PB8;
    config.SdaPin  = I2C_SDA_PIN_I2C1_PB9;
    config.PinPull = I2C_PIN_PULL_UP;

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Gpio_Init_StubWithCallback( Ut_I2c_GpioInitStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    /* Last configured pin: SDA PB9 */
    TEST_ASSERT_EQUAL( GPIO_PORT_B,               utI2c_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_9,             utI2c_GpioConfig.PinId );
    TEST_ASSERT_EQUAL( GPIO_PIN_MODE_ALTERNATE,   utI2c_GpioConfig.PinMode );
    TEST_ASSERT_EQUAL( GPIO_PIN_OUTPUT_OPENDRAIN, utI2c_GpioConfig.PinOutType );
    TEST_ASSERT_EQUAL( GPIO_PIN_PULL_UP,          utI2c_GpioConfig.PinPull );
    TEST_ASSERT_EQUAL( GPIO_ALT_FUNC_4,           utI2c_GpioConfig.PinAltFunction );
}


/**
 * \brief   I2c_Init() configures noise filters and address mode.
 *
 * \details Initializes I2C1 with analog filter disabled, digital filter 5 and
 *          10-bit addressing.
 *
 * \par Expected results
 * - CR1.ANFOFF = 1, CR1.DNF = 5, CR2.ADD10 = 1.
 */
void Ut_I2c_Init_FiltersAndAddrMode_RegistersConfigured( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.AnalogFilter  = I2C_ANALOG_FILTER_DISABLED;
    config.DigitalFilter = I2C_DIGITAL_FILTER_5;
    config.AddrMode      = I2C_ADDR_MODE_10BIT;

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_ANFOFF, UT_I2C_REG->CR1 & I2C_CR1_ANFOFF );
    TEST_ASSERT_EQUAL_HEX32( 5u << I2C_CR1_DNF_Pos, UT_I2C_REG->CR1 & I2C_CR1_DNF );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_ADD10, UT_I2C_REG->CR2 & I2C_CR2_ADD10 );
}


/**
 * \brief   I2c_Init() with ISR data configuration configures NVIC.
 *
 * \details Initializes I2C1 with ISR transfer mode, priority 5. Priority and
 *          activation of event and error IRQ are expected, ISR captured by stub.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, ISR registered.
 * - Event and error IRQ priority 5 set and activated.
 * - I2c_Get_DataConfig() reads back ISR mode and transfer complete callback.
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
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_EV, UT_I2C_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_I2C_NVIC_ER, UT_I2C_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_I2C_NVIC_EV, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_I2C_NVIC_ER, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
    TEST_ASSERT_NOT_NULL( utI2c_Isr );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( I2C_XFER_MODE_ISR, readConfig.XferMode );
    TEST_ASSERT_EQUAL_PTR( Ut_I2c_XferCompleteCallback, readConfig.XferCompleteCallback );
}

/* =========================== DEINITIALIZATION ============================= */

/**
 * \brief   I2c_Deinit() disables the peripheral, resets it and stops its clock.
 *
 * \details Initializes I2C1 without data handling, then deinitializes it and an
 *          invalid peripheral.
 *
 * \par Expected results
 * - RCC reset pulse and clock deactivation of RCC_PERIPH_I2C1_PCLK1.
 * - I2C_REQUEST_OK, PE = 0.
 * - Invalid peripheral: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Deinit_InitializedPeripheral_DisabledResetAndClockOff( void )
{
    Ut_I2c_Init( NULL );
    Ut_I2c_Reset_Mocks();

    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Deinit( I2C_PERIPH_CNT ) );
}


/**
 * \brief   I2c_Deinit() deactivates interrupts and releases data handling.
 *
 * \details Initializes I2C1 in ISR mode and deinitializes it.
 *
 * \par Expected results
 * - Event and error IRQ deactivated, RCC reset pulse and clock deactivation.
 * - I2C_REQUEST_OK, I2c_Get_DataConfig() returns I2C_REQUEST_ERROR.
 */
void Ut_I2c_Deinit_IsrDataConfig_InterruptsDisabled( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    i2c_DataConfig_t       readConfig;

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_I2C_NVIC_EV, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_I2C_NVIC_ER, NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
}

/* =========================== PERIPHERAL STATE ============================= */

/**
 * \brief   I2c_Set_PeriphActive() / I2c_Set_PeriphInactive() control PE.
 *
 * \details Enables I2C1, reads its state, disables it and reads the state again.
 *
 * \par Expected results
 * - Enable: PE = 1, state ACTIVE.
 * - Disable: PE = 0, state INACTIVE.
 */
void Ut_I2c_Set_PeriphActive_EnabledAndReported( void )
{
    i2c_FlagState_t state = I2C_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphActive( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphState( UT_I2C_PERIPH, &state ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_PE );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphState( UT_I2C_PERIPH, &state ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, state );
}


/**
 * \brief   Peripheral state functions reject invalid arguments.
 *
 * \details Calls enable / disable / state read with invalid peripheral and state
 *          read with NULL pointer.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR is returned in all cases.
 */
void Ut_I2c_Set_PeriphState_InvalidArgs_ReturnsError( void )
{
    i2c_FlagState_t state = I2C_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_PeriphActive( I2C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_PeriphInactive( I2C_PERIPH_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_PeriphState( I2C_PERIPH_CNT, &state ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_PeriphState( UT_I2C_PERIPH, NULL ) );
}


/**
 * \brief   I2c_Set_PeriphInactive() is rejected while START is pending.
 *
 * \details Presets PE and CR2.START.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, peripheral stays enabled.
 */
void Ut_I2c_Set_PeriphInactive_StartPending_ReturnsErrorStaysEnabled( void )
{
    UT_I2C_REG->CR1 = I2C_CR1_PE;
    UT_I2C_REG->CR2 = I2C_CR2_START;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   I2c_Get_BusState() reports ISR.BUSY.
 *
 * \details Reads bus state with BUSY set, with BUSY cleared and with NULL pointer.
 *
 * \par Expected results
 * - BUSY set: ACTIVE; BUSY cleared: INACTIVE; NULL pointer: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Get_BusState_BusyFlag_Reported( void )
{
    i2c_FlagState_t busBusy = I2C_FLAG_INACTIVE;

    UT_I2C_REG->ISR = I2C_ISR_BUSY;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( UT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, busBusy );

    UT_I2C_REG->ISR = 0u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( UT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, busBusy );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusState( UT_I2C_PERIPH, NULL ) );
}

/* ============================ BUS FREQUENCY =============================== */

/**
 * \brief   I2c_Set_BusFreq() timing gives requested bus frequency.
 *
 * \details Sets 10 kHz, 100 kHz, 400 kHz and 1 MHz (kernel clock 64 MHz) and reads
 *          the frequency calculated from TIMINGR back.
 *
 * \par Expected results
 * - Every read frequency is within +-10 % of the requested one (never more than
 *   10 % above).
 */
void Ut_I2c_Set_BusFreq_StandardFastFastPlus_ReadBackWithinTolerance( void )
{
    const i2c_FreqHz_t freqs[] = { 10000u, 100000u, 400000u, 1000000u };
    i2c_FreqHz_t       busFreq = 0u;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    for( uint32_t idx = 0u; ( sizeof( freqs ) / sizeof( freqs[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, freqs[ idx ] ) );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( UT_I2C_PERIPH, &busFreq ) );

        TEST_ASSERT_UINT32_WITHIN( ( freqs[ idx ] * UT_I2C_FREQ_TOL_PCT ) / 100u, freqs[ idx ], busFreq );
        TEST_ASSERT_LESS_OR_EQUAL_UINT32( freqs[ idx ] + ( ( freqs[ idx ] * UT_I2C_FREQ_TOL_PCT ) / 100u ), busFreq );
    }
}


/**
 * \brief   I2c_Set_BusFreq() controls Fast-mode Plus driver.
 *
 * \details Sets 1 MHz, then 400 kHz.
 *
 * \par Expected results
 * - 1 MHz: CR1.FMP = 1. 400 kHz: CR1.FMP = 0.
 */
void Ut_I2c_Set_BusFreq_FastModePlus_FmpBitControlled( void )
{
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_FMP, UT_I2C_REG->CR1 & I2C_CR1_FMP );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_FMP );
}


/**
 * \brief   Standard-mode timing respects I2C specification limits.
 *
 * \details Sets 100 kHz and evaluates TIMINGR PRESC, SCLL and SCLH with 64 MHz clock.
 *
 * \par Expected results
 * - tLOW >= 4.7 us and tHIGH >= 4.0 us (50 ns tolerance).
 */
void Ut_I2c_Set_BusFreq_TimingFieldsMeetStandardModeLimits( void )
{
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    const uint32_t timing  = UT_I2C_REG->TIMINGR;
    const uint32_t presc   = ( ( timing & I2C_TIMINGR_PRESC ) >> I2C_TIMINGR_PRESC_Pos ) + 1u;
    const uint32_t tPresNs = ( presc * 1000000000u ) / UT_I2C_CLK_HZ;
    const uint32_t sclL    = ( ( timing & I2C_TIMINGR_SCLL ) >> I2C_TIMINGR_SCLL_Pos ) + 1u;
    const uint32_t sclH    = ( ( timing & I2C_TIMINGR_SCLH ) >> I2C_TIMINGR_SCLH_Pos ) + 1u;

    /* Standard-mode: tLOW >= 4.7 us, tHIGH >= 4.0 us */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 4700u, sclL * tPresNs + 50u );
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 4000u, sclH * tPresNs + 50u );
}


/**
 * \brief   I2c_Set_BusFreq() rejects invalid frequency and enabled peripheral.
 *
 * \details Sets 0 Hz, maximum + 1, invalid peripheral, and 100 kHz with PE = 1.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, TIMINGR is not written.
 */
void Ut_I2c_Set_BusFreq_InvalidOrEnabled_ReturnsErrorWithoutWrite( void )
{
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 0u ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, I2C_BUS_FREQ_MAX_HZ + 1u ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( I2C_PERIPH_CNT, 100000u ) );

    UT_I2C_REG->CR1 = I2C_CR1_PE;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->TIMINGR );
}


/**
 * \brief   I2c_Set_BusFreq() rejects frequency not reachable from kernel clock.
 *
 * \details Kernel clock 4 MHz, bus frequency 1 MHz.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR is returned.
 */
void Ut_I2c_Set_BusFreq_ClockTooSlowForFastPlus_ReturnsError( void )
{
    static rcc_FreqHz_t csiClk = 4000000u;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &csiClk );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
}


/**
 * \brief   I2c_Get_BusFreq() rejects invalid arguments.
 *
 * \details Calls I2c_Get_BusFreq() with invalid peripheral and NULL pointer.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in both cases.
 */
void Ut_I2c_Get_BusFreq_InvalidArgs_ReturnsError( void )
{
    i2c_FreqHz_t busFreq = 0u;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusFreq( I2C_PERIPH_CNT, &busFreq ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_BusFreq( UT_I2C_PERIPH, NULL ) );
}

/* =============================== FILTERS ================================== */

/**
 * \brief   I2c_Set_AnalogFilter() controls CR1.ANFOFF.
 *
 * \details Disables, then enables analog filter and reads it back each time.
 *
 * \par Expected results
 * - Disabled: ANFOFF = 1, read DISABLED. Enabled: ANFOFF = 0, read ENABLED.
 */
void Ut_I2c_Set_AnalogFilter_DisabledEnabled_RegisterAndReadBack( void )
{
    i2c_AnalogFilter_t analogFilter = I2C_ANALOG_FILTER_ENABLED;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_DISABLED ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_ANFOFF, UT_I2C_REG->CR1 & I2C_CR1_ANFOFF );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AnalogFilter( UT_I2C_PERIPH, &analogFilter ) );
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_DISABLED, analogFilter );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_ENABLED ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & I2C_CR1_ANFOFF );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AnalogFilter( UT_I2C_PERIPH, &analogFilter ) );
    TEST_ASSERT_EQUAL( I2C_ANALOG_FILTER_ENABLED, analogFilter );
}


/**
 * \brief   I2c_Set_DigitalFilter() writes every filter length.
 *
 * \details Sets every digital filter value (off, 1 - 15).
 *
 * \par Expected results
 * - CR1.DNF equals the value, read back value equals the set one.
 */
void Ut_I2c_Set_DigitalFilter_AllLengths_RegisterAndReadBack( void )
{
    i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_OFF;

    for( i2c_DigitalFilter_t dnf = I2C_DIGITAL_FILTER_OFF; I2C_DIGITAL_FILTER_CNT > dnf; dnf++ )
    {
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DigitalFilter( UT_I2C_PERIPH, dnf ) );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)dnf << I2C_CR1_DNF_Pos, UT_I2C_REG->CR1 & I2C_CR1_DNF );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DigitalFilter( UT_I2C_PERIPH, &digitalFilter ) );
        TEST_ASSERT_EQUAL( dnf, digitalFilter );
    }
}


/**
 * \brief   Filter functions reject invalid arguments and enabled peripheral.
 *
 * \details Calls filter setters / getters with invalid values, invalid peripheral
 *          and NULL pointer, then setters with PE = 1.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, CR1 keeps only PE.
 */
void Ut_I2c_Set_Filters_InvalidOrEnabled_ReturnsErrorWithoutWrite( void )
{
    i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_ENABLED;
    i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_OFF;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DigitalFilter( UT_I2C_PERIPH, I2C_DIGITAL_FILTER_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AnalogFilter( I2C_PERIPH_CNT, I2C_ANALOG_FILTER_DISABLED ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DigitalFilter( I2C_PERIPH_CNT, I2C_DIGITAL_FILTER_1 ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_AnalogFilter( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DigitalFilter( I2C_PERIPH_CNT, &digitalFilter ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_AnalogFilter( I2C_PERIPH_CNT, &analogFilter ) );

    /* Filters can be changed only when the peripheral is disabled */
    UT_I2C_REG->CR1 = I2C_CR1_PE;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AnalogFilter( UT_I2C_PERIPH, I2C_ANALOG_FILTER_DISABLED ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DigitalFilter( UT_I2C_PERIPH, I2C_DIGITAL_FILTER_1 ) );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 );
}

/* ============================ ADDRESS MODE ================================ */

/**
 * \brief   I2c_Set_AddrMode() controls CR2.ADD10.
 *
 * \details Sets 10-bit, then 7-bit mode, then invalid mode and NULL read pointer.
 *
 * \par Expected results
 * - 10-bit: ADD10 = 1, read back 10-bit. 7-bit: ADD10 = 0, read back 7-bit.
 * - Invalid mode and NULL pointer: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Set_AddrMode_7And10Bit_RegisterAndReadBack( void )
{
    i2c_AddrMode_t addrMode = I2C_ADDR_MODE_7BIT;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_ADD10, UT_I2C_REG->CR2 & I2C_CR2_ADD10 );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AddrMode( UT_I2C_PERIPH, &addrMode ) );
    TEST_ASSERT_EQUAL( I2C_ADDR_MODE_10BIT, addrMode );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_7BIT ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_ADD10 );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AddrMode( UT_I2C_PERIPH, &addrMode ) );
    TEST_ASSERT_EQUAL( I2C_ADDR_MODE_7BIT, addrMode );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_CNT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_AddrMode( UT_I2C_PERIPH, NULL ) );
}

/* ============================== INTERRUPTS ================================ */

/**
 * \brief   I2c_Set_IrqPriority() sets priority of event and error IRQ.
 *
 * \details
 * 1. Both NVIC calls succeed.
 * 2. Error IRQ priority fails.
 * 3. Invalid peripheral.
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
 *          transfer mode.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, NVIC / GPDMA not called.
 */
void Ut_I2c_Set_DataConfig_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( I2C_PERIPH_CNT, &dataConfig ) );

    dataConfig.XferMode = I2C_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
}


/**
 * \brief   I2c_Set_DataConfig() rejects invalid DMA configuration.
 *
 * \details DMA mode with the same RX and TX channel, invalid TX channel and
 *          invalid RX DMA peripheral.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, GPDMA not called.
 */
void Ut_I2c_Set_DataConfig_DmaInvalidChannels_ReturnsErrorWithoutAccess( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    dataConfig.RxDmaChannelId = dataConfig.TxDmaChannelId;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDmaChannelId = I2C_DMA_CHANNEL_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDmaPeriphId = I2C_DMA_PERIPH_CNT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
}


/**
 * \brief   I2c_Get_DataConfig() without data handling returns error.
 *
 * \details Reads data configuration of not initialized I2C1 and with NULL pointer.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in both cases.
 */
void Ut_I2c_Get_DataConfig_NotInitialized_ReturnsError( void )
{
    i2c_DataConfig_t readConfig;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, NULL ) );
}

/* ============================ TRANSFER START ============================== */

/**
 * \brief   I2c_Set_XferStart() rejects invalid transfer request.
 *
 * \details Polling mode is initialized, request is NULL, peripheral is invalid,
 *          7-bit address is too high, TX data is NULL with TX size 1, RX size 1
 *          with NULL RX buffer.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, START is not requested.
 */
void Ut_I2c_Set_XferStart_InvalidRequest_ReturnsErrorWithoutStart( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    i2c_XferRequest_t      request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, NULL ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( I2C_PERIPH_CNT, &request ) );

    request.SlaveAddr = I2C_SLAVE_ADDR_7BIT_MAX + 1u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    request.SlaveAddr = UT_I2C_SLAVE_ADDR;
    request.TxData    = NULL;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    request.TxData = utI2c_TxBuf;
    request.RxSize = 1u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_START );
}


/**
 * \brief   I2c_Set_XferStart() is rejected when the module is not ready.
 *
 * \details Starts 1 byte write:
 * 1. without data handling,
 * 2. with transfer mode NONE,
 * 3. polling mode with bus busy,
 * 4. polling mode with disabled peripheral.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, START is not requested.
 */
void Ut_I2c_Set_XferStart_NotReady_ReturnsErrorWithoutStart( void )
{
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_NONE );

    /* Data handling not initialized */
    Ut_I2c_Init( NULL );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    /* Transfer mode NONE */
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    /* Bus busy */
    dataConfig.XferMode = I2C_XFER_MODE_POLL;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
    UT_I2C_REG->ISR = I2C_ISR_BUSY;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    /* Peripheral disabled */
    UT_I2C_REG->ISR = 0u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_START );
}


/**
 * \brief   Running transfer blocks new transfer and configuration changes.
 *
 * \details Starts 2 byte write in polling mode, reads transfer state, then starts
 *          another transfer and changes address mode, data configuration and
 *          peripheral state.
 *
 * \par Expected results
 * - First start I2C_REQUEST_OK, transfer state ACTIVE.
 * - Second start, address mode, data configuration and disable: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Set_XferStart_RunningTransfer_SecondRequestRejected( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_INACTIVE;

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_ACTIVE, xferState );

    Ut_I2c_Set_StartSent();
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    /* Configuration changes are rejected during the transfer */
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_PeriphInactive( UT_I2C_PERIPH ) );
}


/**
 * \brief   I2c_Set_XferStart() of write programs CR2 and clears flags.
 *
 * \details Starts 3 byte write to slave 0x50 in polling mode.
 *
 * \par Expected results
 * - CR2: SADD = 0x50 << 1, NBYTES = 3, AUTOEND, write direction, START.
 * - ICR: NACK, STOP, BERR, ARLO and OVR flags cleared.
 */
void Ut_I2c_Set_XferStart_Write_Cr2ProgrammedWithStart( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 3u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 3u, I2C_CR2_AUTOEND, 0u ), Ut_I2c_Get_Cr2Xfer() );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_START, UT_I2C_REG->CR2 & I2C_CR2_START );
    TEST_ASSERT_EQUAL_HEX32( I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF,
                             UT_I2C_REG->ICR & ( I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF ) );
}


/**
 * \brief   I2c_Set_XferStart() of read programs read direction.
 *
 * \details Starts 2 byte read from slave 0x50.
 *
 * \par Expected results
 * - CR2: SADD = 0x50 << 1, NBYTES = 2, AUTOEND, RD_WRN = 1.
 */
void Ut_I2c_Set_XferStart_Read_Cr2ReadDirection( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 2u };

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 2u, I2C_CR2_AUTOEND, I2C_CR2_RD_WRN ), Ut_I2c_Get_Cr2Xfer() );
}


/**
 * \brief   Transfer without data sends only address (slave probe).
 *
 * \details Starts request without TX and RX data.
 *
 * \par Expected results
 * - CR2: SADD = 0x50 << 1, NBYTES = 0, AUTOEND, write direction.
 */
void Ut_I2c_Set_XferStart_AddressOnly_ZeroBytesAutoEnd( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 0u, I2C_CR2_AUTOEND, 0u ), Ut_I2c_Get_Cr2Xfer() );
}


/**
 * \brief   10-bit slave address is written to CR2 unshifted.
 *
 * \details Sets 10-bit address mode and starts 1 byte write to slave 0x2A5.
 *
 * \par Expected results
 * - CR2.SADD = 0x2A5, CR2.ADD10 = 1.
 */
void Ut_I2c_Set_XferStart_10BitAddress_NotShifted( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = 0x2A5u, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL_HEX32( 0x2A5u, UT_I2C_REG->CR2 & I2C_CR2_SADD );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_ADD10, UT_I2C_REG->CR2 & I2C_CR2_ADD10 );
}

/* ----------------------------- Polling mode ------------------------------- */

/**
 * \brief   Polling write - I2c_Task() writes bytes and reports completion.
 *
 * \details Starts 2 byte write in polling mode, then for each step presets ISR flag
 *          and calls I2c_Task(): TXIS, TXIS, STOPF. Finally STOPF is processed
 *          again without running transfer.
 *
 * \par Expected results
 * - TXDR = 0x10, then 0x11 (TX buffer data).
 * - Complete callback only after STOPF, no error callback.
 * - CR2 transfer fields cleared, transfer state INACTIVE, error NONE.
 * - Task without running transfer does not call the callback again.
 */
void Ut_I2c_Task_PollWrite_BytesWrittenAndCompleteCallback( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t       xferError  = I2C_XFER_ERROR_CNT;

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    UT_I2C_REG->ISR = I2C_ISR_TXIS;
    I2c_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x10u, UT_I2C_REG->TXDR );

    UT_I2C_REG->ISR = I2C_ISR_TXIS;
    I2c_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_I2C_REG->TXDR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );

    UT_I2C_REG->ISR = I2C_ISR_STOPF;
    I2c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & ( I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_AUTOEND ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( UT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NONE, xferError );

    /* Task without running transfer does nothing */
    UT_I2C_REG->ISR = I2C_ISR_STOPF;
    I2c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
}


/**
 * \brief   Polling write + read uses repeated START.
 *
 * \details Starts 1 byte write + 2 byte read:
 * 1. write phase is programmed without AUTOEND,
 * 2. TXIS - byte written,
 * 3. TC - read phase is started,
 * 4. RXNE (0xA1), RXNE + STOPF (0xA2).
 *
 * \par Expected results
 * 1. CR2: NBYTES = 1, software end, write direction.
 * 2. TXDR = 0x10.
 * 3. CR2: NBYTES = 2, AUTOEND, read direction, START (repeated start).
 * 4. RX buffer = { 0xA1, 0xA2 }, 3rd byte untouched, complete callback once.
 */
void Ut_I2c_Task_PollWriteRead_RepeatedStartAndDataRead( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = utI2c_RxBuf, .RxSize = 2u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    /* Write phase ends without STOP (software end) */
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 1u, 0u, 0u ), Ut_I2c_Get_Cr2Xfer() );
    Ut_I2c_Set_StartSent();

    UT_I2C_REG->ISR = I2C_ISR_TXIS;
    I2c_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x10u, UT_I2C_REG->TXDR );

    /* TC: repeated START with read direction */
    UT_I2C_REG->ISR = I2C_ISR_TC;
    I2c_Task();
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 2u, I2C_CR2_AUTOEND, I2C_CR2_RD_WRN ), Ut_I2c_Get_Cr2Xfer() );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_START, UT_I2C_REG->CR2 & I2C_CR2_START );
    Ut_I2c_Set_StartSent();

    UT_I2C_REG->RXDR = 0xA1u;
    UT_I2C_REG->ISR  = I2C_ISR_RXNE;
    I2c_Task();
    UT_I2C_REG->RXDR = 0xA2u;
    UT_I2C_REG->ISR  = I2C_ISR_RXNE | I2C_ISR_STOPF;
    I2c_Task();

    TEST_ASSERT_EQUAL_HEX8( 0xA1u, utI2c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xA2u, utI2c_RxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x00u, utI2c_RxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
}


/**
 * \brief   Write longer than 255 bytes is split by RELOAD.
 *
 * \details Starts 300 byte write, then processes TCR flag.
 *
 * \par Expected results
 * - First chunk: NBYTES = 255 with RELOAD.
 * - After TCR: NBYTES = 45 with AUTOEND, no new START.
 */
void Ut_I2c_Task_PollLongWrite_ReloadChunks( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = UT_I2C_LONG_SIZE, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    /* First chunk: 255 bytes with reload */
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 255u, I2C_CR2_RELOAD, 0u ), Ut_I2c_Get_Cr2Xfer() );
    Ut_I2c_Set_StartSent();

    /* TCR: remaining 45 bytes without START, automatic end */
    UT_I2C_REG->ISR = I2C_ISR_TCR;
    I2c_Task();

    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, UT_I2C_LONG_SIZE - 255u, I2C_CR2_AUTOEND, 0u ), Ut_I2c_Get_Cr2Xfer() );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_START );
}


/**
 * \brief   NACK is reported after STOP condition.
 *
 * \details Starts address-only transfer, processes NACKF, then STOPF. Then a new
 *          request is started.
 *
 * \par Expected results
 * - No callback after NACKF.
 * - After STOPF: error callback with I2C_XFER_ERROR_NACK, no complete callback,
 *   I2c_Get_XferError() returns NACK.
 * - New request clears the error (NONE).
 */
void Ut_I2c_Task_PollNack_ErrorCallbackAfterStop( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = NULL, .RxSize = 0u };
    i2c_XferErrorId_t       xferError  = I2C_XFER_ERROR_NONE;

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    /* NACK: error reported after STOP condition */
    UT_I2C_REG->ISR = I2C_ISR_NACKF;
    I2c_Task();
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );

    UT_I2C_REG->ISR = I2C_ISR_STOPF;
    I2c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, utI2c_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( UT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, xferError );

    /* Next request clears the error */
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( UT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NONE, xferError );
}


/**
 * \brief   Bus error and arbitration lost are reported.
 *
 * \details Starts write and processes BERR; starts it again and processes ARLO.
 *
 * \par Expected results
 * - 1st error callback with I2C_XFER_ERROR_BUS.
 * - 2nd error callback with I2C_XFER_ERROR_ARBITRATION_LOST.
 */
void Ut_I2c_Task_PollBusErrorAndArbitrationLost_ErrorCallback( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    UT_I2C_REG->ISR = I2C_ISR_BERR;
    I2c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_BUS, utI2c_LastError );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    UT_I2C_REG->ISR = I2C_ISR_ARLO;
    I2c_Task();
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_ARBITRATION_LOST, utI2c_LastError );
}


/**
 * \brief   I2c_Set_XferStop() aborts running transfer.
 *
 * \details Starts 2 byte write, stops it, then stops without running transfer and
 *          for invalid peripheral.
 *
 * \par Expected results
 * - Transfer state INACTIVE, peripheral enabled again (PE = 1), CR2 transfer
 *   fields cleared, no callback.
 * - Stop without transfer: I2C_REQUEST_OK. Invalid peripheral: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Set_XferStop_RunningTransfer_AbortedPeripheralReEnabled( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_ACTIVE;

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStop( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
    TEST_ASSERT_EQUAL_HEX32( 0u, Ut_I2c_Get_Cr2Xfer() & ~I2C_CR2_ADD10 );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );

    /* Stop without running transfer */
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStop( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStop( I2C_PERIPH_CNT ) );
}

/* ---------------------------- Interrupt mode ------------------------------ */

/**
 * \brief   ISR mode write - interrupts enabled by start, data written by ISR.
 *
 * \details Initializes ISR mode, starts 2 byte write and calls captured ISR with
 *          TXIS three times, then STOPF.
 *
 * \par Expected results
 * - Interrupt enables (TX, RX, NACK, STOP, TC, ERR) are 0 after initialization
 *   and set by the start.
 * - TXDR = 0x10, 0x11; third TXIS does not write beyond the request.
 * - Complete callback once, interrupts disabled at the end.
 */
void Ut_I2c_Isr_Write_InterruptsEnabledBytesWrittenAndComplete( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    const uint32_t          itMask     = I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE;

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & itMask );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL_HEX32( itMask, UT_I2C_REG->CR1 & itMask );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_ISR_TXIS );
    TEST_ASSERT_EQUAL_HEX32( 0x10u, UT_I2C_REG->TXDR );
    Ut_I2c_Call_Isr( I2C_ISR_TXIS );
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_I2C_REG->TXDR );

    /* No data are written beyond the request */
    Ut_I2c_Call_Isr( I2C_ISR_TXIS );
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_I2C_REG->TXDR );

    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & itMask );
}


/**
 * \brief   ISR mode read - received bytes stored by ISR.
 *
 * \details Starts 3 byte read, calls ISR with RXNE for RXDR 0xC0 - 0xC2, then STOPF.
 *
 * \par Expected results
 * - RX buffer = { 0xC0, 0xC1, 0xC2 }, complete callback once.
 */
void Ut_I2c_Isr_Read_DataStoredAndComplete( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    for( uint32_t byteIdx = 0u; 3u > byteIdx; byteIdx++ )
    {
        UT_I2C_REG->RXDR = 0xC0u + byteIdx;
        Ut_I2c_Call_Isr( I2C_ISR_RXNE );
    }

    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_HEX8( 0xC0u, utI2c_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xC1u, utI2c_RxBuf[ 1u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xC2u, utI2c_RxBuf[ 2u ] );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
}


/**
 * \brief   NACK and STOP in one interrupt report NACK error.
 *
 * \details Starts 1 byte write and calls ISR with NACKF and STOPF together.
 *
 * \par Expected results
 * - Error callback once with I2C_XFER_ERROR_NACK, no complete callback.
 */
void Ut_I2c_Isr_NackAndStopTogether_NackErrorReported( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_ISR_NACKF | I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, utI2c_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
}


/**
 * \brief   ISR without running transfer ignores flags.
 *
 * \details Initializes ISR mode and calls ISR with TXIS, STOPF and BERR.
 *
 * \par Expected results
 * - TXDR not written, no complete and no error callback.
 */
void Ut_I2c_Isr_NoTransfer_FlagsIgnored( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );

    Ut_I2c_Init( &dataConfig );

    Ut_I2c_Call_Isr( I2C_ISR_TXIS | I2C_ISR_STOPF | I2C_ISR_BERR );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->TXDR );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
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
 * \brief RCC clock source stub - I2C1 kernel clock source is the PCLK1.
 */
static rcc_RequestState_t Ut_I2c_RccGetClkSrcStub( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_I2C_RCC_PCLK, periphId );

    *periphClkSrc = UT_I2C_RCC_PCLK;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief RCC kernel clock stub - returns \ref UT_I2C_CLK_HZ.
 */
static rcc_RequestState_t Ut_I2c_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_I2C_RCC_PCLK, periphId );

    *periphClk = UT_I2C_CLK_HZ;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief GPIO initialization stub - stores the pin configuration.
 */
static gpio_RequestState_t Ut_I2c_GpioInitStub( gpio_Config_t *gpioConfig, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( gpioConfig );

    utI2c_GpioConfig = *gpioConfig;

    return ( GPIO_REQUEST_OK );
}


/**
 * \brief Ignores all calls of RCC / NVIC / GPIO / GPDMA functions used by peripheral
 *        configuration and data handling.
 */
static void Ut_I2c_Ignore_PeriphMocks( void )
{
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Gpio_Init_IgnoreAndReturn( GPIO_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
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
    MockGpdma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockGpdma_Port_Init();
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
 * \brief Returns data handling configuration with test callbacks.
 *
 * \param xferMode [in]: Data transfer mode
 */
static i2c_DataConfig_t Ut_I2c_Get_DataConfig( i2c_XferMode_t xferMode )
{
    i2c_DataConfig_t dataConfig;

    dataConfig.XferMode             = xferMode;
    dataConfig.TxDmaPeriphId        = I2C_DMA_PERIPH_1;
    dataConfig.TxDmaChannelId       = I2C_DMA_CHANNEL_0;
    dataConfig.TxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    dataConfig.RxDmaPeriphId        = I2C_DMA_PERIPH_1;
    dataConfig.RxDmaChannelId       = I2C_DMA_CHANNEL_1;
    dataConfig.RxDmaPriority        = I2C_DMA_PRIORITY_HIGH;
    dataConfig.IrqPriority          = UT_I2C_PRIO;
    dataConfig.XferCompleteCallback = Ut_I2c_XferCompleteCallback;
    dataConfig.ErrorCallback        = Ut_I2c_ErrorCallback;

    return ( dataConfig );
}


/**
 * \brief Initializes I2C1 (100 kHz) with ignored RCC / NVIC calls.
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
 * \brief Calls the captured I2C ISR with given ISR register flags.
 *
 * \param isrFlags [in]: Value of the ISR register
 */
static void Ut_I2c_Call_Isr( uint32_t isrFlags )
{
    TEST_ASSERT_NOT_NULL_MESSAGE( utI2c_Isr, "I2C ISR not registered" );

    UT_I2C_REG->ISR = isrFlags;
    utI2c_Isr();
}


/**
 * \brief Emulates hardware clearing of CR2.START after the address phase.
 */
static void Ut_I2c_Set_StartSent( void )
{
    UT_I2C_REG->CR2 &= ~I2C_CR2_START;
}


/**
 * \brief Returns transfer fields of CR2 (SADD, ADD10, NBYTES, RELOAD, AUTOEND, RD_WRN).
 */
static uint32_t Ut_I2c_Get_Cr2Xfer( void )
{
    return ( UT_I2C_REG->CR2 & ( I2C_CR2_SADD | I2C_CR2_ADD10 | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN ) );
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
