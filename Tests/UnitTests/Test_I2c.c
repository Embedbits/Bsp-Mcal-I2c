/**
 * \author Mr.Nobody
 * \file Test_I2c.c
 * \ingroup I2c
 * \brief Unit tests of Inter-Integrated Circuit (I2C) module (STM32H7 family).
 *
 * I2c.c, I2c_Isr.c, I2c_Poll.c and I2c_Dma.c are compiled unchanged with real
 * LL drivers. I2C and SYSCFG registers are emulated by RegMem, RCC, NVIC, GPIO
 * and DMA modules are mocked by CMock. I2C ISR registered in NVIC is captured by
 * stub and called directly to test interrupt data handling. DMA configurations
 * passed to Dma_Init() are captured by stub, their error handlers are called
 * directly.
 *
 * \note Emulated registers are plain memory:
 *       - ISR flags are not cleared by ICR writes and data register accesses,
 *         tests preset the flags handled by the module before each step,
 *       - CR2.START is not cleared by hardware after the address phase, tests
 *         clear it before the next request (\ref Ut_I2c_Set_StartSent),
 *       - ISR.TXE is set by the TX flush of the module (write of TXE to ISR).
 *       - CR2.START released while PE = 0 is emulated by HW model thread
 *         (\ref Ut_I2c_HwModel, tests registered with RUN_SERIAL).
 *
 * \note Data handling context of the module is static - setUp() releases the
 *       data handling of the previous test (I2c_Deinit with ignored mocks)
 *       and re-initializes the mocks.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "CmsisHost.h"                      /* Core intrinsics emulation      */
#include "I2c_Port.h"                       /* Module under test              */
#include "I2c.h"                            /* Module internal services       */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "MockGpdma_Port.h"                 /* GPDMA module mock (STM32H7R / H7S) */
#include "Stm32_i2c.h"                      /* I2C registers definition       */
#include "Stm32_system.h"                   /* SYSCFG registers definition    */
#include <string.h>                         /* memset                         */
/* ============================= TYPEDEFS =================================== */

#if defined(STM32H7RS)
/** \brief Record of the calls of one GPDMA channel (Gpdma_* stubs) */
typedef struct
{
    uint32_t            ActiveCnt;      /**< Gpdma_Set_ChannelActive() calls          */
    uint32_t            InactiveCnt;    /**< Gpdma_Set_ChannelInactive() calls        */
    uint32_t            IrqOnCnt;       /**< Gpdma_Set_InterruptActive() calls        */
    uint32_t            IrqOffCnt;      /**< Gpdma_Set_InterruptInactive() calls      */
    uint32_t            PrioCnt;        /**< Gpdma_Set_Priority() calls               */
    gpdma_Priority_t    Prio;           /**< Last priority                            */
    gpdma_BlockSize_t   BlockSize;      /**< Last block size                          */
    gpdma_SrcAddr_t     SrcAddr;        /**< Last source address                      */
    gpdma_DstAddr_t     DstAddr;        /**< Last destination address                 */
}   utI2c_GpdmaChannel_t;
#endif /* STM32H7RS */

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_I2c_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetClkSrcStub     ( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccResetHsiStub      ( rcc_PeriphId_t periphId, int callCnt );
static gpio_RequestState_t  Ut_I2c_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
#if !defined(STM32H7RS)
static dma_RequestState_t   Ut_I2c_DmaInitStub          ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_I2c_DmaGetCountStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt );
static dma_RequestState_t   Ut_I2c_DmaIrqOffStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
#endif /* !STM32H7RS */
static void                 Ut_I2c_Ignore_PeriphMocks   ( void );
static void                 Ut_I2c_Release              ( void );
static void                 Ut_I2c_Reset_Mocks          ( void );
static i2c_Config_t         Ut_I2c_Get_Config           ( void );
static i2c_DataConfig_t     Ut_I2c_Get_DataConfig       ( i2c_XferMode_t xferMode );
static void                 Ut_I2c_Init                 ( const i2c_DataConfig_t * const dataConfig );
static void                 Ut_I2c_Call_Isr             ( uint32_t isrFlags );
static void                 Ut_I2c_Set_StartSent        ( void );
#if defined(STM32H7RS)
static void                 Ut_I2c_Setup_GpdmaMocks       ( void );
static i2c_DataConfig_t     Ut_I2c_Get_GpdmaDataConfig    ( void );
static uint32_t             Ut_I2c_Find_GpdmaInit         ( i2c_DmaChannelId_t channelId );
static utI2c_GpdmaChannel_t * Ut_I2c_Get_GpdmaChannel       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel );
static gpdma_RequestState_t Ut_I2c_GpdmaInitStub          ( gpdma_ConfigStruct_t * const configStruct, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaActiveStub        ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaInactiveStub      ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaIrqOnStub         ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaIrqOffStub        ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaPrioStub          ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_Priority_t channelPrio, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaBlockSizeStub     ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t blockSize, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaSrcAddrStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_SrcAddr_t sourceAddr, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaDstAddrStub       ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DstAddr_t destAddr, int callCnt );
static gpdma_RequestState_t Ut_I2c_GpdmaRemainingStub     ( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t * const blockSize, int callCnt );
#endif /* STM32H7RS */
static uint32_t             Ut_I2c_Get_Cr2Xfer          ( void );

static void                 Ut_I2c_HwModel              ( void );
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

#if defined(STM32H7RS)
/** Fast-mode Plus bit of the test peripheral (I2C_CR1.FMP on STM32H7R / H7S) */
#define UT_I2C_FMP_REG                      ( UT_I2C_REG->CR1 )
#define UT_I2C_FMP_BIT                      ( I2C_CR1_FMP )
#else
/** SYSCFG_PMCR Fast-mode Plus bit of the test peripheral */
#define UT_I2C_FMP_REG                      ( SYSCFG->PMCR )
#define UT_I2C_FMP_BIT                      ( SYSCFG_PMCR_I2C1_FMP )
#endif /* STM32H7RS */

/** DMA channels of the test data configurations (DMA1) */
#define UT_I2C_DMA_TX_CHANNEL               ( I2C_DMA_CHANNEL_1 )
#define UT_I2C_DMA_RX_CHANNEL               ( I2C_DMA_CHANNEL_2 )

/** Kernel clock (PCLK1) returned by RCC mock [Hz] */
#define UT_I2C_CLK_HZ                       ( 64000000u )

/** HSI kernel clock returned by RCC mock [Hz] */
#define UT_I2C_HSI_HZ                       ( 16000000u )

/** Interrupt priority of test configurations */
#define UT_I2C_PRIO                         ( 5u )

/** Slave address of test transfers (7-bit) */
#define UT_I2C_SLAVE_ADDR                   ( 0x50u )

/** Bus frequency tolerance of read back value [%] */
#define UT_I2C_FREQ_TOL_PCT                 ( 10u )

/** Size of long transfer (more than one NBYTES chunk) */
#define UT_I2C_LONG_SIZE                    ( 300u )

#if defined(STM32H7RS)
/** Count of GPDMA configurations stored by the Gpdma_Init() stub */
#define UT_I2C_GPDMA_CFG_CNT                  ( 4u )

/** Count of GPDMA channels recorded per GPDMA peripheral */
#define UT_I2C_GPDMA_CHANNELS                 ( 16u )

/** GPDMA errors reported to the user by the module */
#define UT_I2C_GPDMA_ERROR_MASK               ( GPDMA_ERROR_TRANSFER | GPDMA_ERROR_CONFIG_UPDATE | GPDMA_ERROR_CONFIG_ERROR | GPDMA_ERROR_TRIG_OVERRUN )

/** Interrupt enable bits of the transfer events (DMA moves the data - no TXIE / RXIE) */
#define UT_I2C_GPDMA_EVENT_IRQS               ( I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE )
#endif /* STM32H7RS */

/* ============================== MACROS ==================================== */

/** Expected CR2 transfer fields (SADD, NBYTES, AUTOEND / RELOAD, RD_WRN) */
#define UT_I2C_CR2( ADDR, NBYTES, END, RD )     ( ( ( (uint32_t)(ADDR) << 1u ) & I2C_CR2_SADD )                           | \
                                                  ( ( (uint32_t)(NBYTES) << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES )      | \
                                                  (END) | (RD) )

/** All I2C interrupt sources used by the module (CR1) */
#define UT_I2C_CR1_IT_ALL                       ( I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE )

/** I2C interrupt sources of transfer sequencing events (DMA mode, CR1) */
#define UT_I2C_CR1_IT_EVENTS                    ( I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE )

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

#if defined(STM32H7RS)
/** GPDMA configurations of Gpdma_Init() calls (structure and transfer configuration) */
static gpdma_ConfigStruct_t     utI2c_GpdmaConfig[ UT_I2C_GPDMA_CFG_CNT ];
static gpdma_TransferConfig_t   utI2c_GpdmaXferConfig[ UT_I2C_GPDMA_CFG_CNT ];
static uint32_t                 utI2c_GpdmaInitCnt;

/** Return values of the Gpdma_Init() and Gpdma_Set_ChannelActive() stubs */
static gpdma_RequestState_t     utI2c_GpdmaInitState;
static gpdma_RequestState_t     utI2c_GpdmaActiveState;

/** Remaining block size returned by the Gpdma_Get_BlockSize() stub */
static gpdma_BlockSize_t        utI2c_GpdmaRemaining;

/** Records of GPDMA channel calls */
static utI2c_GpdmaChannel_t       utI2c_GpdmaChannel[ GPDMA_PERIPH_CNT ][ UT_I2C_GPDMA_CHANNELS ];

/**
 * Selector of the GPDMA channels of the next DMA data configuration. The GPDMA channel
 * ownership of the module is static (a configured channel is reused, Gpdma_Init() is not
 * called again), so every DMA test configures channels different from the previous test.
 */
static uint32_t                 utI2c_GpdmaChannelSel;
#endif /* STM32H7RS */

/** Kernel clock source returned by Rcc_Get_PeriphClkSrc stub */
static rcc_PeriphId_t           utI2c_ClkSrc;

/** Frequencies returned by Rcc_Get_PeriphClk stub - PCLK1 (APB1) and HSI [Hz] */
static rcc_FreqHz_t             utI2c_PclkHz;
static rcc_FreqHz_t             utI2c_HsiHz;

/** DMA configurations of Dma_Init calls (TX, RX) and count of the calls */
#if !defined(STM32H7RS)
static dma_ConfigStruct_t       utI2c_DmaConfig[ 2u ];
#endif /* !STM32H7RS */
static uint32_t                 utI2c_DmaInitCnt;

/** Count of data not moved by DMA returned by Dma_Get_DataCount stub */
#if !defined(STM32H7RS)
static dma_DataCount_t          utI2c_DmaRemaining;
#endif /* !STM32H7RS */

/** Return value of Dma_Init() stub for the RX channel */
#if !defined(STM32H7RS)
static dma_RequestState_t       utI2c_DmaInitRxState;
#endif /* !STM32H7RS */

/** HW model observed disabled peripheral (PE = 0) */
static volatile uint32_t        utI2c_PeOffSeen;

/** Count of Dma_Set_InterruptInactive() calls and their return value */
static uint32_t                 utI2c_DmaIrqOffCnt;
#if !defined(STM32H7RS)
static dma_RequestState_t       utI2c_DmaIrqOffState;
#endif /* !STM32H7RS */

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    utI2c_ClkSrc         = UT_I2C_RCC_PCLK;
    utI2c_PclkHz         = UT_I2C_CLK_HZ;
    utI2c_HsiHz          = UT_I2C_HSI_HZ;
#if !defined(STM32H7RS)
    utI2c_DmaIrqOffState = DMA_REQUEST_OK;
    utI2c_DmaInitRxState = DMA_REQUEST_OK;
#endif /* !STM32H7RS */

    Ut_I2c_Release();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utI2c_Isr          = NULL;
    utI2c_CompleteCnt  = 0u;
    utI2c_ErrorCnt     = 0u;
    utI2c_LastError    = I2C_XFER_ERROR_CNT;
    utI2c_DmaInitCnt   = 0u;
#if !defined(STM32H7RS)
    utI2c_DmaRemaining = 0u;
#endif /* !STM32H7RS */
    utI2c_DmaIrqOffCnt = 0u;

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
 * - RCC: active kernel clock released, clock activated, reset pulse (active,
 *   inactive) for RCC_PERIPH_I2C1_PCLK1.
 * - I2C_REQUEST_OK, CR1.PE = 1, ANFOFF = 0, DNF = 0, 7-bit addressing.
 * - TIMINGR is calculated (not zero), Fast-mode Plus driver stays disabled.
 */
void Ut_I2c_Init_DefaultConfig_ClockTimingAndEnable( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & ( I2C_CR1_PE | I2C_CR1_ANFOFF | I2C_CR1_DNF ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_ADD10 );
    TEST_ASSERT_NOT_EQUAL( 0u, UT_I2C_REG->TIMINGR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_FMP_REG & UT_I2C_FMP_BIT );
}


/**
 * \brief   I2c_Init() activates the selected HSI kernel clock.
 *
 * \details Initializes I2C1 with HSI clock source (HSI 16 MHz, PCLK1 64 MHz).
 *
 * \par Expected results
 * - Active kernel clock (RCC_PERIPH_I2C1_PCLK1) is released first.
 * - Clock activation and reset are requested for RCC_PERIPH_I2C1_HSI.
 * - I2C_REQUEST_OK is returned.
 */
void Ut_I2c_Init_HsiClockSource_SelectedClockActivated( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.ClkSrc = I2C_CLK_SRC_HSI;
    utI2c_ClkSrc  = UT_I2C_RCC_HSI;

    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
}


/**
 * \brief   Re-initialization with another kernel clock source releases the active one.
 *
 * \details Initializes I2C1 with PCLK clock source, then initializes it again with
 *          HSI clock source without I2c_Deinit(). RCC reset of the peripheral is
 *          emulated by stub (I2C registers cleared).
 *
 * \note    Regression test of STM32H5 module bug AB#1098 - RCC changes the kernel
 *          clock multiplexer only of a released clock, the re-initialization must
 *          release the active source before the new one is activated.
 *
 * \par Expected results
 * - 2nd I2c_Init(): Rcc_Set_PeriphInactive( RCC_PERIPH_I2C1_PCLK1 ) before clock
 *   activation and reset of RCC_PERIPH_I2C1_HSI.
 * - I2C_REQUEST_OK, peripheral enabled (PE = 1).
 */
void Ut_I2c_Init_ReInitOtherClockSource_ActiveSourceReleased( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    Ut_I2c_Init( NULL );
    Ut_I2c_Reset_Mocks();

    config.ClkSrc = I2C_CLK_SRC_HSI;
    utI2c_ClkSrc  = UT_I2C_RCC_HSI;

    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_StubWithCallback( Ut_I2c_RccResetHsiStub );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_HSI, RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   I2c_Init() stops on clock activation error.
 *
 * \details 1. Rcc_Set_PeriphInactive() (release of the active source) returns error.
 *          2. Rcc_Set_PeriphActive() returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, peripheral is not enabled (PE = 0).
 * - 1. Clock is not activated (strict mock).
 */
void Ut_I2c_Init_ClockError_ReturnsErrorPeripheralNotEnabled( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );

    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
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
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
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
 * \details Initializes I2C1 with SCL PB8, SDA PB7 (available on every STM32H7) and
 *          pull-up. GPIO initialization is captured by stub.
 *
 * \par Expected results
 * - I2C_REQUEST_OK.
 * - Last configured pin is PB7: alternate mode, open-drain, pull-up, AF4.
 */
void Ut_I2c_Init_Pins_GpioOpenDrainAlternateWithPull( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.SclPin  = I2C_SCL_PIN_I2C1_PB8;
    config.SdaPin  = I2C_SDA_PIN_I2C1_PB7;
    config.PinPull = I2C_PIN_PULL_UP;

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Gpio_Init_StubWithCallback( Ut_I2c_GpioInitStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    /* Last configured pin: SDA PB7 */
    TEST_ASSERT_EQUAL( GPIO_PORT_B,               utI2c_GpioConfig.PortId );
    TEST_ASSERT_EQUAL( GPIO_PIN_ID_7,             utI2c_GpioConfig.PinId );
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
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
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
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
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


/**
 * \brief   I2c_Deinit() disables the Fast-mode Plus driver in SYSCFG.
 *
 * \details Initializes I2C1, sets SYSCFG_PMCR.I2C1_FMP (1 MHz configuration) and
 *          deinitializes the peripheral. SYSCFG is not reset with the I2C.
 *
 * \par Expected results
 * - SYSCFG clock is activated, then reset pulse and clock deactivation of
 *   RCC_PERIPH_I2C1_PCLK1.
 * - I2C_REQUEST_OK, SYSCFG_PMCR.I2C1_FMP = 0.
 *
 * \note    STM32H7R / H7S: I2C_CR1.FMP is cleared, no SYSCFG clock activation.
 */
void Ut_I2c_Deinit_FastModePlusActive_FmpDriverDisabled( void )
{
    Ut_I2c_Init( NULL );
    Ut_I2c_Reset_Mocks();
    UT_I2C_FMP_REG |= UT_I2C_FMP_BIT;

#if !defined(STM32H7RS)
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_OK );
#endif /* !STM32H7RS */
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_FMP_REG & UT_I2C_FMP_BIT );
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
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );

    for( uint32_t idx = 0u; ( sizeof( freqs ) / sizeof( freqs[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, freqs[ idx ] ) );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( UT_I2C_PERIPH, &busFreq ) );

        TEST_ASSERT_UINT32_WITHIN( ( freqs[ idx ] * UT_I2C_FREQ_TOL_PCT ) / 100u, freqs[ idx ], busFreq );
        TEST_ASSERT_LESS_OR_EQUAL_UINT32( freqs[ idx ] + ( ( freqs[ idx ] * UT_I2C_FREQ_TOL_PCT ) / 100u ), busFreq );
    }
}


/**
 * \brief   I2c_Set_BusFreq() controls Fast-mode Plus driver (SYSCFG_PMCR.I2C1_FMP).
 *
 * \details Sets 1 MHz twice, then 400 kHz twice. Rcc_Set_PeriphActive() is a strict
 *          expectation - SYSCFG clock is activated only when the bit has to change.
 *
 * \par Expected results
 * - 1st 1 MHz: SYSCFG clock activated, I2C1_FMP = 1. 2nd 1 MHz: no RCC call.
 * - 1st 400 kHz: SYSCFG clock activated, I2C1_FMP = 0. 2nd 400 kHz: no RCC call.
 * - Other SYSCFG_PMCR bits are not changed.
 *
 * \note    STM32H7R / H7S: I2C_CR1.FMP = 1 for 1 MHz, 0 for 400 kHz, no SYSCFG clock activation.
 */
void Ut_I2c_Set_BusFreq_FastModePlus_FmpBitControlled( void )
{
#if defined(STM32H7RS)
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_FMP_BIT, UT_I2C_FMP_REG & UT_I2C_FMP_BIT );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_FMP_BIT, UT_I2C_FMP_REG & UT_I2C_FMP_BIT );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_FMP_REG & UT_I2C_FMP_BIT );
#else
    SYSCFG->PMCR = SYSCFG_PMCR_I2C2_FMP;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_FMP_BIT | SYSCFG_PMCR_I2C2_FMP, SYSCFG->PMCR );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( SYSCFG_PMCR_I2C2_FMP, SYSCFG->PMCR );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
#endif /* STM32H7RS */
}


/**
 * \brief   I2c_Set_BusFreq() reports failed SYSCFG clock activation.
 *
 * \details Sets 1 MHz, Rcc_Set_PeriphActive( RCC_PERIPH_SYSCFG ) returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, SYSCFG_PMCR.I2C1_FMP stays 0.
 */
void Ut_I2c_Set_BusFreq_SyscfgClockError_ReturnsError( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: Fast-mode Plus by I2C_CR1.FMP (no SYSCFG clock)" );
#else
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_FMP_REG & UT_I2C_FMP_BIT );
#endif /* STM32H7RS */
}


/**
 * \brief   I2c_Get_BusFreq() uses Fast-mode Plus rise / fall times when FMP is set.
 *
 * \details Sets 1 MHz and reads it back; clears SYSCFG_PMCR.I2C1_FMP and reads the
 *          frequency again (Standard / Fast-mode timing characteristics).
 *
 * \par Expected results
 * - FMP set: read frequency within +-10 % of 1 MHz.
 * - FMP cleared: read frequency differs from the FMP estimate.
 */
void Ut_I2c_Get_BusFreq_FmpState_SpeedModeCharacteristicsUsed( void )
{
    i2c_FreqHz_t fmpFreq = 0u;
    i2c_FreqHz_t fmFreq  = 0u;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( UT_I2C_PERIPH, &fmpFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( 1000000u * UT_I2C_FREQ_TOL_PCT ) / 100u, 1000000u, fmpFreq );

    UT_I2C_FMP_REG &= ~UT_I2C_FMP_BIT;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( UT_I2C_PERIPH, &fmFreq ) );
    TEST_ASSERT_NOT_EQUAL( fmpFreq, fmFreq );
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
 * \details Kernel clock and PCLK1 4 MHz, bus frequency 1 MHz.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR is returned, TIMINGR is not written.
 */
void Ut_I2c_Set_BusFreq_ClockTooSlowForFastPlus_ReturnsError( void )
{
    utI2c_PclkHz = 4000000u;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->TIMINGR );
}


/**
 * \brief   I2c_Set_BusFreq() rejects kernel clock below the minimum of the speed mode.
 *
 * \details Kernel clock HSI (16 MHz, HSIDIV 4), PCLK1 64 MHz (ratio 4 is allowed):
 *          1 MHz, 400 kHz; kernel clock 3 MHz, PCLK1 64 MHz: 100 kHz.
 *
 * \note    Device errata of the I2C IP (STM32G4 ES0430 2.15.1, STM32H7 errata audit AB#844, "wrong data
 *          sampling when data setup time (tSU;DAT) is shorter than one I2C kernel clock
 *          period") - I2CCLK must be at least 4 / 10 / 20 MHz in Standard / Fast /
 *          Fast-mode Plus, a slower kernel clock is refused.
 *
 * \par Expected results
 * - 16 MHz, 1 MHz: I2C_REQUEST_ERROR, TIMINGR not written.
 * - 16 MHz, 400 kHz: I2C_REQUEST_OK.
 * - 3 MHz, 100 kHz: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Set_BusFreq_KernelClkBelowSpeedModeMinimum_ReturnsError( void )
{
    utI2c_ClkSrc = UT_I2C_RCC_HSI;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->TIMINGR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );

    utI2c_HsiHz = 3000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );
}


/**
 * \brief   I2c_Set_BusFreq() rejects APB1 / I2CCLK ratio between 1.5 and 3.
 *
 * \details Kernel clock HSI (16 MHz, HSIDIV 4), 100 kHz, PCLK1: 24 MHz (ratio 1.5),
 *          24.5 MHz, 32 MHz (ratio 2), 47.5 MHz, 48 MHz (ratio 3), 16 MHz (ratio 1).
 *
 * \note    Device errata of the I2C IP (STM32G4 ES0430 2.15.5, STM32H7 errata audit AB#844, "I2C
 *          transmission stalls after the first byte when the APB clock frequency /
 *          I2C kernel clock ratio is between 1.5 and 3") - the ratio is refused.
 *
 * \par Expected results
 * - Ratio 1, 1.5 and 3: I2C_REQUEST_OK.
 * - Ratio above 1.5 and below 3: I2C_REQUEST_ERROR.
 */
void Ut_I2c_Set_BusFreq_ApbKernelClkRatioWindow_ReturnsError( void )
{
    utI2c_ClkSrc = UT_I2C_RCC_HSI;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    utI2c_PclkHz = 24000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    utI2c_PclkHz = 24500000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    utI2c_PclkHz = 32000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    utI2c_PclkHz = 47500000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    utI2c_PclkHz = 48000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    utI2c_PclkHz = 16000000u;
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );
}


/**
 * \brief   I2c_Set_BusFreq() rejects unknown APB1 clock.
 *
 * \details Rcc_Get_PeriphClk() of the kernel clock succeeds, the APB1 clock read
 *          (errata ratio check) returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, TIMINGR is not written.
 */
void Ut_I2c_Set_BusFreq_ApbClockUnknown_ReturnsError( void )
{
    static rcc_FreqHz_t kernelClk = UT_I2C_CLK_HZ;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_ExpectAnyArgsAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &kernelClk );
    Rcc_Get_PeriphClk_ExpectAnyArgsAndReturn( RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 100000u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->TIMINGR );
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
 * - I2C_REQUEST_ERROR in all cases, NVIC / DMA not called.
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
 * - I2C_REQUEST_ERROR in all cases, DMA not called.
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


/**
 * \brief   DMA data configuration initializes the channels of both directions.
 *
 * \details Initializes I2C1 in DMA mode (TX DMA1 stream 1 low priority, RX DMA1
 *          channel 2 high priority).
 *
 * \par Expected results
 * - Dma_Init() twice: TX memory to peripheral with DMAMUX request I2C1_TX and
 *   peripheral address TXDR, RX peripheral to memory with request I2C1_RX and
 *   peripheral address RXDR, configured priorities; 8-bit, normal mode, static
 *   peripheral / incremented memory address; only transfer error callback.
 * - Transfer error interrupt and channel interrupt enabled for TX, then RX channel.
 * - CR1: TXDMAEN, RXDMAEN and I2C interrupt sources disabled.
 */
void Ut_I2c_Set_DataConfig_Dma_ChannelsInitialized( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    i2c_Config_t           config     = Ut_I2c_Get_Config();

    config.DataConfig = &dataConfig;

    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Dma_Get_DefaultConfig_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Init_StubWithCallback( Ut_I2c_DmaInitStub );
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, DMA_REQUEST_OK );
    Dma_Set_InterruptActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_2, DMA_REQUEST_OK );
    Dma_Set_InterruptActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_2, DMA_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );

    TEST_ASSERT_EQUAL( DMA_STREAM_1,             utI2c_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH,  utI2c_DmaConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW,          utI2c_DmaConfig[ 0u ].Priority );
    TEST_ASSERT_EQUAL( DMA_REQ_I2C1_TX,           utI2c_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL_HEX32( (dma_PeriphAddr_t)(uintptr_t)&UT_I2C_REG->TXDR, utI2c_DmaConfig[ 0u ].PeriphAddress );

    TEST_ASSERT_EQUAL( DMA_STREAM_2,             utI2c_DmaConfig[ 1u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY,  utI2c_DmaConfig[ 1u ].Direction );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH,         utI2c_DmaConfig[ 1u ].Priority );
    TEST_ASSERT_EQUAL( DMA_REQ_I2C1_RX,           utI2c_DmaConfig[ 1u ].PeripheralReqId );
    TEST_ASSERT_EQUAL_HEX32( (dma_PeriphAddr_t)(uintptr_t)&UT_I2C_REG->RXDR, utI2c_DmaConfig[ 1u ].PeriphAddress );

    for( uint32_t dirIdx = 0u; 2u > dirIdx; dirIdx++ )
    {
        TEST_ASSERT_EQUAL( DMA_PERIPH_1,              utI2c_DmaConfig[ dirIdx ].DmaPeriphId );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,  utI2c_DmaConfig[ dirIdx ].TransferMode );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,    utI2c_DmaConfig[ dirIdx ].PeriphTransferSize );
        TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,    utI2c_DmaConfig[ dirIdx ].MemoryTransferSize );
        TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC,    utI2c_DmaConfig[ dirIdx ].PeriphAddrIncrement );
        TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT, utI2c_DmaConfig[ dirIdx ].MemoryAddrIncrement );
        TEST_ASSERT_NULL( utI2c_DmaConfig[ dirIdx ].TransferCompleteCallback );
        TEST_ASSERT_NULL( utI2c_DmaConfig[ dirIdx ].HalfTransferCallback );
        TEST_ASSERT_NOT_NULL( utI2c_DmaConfig[ dirIdx ].TransferErrorCallback );
    }

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA channel initialization error is reported and the data handling is
 *          released.
 *
 * \details I2C1 without data handling, I2c_Set_DataConfig() in DMA mode, Dma_Init()
 *          of the RX channel returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, already initialized TX channel released
 *   (Dma_Set_InterruptInactive once), data handling is not initialized.
 */
void Ut_I2c_Set_DataConfig_DmaInitError_ReturnsErrorChannelsReleased( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    i2c_DataConfig_t       readConfig;

    Ut_I2c_Init( NULL );

    utI2c_DmaInitRxState = DMA_REQUEST_ERROR;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA channel kept after failed release is reused by the next initialization.
 *
 * \details I2C1 in DMA mode, I2c_Deinit() with Dma_Set_InterruptInactive() error,
 *          then I2c_Init() in DMA mode again.
 *
 * \par Expected results
 * - I2c_Deinit(): I2C_REQUEST_ERROR, release of both channels tried (2 calls).
 * - I2c_Init(): I2C_REQUEST_OK, Dma_Init() not called again, priority of both
 *   channels updated by Dma_Set_Priority().
 */
void Ut_I2c_Dma_ReleaseError_ChannelReusedByNextInit( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );

    utI2c_DmaIrqOffState = DMA_REQUEST_ERROR;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );

    Ut_I2c_Reset_Mocks();
    utI2c_DmaIrqOffState = DMA_REQUEST_OK;
    Dma_Set_Priority_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, DMA_PRIORITY_LOW, DMA_REQUEST_OK );
    Dma_Set_Priority_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_2, DMA_PRIORITY_HIGH, DMA_REQUEST_OK );

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );
#endif /* STM32H7RS */
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
 * \brief   NACK of the first 10-bit address byte - pending START is released by PE toggling.
 *
 * \details 10-bit addressing, 1 byte write to slave 0x2A5 in polling mode. CR2.START stays set
 *          (HW leaves it pending after NACK of the first address byte), NACKF then STOPF are
 *          processed. HW model clears CR2.START while PE = 0.
 *
 * \note    Device errata bug AB#1147 of STM32F7 (ES0334, same I2C IP on STM32H7 - errata audit AB#844, "10-bit master mode: new transfer cannot be launched
 *          if first part of the address is not acknowledged by the slave").
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_NACK.
 * - Peripheral was disabled (seen by HW model), CR2.START released, PE = 1 again.
 */
void Ut_I2c_Task_Poll10BitNack_PendingStartReleased( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = 0x2A5u, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    UT_I2C_REG->ISR = I2C_ISR_NACKF;
    I2c_Task();

    utI2c_PeOffSeen = 0u;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( Ut_I2c_HwModel ) );

    UT_I2C_REG->ISR = I2C_ISR_STOPF;
    I2c_Task();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, utI2c_LastError );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_PeOffSeen );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_START );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   NACK of the first 10-bit address byte - START not released by PE = 0.
 *
 * \details As \ref Ut_I2c_Task_Poll10BitNack_PendingStartReleased without HW model - CR2.START
 *          stays set also while PE = 0.
 *
 * \note    Device errata bug AB#1147 of STM32F7 (ES0334, same I2C IP on STM32H7 - errata audit AB#844, "10-bit master mode: new transfer cannot be launched
 *          if first part of the address is not acknowledged by the slave").
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_NACK, the peripheral is enabled again (PE = 1).
 */
void Ut_I2c_Task_Poll10BitNack_StartNotReleased_PeripheralEnabled( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = 0x2A5u, .TxData = utI2c_TxBuf, .TxSize = 1u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( UT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    UT_I2C_REG->ISR = I2C_ISR_NACKF;
    I2c_Task();
    UT_I2C_REG->ISR = I2C_ISR_STOPF;
    I2c_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, utI2c_LastError );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_START, UT_I2C_REG->CR2 & I2C_CR2_START );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & I2C_CR1_PE );
}


/**
 * \brief   NACK in 7-bit addressing mode does not toggle the peripheral.
 *
 * \details 7-bit addressing, address-only transfer, CR2.START left set, NACKF then STOPF
 *          are processed with active HW model.
 *
 * \note    Device errata bug AB#1147 - the workaround is limited to 10-bit addressing.
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_NACK, peripheral not disabled (HW model did not see
 *   PE = 0), CR2.START not touched.
 */
void Ut_I2c_Task_Poll7BitNack_PeripheralNotToggled( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    UT_I2C_REG->ISR = I2C_ISR_NACKF;
    I2c_Task();

    utI2c_PeOffSeen = 0u;
    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelActive( Ut_I2c_HwModel ) );

    UT_I2C_REG->ISR = I2C_ISR_STOPF;
    I2c_Task();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Set_ModelInactive() );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, utI2c_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_PeOffSeen );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_START, UT_I2C_REG->CR2 & I2C_CR2_START );
}


/**
 * \brief   Bus error is ignored, arbitration lost is reported.
 *
 * \details Starts 2 byte write and processes BERR, then TXIS, TXIS and STOPF;
 *          starts it again and processes ARLO.
 *
 * \note    Device errata of the I2C IP (STM32G4 ES0430 2.15.2, STM32H7 errata audit AB#844, "spurious
 *          bus error detection in master mode") - BERR flag is only cleared, the
 *          transfer continues normally and no error is reported.
 *
 * \par Expected results
 * - BERR: flag cleared (ICR.BERRCF), transfer ACTIVE, no callback; the transfer
 *   ends by complete callback.
 * - ARLO: error callback with I2C_XFER_ERROR_ARBITRATION_LOST.
 */
void Ut_I2c_Task_PollBusErrorIgnoredArbitrationLostReported( void )
{
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_POLL );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_INACTIVE;

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    UT_I2C_REG->ICR = 0u;
    UT_I2C_REG->ISR = I2C_ISR_BERR;
    I2c_Task();
    TEST_ASSERT_EQUAL_HEX32( I2C_ICR_BERRCF, UT_I2C_REG->ICR & I2C_ICR_BERRCF );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_ACTIVE, xferState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );

    UT_I2C_REG->ISR = I2C_ISR_TXIS;
    I2c_Task();
    UT_I2C_REG->ISR = I2C_ISR_TXIS;
    I2c_Task();
    UT_I2C_REG->ISR = I2C_ISR_STOPF;
    I2c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    UT_I2C_REG->ISR = I2C_ISR_ARLO;
    I2c_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
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

/* ============================== DMA MODE ================================== */

/**
 * \brief   DMA write + read: channels armed, TC starts the read phase, STOPF ends
 *          the transfer.
 *
 * \details DMA mode, 2 byte write + 3 byte read:
 * 1. start - TX and RX channel armed (disabled first, they stay enabled after a
 *    normal mode transfer),
 * 2. TC (write phase moved by DMA) - repeated START,
 * 3. STOPF with empty channels.
 *
 * \par Expected results
 * 1. Dma_Set_TransferInactive / MemoryAddr / DataCount / TransferActive for TX
 *    channel 1 (TX buffer, 2), then RX channel 2 (RX buffer, 3); CR1: TXDMAEN,
 *    RXDMAEN and sequencing interrupts (NACK, STOP, TC, ERR) only; CR2: NBYTES 2,
 *    software end, write direction.
 * 2. CR2: NBYTES 3, AUTOEND, read direction, START; no callback.
 * 3. Complete callback, DMA requests and interrupt sources disabled.
 */
void Ut_I2c_Dma_WriteRead_ChannelsArmedStopEndsTransfer( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();

    Dma_Set_TransferInactive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, (dma_MemoryAddr_t)(uintptr_t)utI2c_TxBuf, DMA_REQUEST_OK );
    Dma_Set_DataCount_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, 2u, DMA_REQUEST_OK );
    Dma_Set_TransferActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, DMA_REQUEST_OK );
    Dma_Set_TransferInactive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_2, DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_2, (dma_MemoryAddr_t)(uintptr_t)utI2c_RxBuf, DMA_REQUEST_OK );
    Dma_Set_DataCount_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_2, 3u, DMA_REQUEST_OK );
    Dma_Set_TransferActive_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_2, DMA_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_EVENTS,
                             UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 2u, 0u, 0u ), Ut_I2c_Get_Cr2Xfer() );
    Ut_I2c_Set_StartSent();

    Ut_I2c_Call_Isr( I2C_ISR_TC );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 3u, I2C_CR2_AUTOEND, I2C_CR2_RD_WRN ), Ut_I2c_Get_Cr2Xfer() );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR2_START, UT_I2C_REG->CR2 & I2C_CR2_START );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    Ut_I2c_Set_StartSent();

    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Get_DataCount_StubWithCallback( Ut_I2c_DmaGetCountStub );
    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA transfer without data (slave probe) arms no channel.
 *
 * \details DMA mode, address only request, then STOPF.
 *
 * \par Expected results
 * - Start: no DMA arming call (strict mock), DMA requests disabled, sequencing
 *   interrupts enabled; CR2: NBYTES 0, AUTOEND.
 * - STOPF: complete callback (no channel data count is checked).
 */
void Ut_I2c_Dma_AddressOnly_NoChannelArmed( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = NULL, .RxSize = 0u };

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR1_IT_EVENTS, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_CR2( UT_I2C_SLAVE_ADDR, 0u, I2C_CR2_AUTOEND, 0u ), Ut_I2c_Get_Cr2Xfer() );
    Ut_I2c_Set_StartSent();

    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
#endif /* STM32H7RS */
}


/**
 * \brief   STOP before DMA moved all bytes is reported as DMA transfer error.
 *
 * \details DMA mode, 2 byte write, STOPF while 1 byte remains in the TX channel.
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_DMA_TRANSFER, no complete callback,
 *   transfer INACTIVE.
 */
void Ut_I2c_Dma_StopWithDataRemaining_DmaErrorReported( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_ACTIVE;

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    utI2c_DmaRemaining = 1u;
    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_DMA_TRANSFER, utI2c_LastError );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA transfer error aborts the transfer.
 *
 * \details DMA mode, 2 byte write: start, transfer error handler of the TX channel
 *          (registered by Dma_Init) is called.
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_DMA_TRANSFER, transfer INACTIVE, peripheral
 *   enabled again (PE = 1), DMA requests disabled.
 */
void Ut_I2c_Dma_TransferError_TransferAborted( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_ACTIVE;

    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    TEST_ASSERT_NOT_NULL( utI2c_DmaConfig[ 0u ].TransferErrorCallback );
    utI2c_DmaConfig[ 0u ].TransferErrorCallback();

    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_DMA_TRANSFER, utI2c_LastError );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL_HEX32( I2C_CR1_PE, UT_I2C_REG->CR1 & ( I2C_CR1_PE | I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN ) );
#endif /* STM32H7RS */
}


/**
 * \brief   Failed DMA channel arming rejects the transfer start.
 *
 * \details DMA mode, 2 byte write, Dma_Set_MemoryAddr() returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, transfer INACTIVE, START not requested, DMA requests and
 *   interrupt sources disabled, no callback.
 */
void Ut_I2c_Dma_ChannelArmError_StartRejected( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_ACTIVE;

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_STREAM_1, (dma_MemoryAddr_t)(uintptr_t)utI2c_TxBuf, DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_START );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
#endif /* STM32H7RS */
}


/**
 * \brief   I2c_Deinit() releases both DMA channels of DMA mode.
 *
 * \details I2C1 in DMA mode (TX channel 1, RX channel 2), I2c_Deinit() twice.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, Dma_Set_InterruptInactive() called for both channels (2x),
 *   DMA requests and interrupt sources disabled.
 * - Second I2c_Deinit(): I2C_REQUEST_OK, channels not touched again.
 */
void Ut_I2c_Dma_Deinit_ChannelsReleased( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    Ut_I2c_Init( &dataConfig );
    UT_I2C_REG->CR1 |= I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL;

    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
#endif /* STM32H7RS */
}

/* ======================== INTERRUPT HANDLERS (DSB) ======================== */

/** ISR registered in NVIC for any I2C peripheral */
static nvic_IsrCallback_t utI2c_AnyIsr;

/** NVIC handler registration stub of any I2C peripheral - stores the handler */
static nvic_RequestState_t Ut_I2c_NvicAnyHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utI2c_AnyIsr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/** RCC clock source stub of any I2C peripheral - the kernel clock source is the PCLK1 */
static rcc_RequestState_t Ut_I2c_RccAnyClkSrcStub( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt )
{
    (void)callCnt;

    *periphClkSrc = periphId;

    return ( RCC_REQUEST_OK );
}


/** RCC clock stub of any I2C peripheral - returns \ref UT_I2C_CLK_HZ */
static rcc_RequestState_t Ut_I2c_RccAnyClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *periphClk = UT_I2C_CLK_HZ;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief   Interrupt service routine of every I2C is registered and ends without DSB.
 *
 * \details Every I2C of the MCU initialized in ISR mode, the captured interrupt (event and error
 *          line share one handler) is called once without running transfer, the peripheral is
 *          deinitialized.
 *
 * \note    STM32H7 (Cortex-M7): the DSB at the end of the ISRs of STM32G4 (Cortex-M4 erratum
 *          838869, AB#1104) is not used.
 *
 * \par Expected results
 * - ISR registered for every peripheral, no DSB executed by the ISR.
 */
void Ut_I2c_Isr_AllPeriphs_RegisteredWithoutDsb( void )
{
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_ISR );

    for( uint32_t periphIdx = 0u; I2C_PERIPH_CNT > periphIdx; periphIdx++ )
    {
        i2c_Config_t config = Ut_I2c_Get_Config();

        config.PeriphId   = (i2c_PeriphId_t)periphIdx;
        config.DataConfig = &dataConfig;

        Ut_I2c_Reset_Mocks();
        Ut_I2c_Ignore_PeriphMocks();
        Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicAnyHandlerStub );
        Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccAnyClkSrcStub );
        Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccAnyClkStub );
        utI2c_AnyIsr = NULL;

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
        TEST_ASSERT_NOT_NULL( utI2c_AnyIsr );

        const uint32_t dsbCnt = CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB );

        utI2c_AnyIsr();

        TEST_ASSERT_EQUAL_UINT32( dsbCnt, CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( (i2c_PeriphId_t)periphIdx ) );
    }
}


/**
 * \brief   Other I2C peripherals use their own DMAMUX1 requests, I2C4 refuses DMA mode.
 *
 * \details I2C2, I2C3 and I2C5 (STM32H72x / H73x) initialized in DMA mode (DMA1 streams 1 / 2),
 *          I2C4 (D3 domain - requests routed by DMAMUX2 to BDMA) initialized in DMA mode.
 *
 * \par Expected results
 * - I2C2 / I2C3 / I2C5: I2C_REQUEST_OK, DMAMUX1 requests and TXDR / RXDR of the peripheral.
 * - I2C4: I2C_REQUEST_ERROR without Dma_Init(), I2c_Get_PeriphDmaReq() of I2C4 returns error.
 */
void Ut_I2c_Dma_OtherPeriphs_OwnRequestsI2c4Refused( void )
{
#if defined(STM32H7RS)
    TEST_IGNORE_MESSAGE( "STM32H7R / H7S: GPDMA (test of the DMA stream functionality of the classic lines)" );
#else
    const struct
    {
        i2c_PeriphId_t     PeriphId;
        I2C_TypeDef *      PeriphReg;
        dma_PeriphReqId_t  TxReq;
        dma_PeriphReqId_t  RxReq;
    }   periphLut[] =
    {
        { I2C_PERIPH_2, I2C2, DMA_REQ_I2C2_TX, DMA_REQ_I2C2_RX },
        { I2C_PERIPH_3, I2C3, DMA_REQ_I2C3_TX, DMA_REQ_I2C3_RX },
#ifdef I2C5
        { I2C_PERIPH_5, I2C5, DMA_REQ_I2C5_TX, DMA_REQ_I2C5_RX },
#endif /* I2C5 */
    };
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    i2c_Config_t           config     = Ut_I2c_Get_Config();
    dma_PeriphReqId_t      txRequest  = DMA_REQ_MEM2MEM;
    dma_PeriphReqId_t      rxRequest  = DMA_REQ_MEM2MEM;

    config.DataConfig = &dataConfig;

    for( uint32_t idx = 0u; ( sizeof( periphLut ) / sizeof( periphLut[ 0u ] ) ) > idx; idx++ )
    {
        config.PeriphId = periphLut[ idx ].PeriphId;

        Ut_I2c_Reset_Mocks();
        Ut_I2c_Ignore_PeriphMocks();
        Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccAnyClkSrcStub );
        Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccAnyClkStub );
        Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicAnyHandlerStub );
        Dma_Init_StubWithCallback( Ut_I2c_DmaInitStub );
        utI2c_DmaInitCnt = 0u;

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
        TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );
        TEST_ASSERT_EQUAL( periphLut[ idx ].TxReq, utI2c_DmaConfig[ 0u ].PeripheralReqId );
        TEST_ASSERT_EQUAL( periphLut[ idx ].RxReq, utI2c_DmaConfig[ 1u ].PeripheralReqId );
        TEST_ASSERT_EQUAL_HEX32( (dma_PeriphAddr_t)(uintptr_t)&periphLut[ idx ].PeriphReg->TXDR, utI2c_DmaConfig[ 0u ].PeriphAddress );
        TEST_ASSERT_EQUAL_HEX32( (dma_PeriphAddr_t)(uintptr_t)&periphLut[ idx ].PeriphReg->RXDR, utI2c_DmaConfig[ 1u ].PeriphAddress );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( periphLut[ idx ].PeriphId ) );
    }

    /* I2C4 - no DMAMUX1 request */
    config.PeriphId = I2C_PERIPH_4;

    Ut_I2c_Reset_Mocks();
    Ut_I2c_Ignore_PeriphMocks();
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccAnyClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccAnyClkStub );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_I2c_NvicAnyHandlerStub );
    Dma_Init_StubWithCallback( Ut_I2c_DmaInitStub );
    utI2c_DmaInitCnt = 0u;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_DmaInitCnt );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_PeriphDmaReq( I2C_PERIPH_4, &txRequest, &rxRequest ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphDmaReq( I2C_PERIPH_1, &txRequest, &rxRequest ) );
    TEST_ASSERT_EQUAL( DMA_REQ_I2C1_TX, txRequest );
    TEST_ASSERT_EQUAL( DMA_REQ_I2C1_RX, rxRequest );
#endif /* STM32H7RS */
}

/* ----------------- GPDMA mode (STM32H7R / STM32H7S) ----------------- */

/**
 * \brief   DMA data configuration initializes the GPDMA channels of both directions.
 *
 * \details Initializes I2C1 with DMA data handling and evaluates the GPDMA configurations
 *          passed to Gpdma_Init().
 *
 * \par Expected results
 * - Gpdma_Init() 2x.
 * - Transmission: memory to peripheral, request I2C1_TX, destination TXDR (static), source
 *   address increment, 8-bit, priority and channel of the configuration.
 * - Reception: peripheral to memory, request I2C1_RX, source RXDR (static), destination address
 *   increment.
 * - Error handler of both channels, no complete / half transfer handler, all GPDMA errors
 *   reported, GPDMA interrupt of both channels enabled, DMA requests of the I2C disabled.
 */
void Ut_I2c_Init_GpdmaDataConfig_ChannelsInitialized( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    uint32_t         txIdx      = 0u;
    uint32_t         rxIdx      = 0u;

    Ut_I2c_Setup_GpdmaMocks();
    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_GpdmaInitCnt );

    txIdx = Ut_I2c_Find_GpdmaInit( dataConfig.TxDmaChannelId );
    rxIdx = Ut_I2c_Find_GpdmaInit( dataConfig.RxDmaChannelId );

    TEST_ASSERT_EQUAL( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, utI2c_GpdmaConfig[ txIdx ].PeriphId );
    TEST_ASSERT_EQUAL( (gpdma_Priority_t)dataConfig.TxDmaPriority, utI2c_GpdmaConfig[ txIdx ].ChannelPrio );
    TEST_ASSERT_EQUAL( GPDMA_DIR_MEMORY_TO_PERIPH,  utI2c_GpdmaXferConfig[ txIdx ].Direction );
    TEST_ASSERT_EQUAL( GPDMA_REQ_I2C1_TX,           utI2c_GpdmaXferConfig[ txIdx ].RequestSource );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_I2C_REG->TXDR, utI2c_GpdmaXferConfig[ txIdx ].DestinationAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC,           utI2c_GpdmaXferConfig[ txIdx ].DestinationAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT,        utI2c_GpdmaXferConfig[ txIdx ].SourceAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_DATA_SIZE_8BITS,       utI2c_GpdmaXferConfig[ txIdx ].SourceDataSize );

    TEST_ASSERT_EQUAL( (gpdma_PeriphId_t)dataConfig.RxDmaPeriphId, utI2c_GpdmaConfig[ rxIdx ].PeriphId );
    TEST_ASSERT_EQUAL( (gpdma_Priority_t)dataConfig.RxDmaPriority, utI2c_GpdmaConfig[ rxIdx ].ChannelPrio );
    TEST_ASSERT_EQUAL( GPDMA_DIR_PERIPH_TO_MEMORY,  utI2c_GpdmaXferConfig[ rxIdx ].Direction );
    TEST_ASSERT_EQUAL( GPDMA_REQ_I2C1_RX,           utI2c_GpdmaXferConfig[ rxIdx ].RequestSource );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_I2C_REG->RXDR, utI2c_GpdmaXferConfig[ rxIdx ].SourceAddr );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_STATIC,           utI2c_GpdmaXferConfig[ rxIdx ].SourceAddrMode );
    TEST_ASSERT_EQUAL( GPDMA_ADDR_INCREMENT,        utI2c_GpdmaXferConfig[ rxIdx ].DestinationAddrMode );

    for( uint32_t cfgIdx = 0u; 2u > cfgIdx; cfgIdx++ )
    {
        TEST_ASSERT_NOT_NULL( utI2c_GpdmaConfig[ cfgIdx ].ErrorIsr );
        TEST_ASSERT_NULL( utI2c_GpdmaConfig[ cfgIdx ].TransferCompleteIsr );
        TEST_ASSERT_NULL( utI2c_GpdmaConfig[ cfgIdx ].HalfTransferIsr );
        TEST_ASSERT_EQUAL( UT_I2C_GPDMA_ERROR_MASK, utI2c_GpdmaConfig[ cfgIdx ].ErrorMask );
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.TxDmaChannelId )->IrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.RxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.RxDmaChannelId )->IrqOnCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN ) );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   GPDMA initialization failure is reported.
 *
 * \details Gpdma_Init() returns error.
 *
 * \par Expected results
 * - I2c_Init() returns I2C_REQUEST_ERROR.
 */
void Ut_I2c_Init_GpdmaInitFailure_ReturnsError( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    i2c_Config_t     config     = Ut_I2c_Get_Config();

    config.DataConfig = &dataConfig;

    Ut_I2c_Setup_GpdmaMocks();
    Ut_I2c_Ignore_PeriphMocks();
    utI2c_GpdmaInitState = GPDMA_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   Configured GPDMA channels are reused, only the priority is updated.
 *
 * \details Initializes DMA data handling, then configures it again with the same channels and
 *          other priorities.
 *
 * \par Expected results
 * - Gpdma_Init() called only 2x (first configuration).
 * - Second configuration: Gpdma_Set_Priority() with the new priorities, GPDMA interrupts enabled again.
 */
void Ut_I2c_Set_DataConfig_GpdmaSameChannels_PriorityUpdatedWithoutInit( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t     dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    utI2c_GpdmaChannel_t * txChannel  = NULL;
    utI2c_GpdmaChannel_t * rxChannel  = NULL;

    Ut_I2c_Setup_GpdmaMocks();
    txChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.TxDmaChannelId );
    rxChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.RxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.RxDmaChannelId );

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_GpdmaInitCnt );

    dataConfig.TxDmaPriority = I2C_DMA_PRIORITY_VERYHIGH;
    dataConfig.RxDmaPriority = I2C_DMA_PRIORITY_MEDIUM;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_GpdmaInitCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, txChannel->PrioCnt );
    TEST_ASSERT_EQUAL( (gpdma_Priority_t)I2C_DMA_PRIORITY_VERYHIGH, txChannel->Prio );
    TEST_ASSERT_EQUAL_UINT32( 1u, rxChannel->PrioCnt );
    TEST_ASSERT_EQUAL( (gpdma_Priority_t)I2C_DMA_PRIORITY_MEDIUM, rxChannel->Prio );
    TEST_ASSERT_EQUAL_UINT32( 2u, txChannel->IrqOnCnt );
    TEST_ASSERT_EQUAL_UINT32( 2u, rxChannel->IrqOnCnt );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA write programs the transmit channel and ends by the STOP flag.
 *
 * \details Starts a 2 byte write and calls the captured ISR with STOPF (DMA moved all bytes).
 *
 * \par Expected results
 * - Block size 2, source address the transmit buffer, channel enabled, TXDMAEN set, receive
 *   channel not enabled, event interrupts enabled (no TXIE / RXIE).
 * - Complete callback 1x, DMA requests disabled, both channels stopped.
 */
void Ut_I2c_Gpdma_Write_ChannelProgrammedAndStopCompletes( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    utI2c_GpdmaChannel_t *    txChannel  = NULL;
    utI2c_GpdmaChannel_t *    rxChannel  = NULL;

    Ut_I2c_Setup_GpdmaMocks();
    txChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.TxDmaChannelId );
    rxChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.RxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.RxDmaChannelId );
    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    TEST_ASSERT_EQUAL_UINT32( 2u, txChannel->BlockSize );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utI2c_TxBuf, txChannel->SrcAddr );
    TEST_ASSERT_EQUAL_UINT32( 1u, txChannel->ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, rxChannel->ActiveCnt );
    TEST_ASSERT_BITS_HIGH( I2C_CR1_TXDMAEN, UT_I2C_REG->CR1 );
    TEST_ASSERT_BITS_LOW( I2C_CR1_RXDMAEN, UT_I2C_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_GPDMA_EVENT_IRQS, UT_I2C_REG->CR1 & ( UT_I2C_GPDMA_EVENT_IRQS | I2C_CR1_TXIE | I2C_CR1_RXIE ) );

    const uint32_t txInactive = txChannel->InactiveCnt;
    const uint32_t rxInactive = rxChannel->InactiveCnt;

    utI2c_GpdmaRemaining = 0u;
    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
    TEST_ASSERT_BITS_LOW( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN, UT_I2C_REG->CR1 );
    TEST_ASSERT_GREATER_THAN_UINT32( txInactive, txChannel->InactiveCnt );
    TEST_ASSERT_GREATER_THAN_UINT32( rxInactive, rxChannel->InactiveCnt );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA read programs the receive channel and ends by the STOP flag.
 *
 * \details Starts a 3 byte read and calls the captured ISR with STOPF (DMA moved all bytes).
 *
 * \par Expected results
 * - Block size 3, destination address the receive buffer, channel enabled, RXDMAEN set, transmit
 *   channel not enabled.
 * - Complete callback 1x, DMA requests disabled.
 */
void Ut_I2c_Gpdma_Read_ChannelProgrammedAndStopCompletes( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 3u };
    utI2c_GpdmaChannel_t *    txChannel  = NULL;
    utI2c_GpdmaChannel_t *    rxChannel  = NULL;

    Ut_I2c_Setup_GpdmaMocks();
    txChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.TxDmaChannelId );
    rxChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.RxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.RxDmaChannelId );
    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    TEST_ASSERT_EQUAL_UINT32( 3u, rxChannel->BlockSize );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utI2c_RxBuf, rxChannel->DstAddr );
    TEST_ASSERT_EQUAL_UINT32( 1u, rxChannel->ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, txChannel->ActiveCnt );
    TEST_ASSERT_BITS_HIGH( I2C_CR1_RXDMAEN, UT_I2C_REG->CR1 );
    TEST_ASSERT_BITS_LOW( I2C_CR1_TXDMAEN, UT_I2C_REG->CR1 );

    utI2c_GpdmaRemaining = 0u;
    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
    TEST_ASSERT_BITS_LOW( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN, UT_I2C_REG->CR1 );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA write followed by read programs both channels.
 *
 * \details Starts a 2 byte write with a 3 byte read (repeated start) and ends it by STOPF.
 *
 * \par Expected results
 * - Both channels programmed (block sizes 2 and 3), TXDMAEN and RXDMAEN set, complete callback 1x.
 */
void Ut_I2c_Gpdma_WriteRead_BothChannelsProgrammed( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = utI2c_RxBuf, .RxSize = 3u };
    utI2c_GpdmaChannel_t *    txChannel  = NULL;
    utI2c_GpdmaChannel_t *    rxChannel  = NULL;

    Ut_I2c_Setup_GpdmaMocks();
    txChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.TxDmaChannelId );
    rxChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.RxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.RxDmaChannelId );
    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    TEST_ASSERT_EQUAL_UINT32( 2u, txChannel->BlockSize );
    TEST_ASSERT_EQUAL_UINT32( 3u, rxChannel->BlockSize );
    TEST_ASSERT_EQUAL_UINT32( 1u, txChannel->ActiveCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, rxChannel->ActiveCnt );
    TEST_ASSERT_BITS_HIGH( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN, UT_I2C_REG->CR1 );

    utI2c_GpdmaRemaining = 0u;
    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   STOP flag with unfinished DMA transfer is reported as DMA transfer error.
 *
 * \details Starts a 3 byte read, the receive channel reports 1 remaining byte at STOPF.
 *
 * \par Expected results
 * - Error callback 1x with I2C_XFER_ERROR_DMA_TRANSFER, no complete callback.
 */
void Ut_I2c_Gpdma_StopWithUnfinishedTransfer_DmaTransferErrorReported( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Setup_GpdmaMocks();
    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();

    utI2c_GpdmaRemaining = 1u;
    Ut_I2c_Call_Isr( I2C_ISR_STOPF );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_DMA_TRANSFER, utI2c_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   DMA transfer start reports GPDMA channel enable error.
 *
 * \details Gpdma_Set_ChannelActive() returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR for write and for read, transfer not started (second start possible).
 */
void Ut_I2c_Gpdma_Start_ChannelActivationError_ReturnsError( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    const i2c_XferRequest_t writeReq   = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    const i2c_XferRequest_t readReq    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = NULL, .TxSize = 0u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Setup_GpdmaMocks();
    Ut_I2c_Init( &dataConfig );

    utI2c_GpdmaActiveState = GPDMA_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &writeReq ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &readReq ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   GPDMA errors are reported to the user and abort the transfer.
 *
 * \details Starts a write and calls the transmit error handler captured from Gpdma_Init() with
 *          the transfer, configuration, configuration update and trigger overrun error bits
 *          (every call after a new transfer start), then the receive error handler.
 *
 * \par Expected results
 * - Error callback with I2C_XFER_ERROR_DMA_TRANSFER / _DMA_CONFIG / _DMA_CONFIG_UPDATE /
 *   _DMA_TRIGGER_OVERRUN for the respective bit, no complete callback.
 */
void Ut_I2c_Gpdma_ErrorBits_ReportedToUser( void )
{
#if defined(STM32H7RS)
    const struct
    {
        gpdma_ErrorMaskId_t DmaError;
        i2c_XferErrorId_t   ErrorId;
    }   errorLut[] =
    {
        { GPDMA_ERROR_TRANSFER,      I2C_XFER_ERROR_DMA_TRANSFER        },
        { GPDMA_ERROR_CONFIG_ERROR,  I2C_XFER_ERROR_DMA_CONFIG          },
        { GPDMA_ERROR_CONFIG_UPDATE, I2C_XFER_ERROR_DMA_CONFIG_UPDATE   },
        { GPDMA_ERROR_TRIG_OVERRUN,  I2C_XFER_ERROR_DMA_TRIGGER_OVERRUN },
    };
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = utI2c_RxBuf, .RxSize = 3u };
    uint32_t                txIdx      = 0u;
    uint32_t                rxIdx      = 0u;

    Ut_I2c_Setup_GpdmaMocks();
    Ut_I2c_Init( &dataConfig );
    txIdx = Ut_I2c_Find_GpdmaInit( dataConfig.TxDmaChannelId );
    rxIdx = Ut_I2c_Find_GpdmaInit( dataConfig.RxDmaChannelId );

    for( uint32_t errIdx = 0u; ( sizeof( errorLut ) / sizeof( errorLut[ 0u ] ) ) > errIdx; errIdx++ )
    {
        utI2c_ErrorCnt  = 0u;
        utI2c_LastError = I2C_XFER_ERROR_CNT;

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
        Ut_I2c_Set_StartSent();

        utI2c_GpdmaConfig[ txIdx ].ErrorIsr( errorLut[ errIdx ].DmaError );

        TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
        TEST_ASSERT_EQUAL( errorLut[ errIdx ].ErrorId, utI2c_LastError );
    }

    utI2c_ErrorCnt  = 0u;
    utI2c_LastError = I2C_XFER_ERROR_CNT;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    utI2c_GpdmaConfig[ rxIdx ].ErrorIsr( GPDMA_ERROR_TRANSFER );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_ErrorCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_DMA_TRANSFER, utI2c_LastError );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   Deinitialization releases the GPDMA channels.
 *
 * \details Initializes DMA data handling and deinitializes the peripheral.
 *
 * \par Expected results
 * - Both GPDMA channels stopped and their interrupts disabled, DMA requests of the I2C disabled.
 */
void Ut_I2c_Deinit_GpdmaDataConfig_ChannelsReleased( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t     dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    utI2c_GpdmaChannel_t * txChannel  = NULL;
    utI2c_GpdmaChannel_t * rxChannel  = NULL;

    Ut_I2c_Setup_GpdmaMocks();
    txChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.TxDmaChannelId );
    rxChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.RxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.RxDmaChannelId );
    Ut_I2c_Init( &dataConfig );

    const uint32_t txInactive = txChannel->InactiveCnt;
    const uint32_t rxInactive = rxChannel->InactiveCnt;
    const uint32_t txIrqOff   = txChannel->IrqOffCnt;
    const uint32_t rxIrqOff   = rxChannel->IrqOffCnt;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );

    TEST_ASSERT_GREATER_THAN_UINT32( txInactive, txChannel->InactiveCnt );
    TEST_ASSERT_GREATER_THAN_UINT32( rxInactive, rxChannel->InactiveCnt );
    TEST_ASSERT_EQUAL_UINT32( txIrqOff + 1u, txChannel->IrqOffCnt );
    TEST_ASSERT_EQUAL_UINT32( rxIrqOff + 1u, rxChannel->IrqOffCnt );
    TEST_ASSERT_BITS_LOW( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN, UT_I2C_REG->CR1 );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/**
 * \brief   Stopping a running DMA transfer disables the DMA requests and the channels.
 *
 * \details Starts a write and stops it by I2c_Set_XferStop().
 *
 * \par Expected results
 * - Transmit channel stopped, TXDMAEN cleared.
 */
void Ut_I2c_Gpdma_XferStop_RunningTransfer_ChannelsAndRequestsStopped( void )
{
#if defined(STM32H7RS)
    i2c_DataConfig_t        dataConfig = Ut_I2c_Get_GpdmaDataConfig();
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    utI2c_GpdmaChannel_t *    txChannel  = NULL;

    Ut_I2c_Setup_GpdmaMocks();
    txChannel = Ut_I2c_Get_GpdmaChannel( (gpdma_PeriphId_t)dataConfig.TxDmaPeriphId, (gpdma_ChannelId_t)dataConfig.TxDmaChannelId );
    Ut_I2c_Init( &dataConfig );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );
    Ut_I2c_Set_StartSent();
    TEST_ASSERT_BITS_HIGH( I2C_CR1_TXDMAEN, UT_I2C_REG->CR1 );

    const uint32_t inactiveCnt = txChannel->InactiveCnt;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStop( UT_I2C_PERIPH ) );

    TEST_ASSERT_GREATER_THAN_UINT32( inactiveCnt, txChannel->InactiveCnt );
    TEST_ASSERT_BITS_LOW( I2C_CR1_TXDMAEN, UT_I2C_REG->CR1 );
#else
    TEST_IGNORE_MESSAGE( "GPDMA data transfer: STM32H7R / STM32H7S only (classic lines: DMA streams, tests Ut_I2c_Dma_*)" );
#endif /* STM32H7RS */
}


/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief NVIC handler registration stub - stores ISR of the I2C (event and error line).
 */
static nvic_RequestState_t Ut_I2c_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_TRUE( ( UT_I2C_NVIC_EV == irqId ) ||
                      ( UT_I2C_NVIC_ER == irqId )    );
    TEST_ASSERT_NOT_NULL( irqHandler );

    utI2c_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief RCC clock source stub - I2C1 kernel clock source is \ref utI2c_ClkSrc.
 */
static rcc_RequestState_t Ut_I2c_RccGetClkSrcStub( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_I2C_RCC_PCLK, periphId );

    *periphClkSrc = utI2c_ClkSrc;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief RCC clock stub - returns \ref utI2c_PclkHz for PCLK1 (kernel clock source and
 *        APB1 clock) and \ref utI2c_HsiHz for HSI.
 */
static rcc_RequestState_t Ut_I2c_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;

    if( UT_I2C_RCC_HSI == periphId )
    {
        *periphClk = utI2c_HsiHz;
    }
    else
    {
        TEST_ASSERT_EQUAL( UT_I2C_RCC_PCLK, periphId );

        *periphClk = utI2c_PclkHz;
    }

    return ( RCC_REQUEST_OK );
}


/**
 * \brief RCC reset stub of the I2C1 with HSI kernel clock - emulates the peripheral
 *        reset (I2C registers cleared).
 */
static rcc_RequestState_t Ut_I2c_RccResetHsiStub( rcc_PeriphId_t periphId, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_I2C_RCC_HSI, periphId );

    UT_I2C_REG->CR1     = 0u;
    UT_I2C_REG->CR2     = 0u;
    UT_I2C_REG->TIMINGR = 0u;

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


#if !defined(STM32H7RS)
/**
 * \brief DMA channel initialization stub - stores the configuration (TX at index 0, RX at
 *        index 1), the RX channel returns \ref utI2c_DmaInitRxState.
 */
static dma_RequestState_t Ut_I2c_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    dma_RequestState_t retState = DMA_REQUEST_OK;

    (void)callCnt;

    TEST_ASSERT_NOT_NULL( dmaConfig );

    if( DMA_DIR_PERIPH_TO_MEMORY == dmaConfig->Direction )
    {
        utI2c_DmaConfig[ 1u ] = *dmaConfig;
        retState              = utI2c_DmaInitRxState;
    }
    else
    {
        utI2c_DmaConfig[ 0u ] = *dmaConfig;
    }

    utI2c_DmaInitCnt++;

    return ( retState );
}
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
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
#endif /* !STM32H7RS */


#if !defined(STM32H7RS)
/**
 * \brief Dma_Set_InterruptInactive() stub (DMA channel released) - counts the calls and
 *        returns \ref utI2c_DmaIrqOffState.
 */
static dma_RequestState_t Ut_I2c_DmaIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)dmaBus;
    (void)dmaChannel;
    (void)callCnt;

    utI2c_DmaIrqOffCnt++;

    return ( utI2c_DmaIrqOffState );
}
#endif /* !STM32H7RS */


#if defined(STM32H7RS)
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
#else
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
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
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
    Dma_Set_InterruptActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_InterruptInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferActive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_DataCount_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Get_DataCount_StubWithCallback( Ut_I2c_DmaGetCountStub );
}
#endif /* STM32H7RS */


/**
 * \brief Releases data handling of the previous test (static module context) and
 *        re-initializes the mocks.
 */
static void Ut_I2c_Release( void )
{
    Ut_I2c_Ignore_PeriphMocks();

#if defined(STM32H7RS)
    /* Release of a GPDMA data handling disables the GPDMA channels (not ignored in the tests, which count the calls) */
    Gpdma_Set_ChannelInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Set_InterruptInactive_IgnoreAndReturn( GPDMA_REQUEST_OK );
#endif /* STM32H7RS */

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
    MockGpdma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();
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
    dataConfig.TxDmaChannelId       = UT_I2C_DMA_TX_CHANNEL;
    dataConfig.TxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    dataConfig.RxDmaPeriphId        = I2C_DMA_PERIPH_1;
    dataConfig.RxDmaChannelId       = UT_I2C_DMA_RX_CHANNEL;
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


/**
 * \brief HW model - CR2.START is cleared by hardware while the peripheral is disabled (PE = 0).
 *
 * Runs in background thread (\ref RegMem_Set_ModelActive), no Unity assertions.
 */
static void Ut_I2c_HwModel( void )
{
    if( 0u == ( UT_I2C_REG->CR1 & I2C_CR1_PE ) )
    {
        utI2c_PeOffSeen = 1u;
        (void)__atomic_and_fetch( &UT_I2C_REG->CR2, ~I2C_CR2_START, __ATOMIC_SEQ_CST );
    }
    else
    {
        /* Peripheral enabled - START is handled by the address phase */
    }
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


#if defined(STM32H7RS)
/**
 * \brief Installs the GPDMA stubs of a DMA test and clears the records.
 */
static void Ut_I2c_Setup_GpdmaMocks( void )
{
    (void)memset( utI2c_GpdmaConfig, 0, sizeof( utI2c_GpdmaConfig ) );
    (void)memset( utI2c_GpdmaXferConfig, 0, sizeof( utI2c_GpdmaXferConfig ) );
    (void)memset( utI2c_GpdmaChannel, 0, sizeof( utI2c_GpdmaChannel ) );

    utI2c_GpdmaInitCnt     = 0u;
    utI2c_GpdmaInitState   = GPDMA_REQUEST_OK;
    utI2c_GpdmaActiveState = GPDMA_REQUEST_OK;
    utI2c_GpdmaRemaining   = 0u;

    Gpdma_Get_DefaultConfig_IgnoreAndReturn( GPDMA_REQUEST_OK );
    Gpdma_Init_StubWithCallback( Ut_I2c_GpdmaInitStub );
    Gpdma_Set_ChannelActive_StubWithCallback( Ut_I2c_GpdmaActiveStub );
    Gpdma_Set_ChannelInactive_StubWithCallback( Ut_I2c_GpdmaInactiveStub );
    Gpdma_Set_InterruptActive_StubWithCallback( Ut_I2c_GpdmaIrqOnStub );
    Gpdma_Set_InterruptInactive_StubWithCallback( Ut_I2c_GpdmaIrqOffStub );
    Gpdma_Set_Priority_StubWithCallback( Ut_I2c_GpdmaPrioStub );
    Gpdma_Set_BlockSize_StubWithCallback( Ut_I2c_GpdmaBlockSizeStub );
    Gpdma_Set_SourceAddr_StubWithCallback( Ut_I2c_GpdmaSrcAddrStub );
    Gpdma_Set_DestinationAddr_StubWithCallback( Ut_I2c_GpdmaDstAddrStub );
    Gpdma_Get_BlockSize_StubWithCallback( Ut_I2c_GpdmaRemainingStub );
}


/**
 * \brief Returns DMA data configuration. Every call selects other GPDMA channels than the previous
 *        call (the module keeps its channel ownership).
 */
static i2c_DataConfig_t Ut_I2c_Get_GpdmaDataConfig( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    dataConfig.TxDmaChannelId = (i2c_DmaChannelId_t)( ( 2u * utI2c_GpdmaChannelSel ) % (uint32_t)I2C_DMA_CHANNEL_CNT );
    dataConfig.RxDmaChannelId = (i2c_DmaChannelId_t)( ( ( 2u * utI2c_GpdmaChannelSel ) + 1u ) % (uint32_t)I2C_DMA_CHANNEL_CNT );

    utI2c_GpdmaChannelSel++;

    return ( dataConfig );
}


/**
 * \brief Returns index of the Gpdma_Init() record of the GPDMA channel.
 *
 * \param channelId [in]: GPDMA channel
 */
static uint32_t Ut_I2c_Find_GpdmaInit( i2c_DmaChannelId_t channelId )
{
    uint32_t foundIdx = UT_I2C_GPDMA_CFG_CNT;

    for( uint32_t cfgIdx = 0u; ( UT_I2C_GPDMA_CFG_CNT > cfgIdx ) && ( UT_I2C_GPDMA_CFG_CNT == foundIdx ); cfgIdx++ )
    {
        if( ( cfgIdx < utI2c_GpdmaInitCnt ) && ( (gpdma_ChannelId_t)channelId == utI2c_GpdmaConfig[ cfgIdx ].ChannelId ) )
        {
            foundIdx = cfgIdx;
        }
        else
        {
            /* Other channel */
        }
    }

    TEST_ASSERT_LESS_THAN_UINT32_MESSAGE( UT_I2C_GPDMA_CFG_CNT, foundIdx, "GPDMA channel was not initialized" );

    return ( foundIdx );
}


/** \brief Returns record of the GPDMA channel calls */
static utI2c_GpdmaChannel_t * Ut_I2c_Get_GpdmaChannel( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel )
{
    TEST_ASSERT_LESS_THAN_UINT32( GPDMA_PERIPH_CNT, (uint32_t)dmaBus );
    TEST_ASSERT_LESS_THAN_UINT32( UT_I2C_GPDMA_CHANNELS, (uint32_t)dmaChannel );

    return ( &utI2c_GpdmaChannel[ dmaBus ][ dmaChannel ] );
}


/** \brief Gpdma_Init() stub - stores the configuration */
static gpdma_RequestState_t Ut_I2c_GpdmaInitStub( gpdma_ConfigStruct_t * const configStruct, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( configStruct );
    TEST_ASSERT_NOT_NULL( configStruct->TransferConfig );

    if( UT_I2C_GPDMA_CFG_CNT > utI2c_GpdmaInitCnt )
    {
        utI2c_GpdmaConfig[ utI2c_GpdmaInitCnt ]     = *configStruct;
        utI2c_GpdmaXferConfig[ utI2c_GpdmaInitCnt ] = *configStruct->TransferConfig;
    }
    else
    {
        /* Record buffer full */
    }

    utI2c_GpdmaInitCnt++;

    return ( utI2c_GpdmaInitState );
}


/** \brief Gpdma_Set_ChannelActive() stub - returns \ref utI2c_GpdmaActiveState */
static gpdma_RequestState_t Ut_I2c_GpdmaActiveStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel )->ActiveCnt++;
    return ( utI2c_GpdmaActiveState );
}


/** \brief Gpdma_Set_ChannelInactive() stub */
static gpdma_RequestState_t Ut_I2c_GpdmaInactiveStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel )->InactiveCnt++;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_InterruptActive() stub */
static gpdma_RequestState_t Ut_I2c_GpdmaIrqOnStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel )->IrqOnCnt++;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_InterruptInactive() stub */
static gpdma_RequestState_t Ut_I2c_GpdmaIrqOffStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel )->IrqOffCnt++;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_Priority() stub */
static gpdma_RequestState_t Ut_I2c_GpdmaPrioStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_Priority_t channelPrio, int callCnt )
{
    utI2c_GpdmaChannel_t * const channel = Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel );

    (void)callCnt;
    channel->PrioCnt++;
    channel->Prio = channelPrio;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_BlockSize() stub */
static gpdma_RequestState_t Ut_I2c_GpdmaBlockSizeStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t blockSize, int callCnt )
{
    (void)callCnt;
    Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel )->BlockSize = blockSize;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_SourceAddr() stub */
static gpdma_RequestState_t Ut_I2c_GpdmaSrcAddrStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_SrcAddr_t sourceAddr, int callCnt )
{
    (void)callCnt;
    Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel )->SrcAddr = sourceAddr;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Set_DestinationAddr() stub */
static gpdma_RequestState_t Ut_I2c_GpdmaDstAddrStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_DstAddr_t destAddr, int callCnt )
{
    (void)callCnt;
    Ut_I2c_Get_GpdmaChannel( dmaBus, dmaChannel )->DstAddr = destAddr;
    return ( GPDMA_REQUEST_OK );
}


/** \brief Gpdma_Get_BlockSize() stub - returns \ref utI2c_GpdmaRemaining */
static gpdma_RequestState_t Ut_I2c_GpdmaRemainingStub( gpdma_PeriphId_t dmaBus, gpdma_ChannelId_t dmaChannel, gpdma_BlockSize_t * const blockSize, int callCnt )
{
    (void)callCnt;
    (void)dmaBus;
    (void)dmaChannel;
    *blockSize = utI2c_GpdmaRemaining;
    return ( GPDMA_REQUEST_OK );
}
#endif /* STM32H7RS */
