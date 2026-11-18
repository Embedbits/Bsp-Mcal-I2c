/**
 * \author Mr.Nobody
 * \file ItTest_I2c.c
 * \ingroup I2c
 * \brief Integration tests of I2C module on target (STM32F7).
 *
 * I2c module runs on the MCU together with real RCC, NVIC, GPIO and DMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): peripheral and pin initialization, kernel clock
 * selection, timing (SCL frequency) calculation from the real kernel clock,
 * Fast-mode Plus drive in SYSCFG, idle bus state, master sequencing of the
 * hardware (address, NACK, repeated START, NBYTES reload, 10-bit addressing) in
 * polling, interrupt and DMA mode, slave transfers (own addresses, register
 * device, buffers, DMA) and deinitialization.
 *
 * Master: I2C1 on PB8 (SCL) / PB9 (SDA) - Arduino D15 / D14 of both board types -
 * with internal pull-ups. Address 0x50 (7-bit) and 0x2A5 (10-bit) are not present
 * on the bus (NACK). DMA1 stream 6 (TX) / stream 0 (RX), channel 1.
 *
 * Master + slave tests: the slave is a second I2C peripheral of the same MCU
 * (slave of this module) wired to the master bus (SCL - SCL, SDA - SDA, external
 * pull-up resistors 4.7 kOhm, pins in the board blocks below). Fixture of the board
 * (EmBi_Platform IntegrationTesting, "Board fixtures"): the wiring is feature
 * I2C_LOOP - the master + slave tests are ignored on a board with a fixture without
 * it (a board without a fixture is expected to be wired) and on boards without a
 * known wiring. Both peripherals stretch the clock, the master and the slave share
 * one CPU in every mode combination.
 *
 * The register device of the transfer tests (write, write + repeated START + read,
 * read only, 10-bit, NBYTES reload) is the slave of the module in interrupt mode: the
 * first written byte selects the register, following written bytes are stored (transfer
 * complete callback) and the read bytes are returned from the selected register (address
 * callback of the read phase).
 *
 * Boards (named by the MCU as the detection of the connected boards does):
 * - STM32F745xG / STM32F746xG - 32F746GDISCOVERY (STM32F746NG); the detection
 *   names every board with ID 0x449 and 1 MB flash STM32F745xG
 * - STM32F722xE - NUCLEO-F722ZE (master I2C1 PB8 / PB9, slave I2C2 PB10 / PB11)
 * - STM32F756xG, STM32F765xI / STM32F767xI - NUCLEO-F756ZG (board file override with
 *   the name STM32F756xG), NUCLEO-F767ZI (Nucleo-144 boards)
 *
 * Used resources: master I2C1, slave I2C2; DMA1 streams 6 / 0 (master DMA mode) and
 * 7 / 2 (slave DMA mode).
 *
 * \note The host halts the core for a few milliseconds on every mailbox access
 *       (probe-rs). Both peripherals stretch the clock, no byte is lost.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "I2c_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* Kernel clock source read back  */
#include "Stm32_system.h"                   /* SYSCFG Fast-mode Plus bits     */
/* ============================= TYPEDEFS =================================== */

/** Callback record of one peripheral */
typedef struct
{
    volatile uint32_t           CompleteCnt;  /**< Transfer complete callbacks           */
    volatile uint32_t           ErrorCnt;     /**< Error callbacks                       */
    volatile i2c_XferErrorId_t  ErrorId;      /**< Error of the last error callback      */
}   it_I2cCallbacks_t;


/** Address match record of the slave */
typedef struct
{
    volatile uint32_t            AddrCnt;     /**< Address callbacks                     */
    volatile i2c_SlaveDir_t      LastDir;     /**< Direction of the last address match   */
    volatile i2c_SlaveAddr_t     LastAddr;    /**< Matched address of the last match     */
    volatile i2c_FunctionState_t RegMode;     /**< Register device pattern is emulated   */
    i2c_SlaveResult_t            Result;      /**< Result of the last complete callback  */
}   it_I2cSlave_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void ItI2c_Init                 ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq );
static void ItI2c_Init_Master          ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq, i2c_AddrMode_t addrMode );
static void ItI2c_Init_Slave           ( i2c_XferMode_t xferMode,
                                         i2c_FreqHz_t busFreq,
                                         const i2c_SlaveConfig_t * const slaveConfig );
static void ItI2c_Init_RegSlave        ( i2c_AddrMode_t addrMode, i2c_FreqHz_t busFreq );
static void ItI2c_Get_SlaveConfig      ( i2c_SlaveConfig_t * const slaveConfig );
static void ItI2c_Wait_XferEnd         ( void );
static void ItI2c_Wait_MasterSlaveEnd  ( void );
static void ItI2c_Check_Xfer           ( const i2c_XferRequest_t * const xferRequest );
static void ItI2c_Check_NackTransfer   ( const i2c_XferRequest_t * const xferRequest );
static void ItI2c_Check_WriteRead      ( i2c_SlaveAddr_t slaveAddr, uint8_t regAddr, i2c_DataCnt_t dataCnt );
static void ItI2c_Check_MasterOk       ( void );
static void ItI2c_Check_SlaveResult    ( i2c_DataCnt_t rxCnt, i2c_DataCnt_t txCnt );
static void ItI2c_Set_Pattern          ( i2c_Data_t * const buffer, i2c_DataCnt_t dataCnt, i2c_Data_t seed );
static void ItI2c_RequireLoop          ( void );

static void ItI2c_XferCompleteCallback ( void );
static void ItI2c_ErrorCallback        ( i2c_XferErrorId_t errorId );
static void ItI2c_SlaveErrorCallback   ( i2c_XferErrorId_t errorId );
static void ItI2c_SlaveAddrCallback    ( i2c_SlaveDir_t slaveDir, i2c_SlaveAddr_t matchedAddr );
static void ItI2c_SlaveCompleteCallback( const i2c_SlaveResult_t * const slaveResult );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32F745xG) || \
    defined(IT_BOARD_STM32F746xG)

    /* 32F746GDISCOVERY */

    /** I2C1 master on Arduino D15 (PB8, SCL) / D14 (PB9, SDA) */
    #define IT_I2C_PERIPH                   ( I2C_PERIPH_1 )
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_RCC_PCLK                 ( RCC_PERIPH_I2C1_PCLK1 )
    #define IT_I2C_RCC_HSI                  ( RCC_PERIPH_I2C1_HSI )
    #define IT_I2C_FMP_BIT                  ( SYSCFG_PMC_I2C1_FMP )

    /** No second I2C on the connectors of the board - slave tests are ignored */
    #define IT_I2C_SLAVE_AVAILABLE          ( 0u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_UNUSED )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_UNUSED )

#elif defined(IT_BOARD_STM32F722xE)

    /* NUCLEO-F722ZE (Nucleo-144): wiring of the common board wiring */

    /** I2C1 master on Arduino D15 (PB8, SCL) / D14 (PB9, SDA) */
    #define IT_I2C_PERIPH                   ( I2C_PERIPH_1 )
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_RCC_PCLK                 ( RCC_PERIPH_I2C1_PCLK1 )
    #define IT_I2C_RCC_HSI                  ( RCC_PERIPH_I2C1_HSI )
    #define IT_I2C_FMP_BIT                  ( SYSCFG_PMC_I2C1_FMP )

    /** I2C2 slave on PB10 (SCL) / PB11 (SDA), wired to the master pins, pull-ups 4.7 kOhm */
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C2_PB10 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C2_PB11 )

#elif defined(IT_BOARD_STM32F756xG) || \
      defined(IT_BOARD_STM32F765xI) || \
      defined(IT_BOARD_STM32F767xI)

    /* NUCLEO-F756ZG / NUCLEO-F767ZI (Nucleo-144) */

    /** I2C1 master on Arduino D15 (PB8, SCL) / D14 (PB9, SDA) */
    #define IT_I2C_PERIPH                   ( I2C_PERIPH_1 )
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_RCC_PCLK                 ( RCC_PERIPH_I2C1_PCLK1 )
    #define IT_I2C_RCC_HSI                  ( RCC_PERIPH_I2C1_HSI )
    #define IT_I2C_FMP_BIT                  ( SYSCFG_PMC_I2C1_FMP )

    /** No wiring of a slave known for the board - slave tests are ignored */
    #define IT_I2C_SLAVE_AVAILABLE          ( 0u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_UNUSED )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_UNUSED )

#else
    #error "Board of I2c integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** DMA1 streams of I2C1 (channel 1) - master */
#define IT_I2C_DMA_TX                       ( I2C_TX_DMA_I2C1_DMA1_STREAM6 )
#define IT_I2C_DMA_RX                       ( I2C_RX_DMA_I2C1_DMA1_STREAM0 )

/** DMA1 streams of I2C2 (channel 7) - slave */
#define IT_I2C_SLAVE_DMA_TX                 ( I2C_TX_DMA_I2C2_DMA1_STREAM7 )
#define IT_I2C_SLAVE_DMA_RX                 ( I2C_RX_DMA_I2C2_DMA1_STREAM2 )

/** Own address 1 of the slave of the module (7-bit and 10-bit) */
#define IT_I2C_SLAVE_ADDR                   ( 0x2Cu )
#define IT_I2C_SLAVE_ADDR10                 ( 0x1B7u )

/** Own address 2 of the slave */
#define IT_I2C_SLAVE_ADDR2                  ( 0x3Au )

/** Address of a not present slave (7-bit and 10-bit) */
#define IT_I2C_ABSENT_ADDR                  ( 0x50u )
#define IT_I2C_ABSENT_ADDR10                ( 0x2A5u )

/** Standard-mode, Fast-mode and Fast-mode Plus SCL frequency [Hz] */
#define IT_I2C_FREQ_STANDARD_HZ             ( 100000u )
#define IT_I2C_FREQ_FAST_HZ                 ( 400000u )
#define IT_I2C_FREQ_FAST_PLUS_HZ            ( 1000000u )

/** Tolerance of the SCL frequency read back from TIMINGR [%] */
#define IT_I2C_FREQ_TOL_PCT                 ( 10u )

/** Maximal count of wait loop iterations (I2c_Task called) */
#define IT_I2C_WAIT_LOOPS                   ( 2000000u )

/** Interrupt priority of the master and of the slave */
#define IT_I2C_IRQ_PRIO                     ( 5u )
#define IT_I2C_SLAVE_IRQ_PRIO               ( 4u )

/** Count of data bytes of the longest write / read back test transfer */
#define IT_I2C_DATA_MAX                     ( 16u )

/** Count of data bytes of the long transfer (more than one NBYTES chunk of 255 bytes) */
#define IT_I2C_LONG_SIZE                    ( 300u )

/** Count of registers of the register device (the long transfer fits) */
#define IT_I2C_REG_CNT                      ( IT_I2C_LONG_SIZE )

/** Size of the receive buffer of the slave (register address and the long transfer) */
#define IT_I2C_SLAVE_RX_SIZE                ( IT_I2C_LONG_SIZE + 1u )

/** Count of bytes of a short slave transfer */
#define IT_I2C_SHORT_SIZE                   ( 8u )

/** Register address and count of registers read in the register read tests */
#define IT_I2C_REG_ADDR                     ( 5u )
#define IT_I2C_REG_READ_CNT                 ( 4u )

/** Count of address matches of a register read (write phase of the register address, read phase) */
#define IT_I2C_REG_READ_ADDR_CNT            ( 2u )

/** Count of bytes read by the master in the read beyond the buffer tests */
#define IT_I2C_BEYOND_READ_CNT              ( 6u )

/** Size of the transmit buffer of the slave in the read beyond the buffer tests */
#define IT_I2C_TX_PART_SIZE                 ( 2u )

/** Size of the receive buffer of the slave in the write beyond the buffer tests */
#define IT_I2C_RX_PART_SIZE                 ( 4u )

/** Fill byte of the slave */
#define IT_I2C_FILL_BYTE                    ( 0xA5u )

/** Seeds of the data patterns */
#define IT_I2C_SEED_MASTER                  ( 0x11u )
#define IT_I2C_SEED_SLAVE                   ( 0x80u )
#define IT_I2C_SEED_REGS                    ( 0x40u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Data handling configuration of the master and of the slave (copied by the module) */
static i2c_DataConfig_t             itI2c_DataConfig;
static i2c_DataConfig_t             itI2c_SlaveDataConfig;

/** Transmit data of NACK transfers */
static const i2c_Data_t             itI2c_TxData[ 2u ] = { 0x00u, 0x10u };

/** Transmit buffer (register address followed by data) and receive buffer of the master */
static i2c_Data_t                   itI2c_TxBuf[ IT_I2C_LONG_SIZE + 1u ];
static i2c_Data_t                   itI2c_RxBuf[ IT_I2C_LONG_SIZE ];

/** Transmit and receive buffers of the slave */
static i2c_Data_t                   itI2c_SlaveTx[ IT_I2C_LONG_SIZE ];
static i2c_Data_t                   itI2c_SlaveRx[ IT_I2C_SLAVE_RX_SIZE ];

/** Registers of the register device */
static i2c_Data_t                   itI2c_Regs[ IT_I2C_REG_CNT ];

/** Callback record of the master and of the slave */
static it_I2cCallbacks_t            itI2c_Master;
static it_I2cCallbacks_t            itI2c_SlaveCb;

/** Address match record of the slave */
static it_I2cSlave_t                itI2c_Slave;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itI2c_Master.CompleteCnt  = 0u;
    itI2c_Master.ErrorCnt     = 0u;
    itI2c_Master.ErrorId      = I2C_XFER_ERROR_CNT;
    itI2c_SlaveCb.CompleteCnt = 0u;
    itI2c_SlaveCb.ErrorCnt    = 0u;
    itI2c_SlaveCb.ErrorId     = I2C_XFER_ERROR_CNT;
    itI2c_Slave.AddrCnt       = 0u;
    itI2c_Slave.LastDir       = I2C_SLAVE_DIR_CNT;
    itI2c_Slave.LastAddr      = 0u;
    itI2c_Slave.RegMode       = I2C_FUNCTION_INACTIVE;
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
 * \details Initializes the master I2C1 (polling mode, 100 kHz, PCLK1 kernel clock, internal
 *          pull-ups on board pins) and reads the configuration back.
 *
 * \par Expected results
 * - Bus frequency 100 kHz +- 10 % (calculated from real kernel clock).
 * - Analog filter enabled, digital filter off, 7-bit addressing.
 * - Peripheral is enabled, kernel clock source is PCLK1, Fast-mode Plus drive off.
 */
void It_I2c_Init_StandardMode_ConfigurationReadBack( void )
{
    i2c_FreqHz_t        busFreq       = 0u;
    i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_CNT;
    i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_CNT;
    i2c_AddrMode_t      addrMode      = I2C_ADDR_MODE_CNT;
    i2c_FlagState_t     periphState   = I2C_FLAG_INACTIVE;
    rcc_PeriphId_t      clkSrc        = RCC_PERIPH_ID_CNT;

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

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

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphClkSrc( IT_I2C_RCC_PCLK, &clkSrc ) );
    TEST_ASSERT_EQUAL( IT_I2C_RCC_PCLK, clkSrc );
    TEST_ASSERT_EQUAL_HEX32( 0u, SYSCFG->PMC & IT_I2C_FMP_BIT );
}


/**
 * \brief   I2c_Init() on target in fast mode - bus frequency read back.
 *
 * \details Initializes the master I2C1 with 400 kHz and reads the bus frequency.
 *
 * \par Expected results
 * - Bus frequency 400 kHz +- 10 %.
 */
void It_I2c_Init_FastMode_BusFreqReadBack( void )
{
    i2c_FreqHz_t busFreq = 0u;

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_HZ * IT_I2C_FREQ_TOL_PCT ) / 100u, IT_I2C_FREQ_FAST_HZ, busFreq );
}


/**
 * \brief   Fast-mode Plus enables the 20 mA drive in SYSCFG, I2c_Deinit() releases it.
 *
 * \details Initializes the master I2C1 with 1 MHz (PCLK1 kernel clock), reads the bus frequency
 *          and SYSCFG_PMC.I2C1_FMP, then deinitializes the peripheral.
 *
 * \par Expected results
 * - Bus frequency 1 MHz +- 10 %, I2C1_FMP = 1.
 * - After I2c_Deinit(): I2C1_FMP = 0 (SYSCFG is not reset with the I2C).
 */
void It_I2c_Init_FastModePlus_FmpDriveEnabledAndReleased( void )
{
    i2c_FreqHz_t busFreq = 0u;

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_PLUS_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_PLUS_HZ * IT_I2C_FREQ_TOL_PCT ) / 100u, IT_I2C_FREQ_FAST_PLUS_HZ, busFreq );
    TEST_ASSERT_EQUAL_HEX32( IT_I2C_FMP_BIT, SYSCFG->PMC & IT_I2C_FMP_BIT );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( IT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, SYSCFG->PMC & IT_I2C_FMP_BIT );
}


/**
 * \brief   HSI kernel clock - Fast-mode is accepted, Fast-mode Plus is refused.
 *
 * \details Initializes the master I2C1 with HSI kernel clock (16 MHz) and 400 kHz, reads the
 *          bus frequency, then initializes it with 1 MHz.
 *
 * \note    Device errata ES0334 ("Wrong data sampling when data setup time (tSU;DAT) is
 *          shorter than one I2C kernel clock period") - Fast-mode Plus requires
 *          I2CCLK >= 20 MHz, Fast-mode I2CCLK >= 10 MHz. APB1 54 MHz / HSI 16 MHz ratio
 *          3.375 is outside of the stall window 1.5 - 3 of ES0334.
 *
 * \par Expected results
 * - 400 kHz: I2C_REQUEST_OK, bus frequency 400 kHz +- 10 %.
 * - 1 MHz: I2C_REQUEST_ERROR.
 */
void It_I2c_Init_HsiKernelClock_FastModePlusRefused( void )
{
    i2c_Config_t config;
    i2c_FreqHz_t busFreq = 0u;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = IT_I2C_PERIPH;
    config.ClkSrc   = I2C_CLK_SRC_HSI;
    config.BusFreq  = IT_I2C_FREQ_FAST_HZ;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_HZ * IT_I2C_FREQ_TOL_PCT ) / 100u, IT_I2C_FREQ_FAST_HZ, busFreq );

    config.BusFreq = IT_I2C_FREQ_FAST_PLUS_HZ;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
}


/**
 * \brief   Re-initialization with another kernel clock source changes the clock.
 *
 * \details Initializes the master I2C1 with PCLK1 kernel clock, then with HSI kernel clock
 *          without I2c_Deinit() and reads the kernel clock source from RCC.
 *
 * \note    Regression test of STM32H5 module bug AB#1098 - RCC changes the kernel
 *          clock multiplexer only of a released clock.
 *
 * \par Expected results
 * - Both initializations I2C_REQUEST_OK.
 * - Kernel clock source is HSI, bus frequency 100 kHz +- 10 %.
 */
void It_I2c_Init_ReInitOtherClockSource_KernelClockChanged( void )
{
    i2c_Config_t   config;
    i2c_FreqHz_t   busFreq = 0u;
    rcc_PeriphId_t clkSrc  = RCC_PERIPH_ID_CNT;

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = IT_I2C_PERIPH;
    config.ClkSrc   = I2C_CLK_SRC_HSI;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphClkSrc( IT_I2C_RCC_PCLK, &clkSrc ) );
    TEST_ASSERT_EQUAL( IT_I2C_RCC_HSI, clkSrc );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_STANDARD_HZ * IT_I2C_FREQ_TOL_PCT ) / 100u, IT_I2C_FREQ_STANDARD_HZ, busFreq );
}


/**
 * \brief   Idle bus with pull-ups is reported free.
 *
 * \details Initializes the master I2C1 with internal pull-ups and reads bus state.
 *
 * \par Expected results
 * - Bus busy flag is INACTIVE.
 */
void It_I2c_Get_BusState_IdleBusWithPullUps_BusFree( void )
{
    i2c_FlagState_t busBusy = I2C_FLAG_ACTIVE;

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

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
 * \details Initializes the master I2C1 in DMA mode, deinitializes it and tries to start an
 *          address-only transfer.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, peripheral state INACTIVE.
 * - Transfer start is rejected (I2C_REQUEST_ERROR).
 */
void It_I2c_Deinit_InitializedPeripheral_PeripheralInactive( void )
{
    i2c_FlagState_t   periphState = I2C_FLAG_ACTIVE;
    i2c_XferRequest_t request     = { IT_I2C_ABSENT_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };

    ItI2c_Init( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( IT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_PeriphState( IT_I2C_PERIPH, &periphState ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, periphState );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
}

/*--------------------------- Transfers (NACK) -------------------------------*/

/**
 * \brief   Polling address-only transfer to absent slave reports NACK.
 *
 * \details Polling mode, address-only transfer to 0x50 (no device), task polled
 *          until the transfer ends.
 *
 * \par Expected results
 * - Error callback once with I2C_XFER_ERROR_NACK, no complete callback, bus free.
 */
void It_I2c_Set_XferStart_PollAddressOnlyAbsent_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );
}


/**
 * \brief   ISR mode write to absent slave reports NACK.
 *
 * \details ISR mode, 2 byte write to 0x50 at 400 kHz.
 *
 * \par Expected results
 * - Error callback once with I2C_XFER_ERROR_NACK, no complete callback, bus free.
 */
void It_I2c_Set_XferStart_IsrWriteAbsent_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR, itI2c_TxData, 2u, I2C_NULL_PTR, 0u };

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_FAST_HZ );

    ItI2c_Check_NackTransfer( &request );
}


/**
 * \brief   ISR mode write + read to absent slave reports NACK of the write phase.
 *
 * \details ISR mode, 1 byte write + 2 byte read from 0x50.
 *
 * \par Expected results
 * - Error callback once with I2C_XFER_ERROR_NACK, no complete callback, bus free.
 */
void It_I2c_Set_XferStart_IsrWriteReadAbsent_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR, itI2c_TxData, 1u, itI2c_RxBuf, 2u };

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );
}


/**
 * \brief   DMA mode read from absent slave reports NACK.
 *
 * \details DMA mode, 4 byte read from 0x50.
 *
 * \par Expected results
 * - Error callback once with I2C_XFER_ERROR_NACK, no complete callback, bus free.
 */
void It_I2c_Set_XferStart_DmaReadAbsent_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR, I2C_NULL_PTR, 0u, itI2c_RxBuf, 4u };

    ItI2c_Init( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );
}


/**
 * \brief   10-bit addressed write to absent slave reports NACK of the header.
 *
 * \details ISR mode, 10-bit addressing, 1 byte write to 0x2A5.
 *
 * \par Expected results
 * - Error callback once with I2C_XFER_ERROR_NACK, no complete callback, bus free.
 */
void It_I2c_Set_XferStart_Isr10BitAbsent_NackReported( void )
{
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR10, itI2c_TxData, 1u, I2C_NULL_PTR, 0u };

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( IT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );

    ItI2c_Check_NackTransfer( &request );
}


/**
 * \brief   Transfer after NACK can be started and reports NACK again.
 *
 * \details ISR mode, 2 byte write to 0x50 twice.
 *
 * \par Expected results
 * - Both transfers: NACK error reported once, no complete callback, bus free
 *   (STOP of the first transfer is finished before the second START).
 */
void It_I2c_Set_XferStart_RepeatedAfterNack_NackReportedAgain( void )
{
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR, itI2c_TxData, 2u, I2C_NULL_PTR, 0u };

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );

    itI2c_Master.ErrorCnt = 0u;
    itI2c_Master.ErrorId  = I2C_XFER_ERROR_CNT;
    ItI2c_Check_NackTransfer( &request );
}


/**
 * \brief   Transfer after NACK of the first 10-bit address byte can be started.
 *
 * \details ISR mode, 10-bit addressing, 1 byte write to absent 0x2A5 twice.
 *
 * \note    Device errata bug AB#1147 (ES0334 "10-bit master mode: new transfer cannot be
 *          launched if first part of the address is not acknowledged by the slave") - START
 *          left pending by the NACK is released by the module (PE toggled after STOPF).
 *
 * \par Expected results
 * - Both transfers: NACK error reported once, no complete callback, bus free (the second
 *   transfer is not blocked by the START of the first one).
 */
void It_I2c_Set_XferStart_Repeated10BitAfterNack_NackReportedAgain( void )
{
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR10, itI2c_TxData, 1u, I2C_NULL_PTR, 0u };

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( IT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );

    ItI2c_Check_NackTransfer( &request );

    itI2c_Master.ErrorCnt = 0u;
    itI2c_Master.ErrorId  = I2C_XFER_ERROR_CNT;
    ItI2c_Check_NackTransfer( &request );
}

/*------------------------- Transfers with slave -----------------------------*/

/**
 * \brief   Address-only transfer to the slave completes in all modes.
 *
 * \details Address-only transfer to the register device (slave of the module, interrupt
 *          mode) in POLL, ISR and DMA mode of the master.
 *
 * \par Expected results
 * - Complete callback for every transfer, slave counted 3 address matches.
 */
void It_I2c_Set_XferStart_AddressOnlyAllModes_Completed( void )
{
    const i2c_XferMode_t    modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_XferRequest_t request    = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };

    ItI2c_Init_RegSlave( I2C_ADDR_MODE_7BIT, IT_I2C_FREQ_STANDARD_HZ );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        ItI2c_Check_Xfer( &request );
    }

    TEST_ASSERT_EQUAL_UINT32( 3u, itI2c_Slave.AddrCnt );
}


/**
 * \brief   Write and read back of 1, 2, 3, 5 and 16 bytes in all modes (100 kHz).
 *
 * \details For every mode (POLL, ISR, DMA) and count of bytes: register address +
 *          data are written, the register file is checked, then register address
 *          write + repeated START + read returns the data.
 *
 * \par Expected results
 * - Every transfer completes, registers and read data equal the written data
 *   (DMA channels are re-armed for every transfer).
 */
void It_I2c_Set_XferStart_WriteReadStandardModeAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 3u, 5u, IT_I2C_DATA_MAX };

    ItI2c_Init_RegSlave( I2C_ADDR_MODE_7BIT, IT_I2C_FREQ_STANDARD_HZ );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            ItI2c_Check_WriteRead( IT_I2C_SLAVE_ADDR, (uint8_t)( modeIdx + cntIdx ), cntLut[ cntIdx ] );
        }
    }
}


/**
 * \brief   Write and read back in all modes in Fast-mode (400 kHz).
 *
 * \details Same as the Standard-mode test with 1, 2, 3 and 16 bytes at 400 kHz.
 *
 * \par Expected results
 * - Every transfer completes, registers and read data equal the written data.
 */
void It_I2c_Set_XferStart_WriteReadFastModeAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 3u, IT_I2C_DATA_MAX };

    ItI2c_Init_RegSlave( I2C_ADDR_MODE_7BIT, IT_I2C_FREQ_FAST_HZ );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_FAST_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            ItI2c_Check_WriteRead( IT_I2C_SLAVE_ADDR, (uint8_t)( 8u + modeIdx + cntIdx ), cntLut[ cntIdx ] );
        }
    }
}


/**
 * \brief   Read-only transfers (no repeated START) return data from the selected
 *          register in all modes.
 *
 * \details Registers are preset, register 4 is selected by a 1 byte write, then 1, 2
 *          and 6 bytes are read by read-only transfers, the next register is selected
 *          before every read in POLL, ISR and DMA mode.
 *
 * \par Expected results
 * - Read data equal consecutive registers 4, 5 - 6, 7 - 12.
 */
void It_I2c_Set_XferStart_ReadOnlyAllModes_ConsecutiveRegisters( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 6u };

    ItI2c_Init_RegSlave( I2C_ADDR_MODE_7BIT, IT_I2C_FREQ_STANDARD_HZ );

    for( uint32_t regIdx = 0u; IT_I2C_REG_CNT > regIdx; regIdx++ )
    {
        itI2c_Regs[ regIdx ] = (i2c_Data_t)( 0xC0u + regIdx );
    }

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        const i2c_XferRequest_t selRequest = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, 1u, I2C_NULL_PTR, 0u };
        uint8_t                 regAddr    = 4u;

        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            const i2c_XferRequest_t readRequest = { IT_I2C_SLAVE_ADDR,
                                                    I2C_NULL_PTR,
                                                    0u,
                                                    itI2c_RxBuf,
                                                    cntLut[ cntIdx ] };

            itI2c_TxBuf[ 0u ] = regAddr;
            ItI2c_Check_Xfer( &selRequest );

            ItI2c_Check_Xfer( &readRequest );

            for( uint32_t byteIdx = 0u; cntLut[ cntIdx ] > byteIdx; byteIdx++ )
            {
                TEST_ASSERT_EQUAL_HEX8( 0xC0u + regAddr + byteIdx, itI2c_RxBuf[ byteIdx ] );
            }

            regAddr += (uint8_t)cntLut[ cntIdx ];
        }
    }
}


/**
 * \brief   10-bit addressing: write, write + read and read-only transfers.
 *
 * \details Slave with 10-bit address 0x1B7, master in 10-bit mode: write + read
 *          back of 3 bytes and read-only transfer of 2 bytes (complete 10-bit read
 *          sequence - address in write direction, repeated START, read header) in
 *          POLL, ISR and DMA mode.
 *
 * \par Expected results
 * - Every transfer completes, read data equal the registers.
 */
void It_I2c_Set_XferStart_10BitAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };

    ItI2c_Init_RegSlave( I2C_ADDR_MODE_10BIT, IT_I2C_FREQ_STANDARD_HZ );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        const i2c_XferRequest_t readRequest = { IT_I2C_SLAVE_ADDR10, I2C_NULL_PTR, 0u, itI2c_RxBuf, 2u };
        const uint8_t           regAddr     = (uint8_t)( 20u + modeIdx );

        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( IT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );

        ItI2c_Check_WriteRead( IT_I2C_SLAVE_ADDR10, regAddr, 3u );

        /* Read-only transfer starts at the register selected here */
        itI2c_SlaveRx[ 0u ] = regAddr;
        ItI2c_Check_Xfer( &readRequest );
        TEST_ASSERT_EQUAL_HEX8( itI2c_Regs[ regAddr ],      itI2c_RxBuf[ 0u ] );
        TEST_ASSERT_EQUAL_HEX8( itI2c_Regs[ regAddr + 1u ], itI2c_RxBuf[ 1u ] );
    }
}


/**
 * \brief   Transfers longer than 255 bytes use NBYTES reload in all modes.
 *
 * \details Write of register address + 300 bytes, then register address write +
 *          repeated START + read of 300 bytes in POLL, ISR and DMA mode (400 kHz).
 *
 * \par Expected results
 * - Every transfer completes, registers contain the written values, read data
 *   equal the written data (all 300 bytes).
 */
void It_I2c_Set_XferStart_LongTransferAllModes_ReloadChunks( void )
{
    const i2c_XferMode_t    modeLut[ ]   = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_XferRequest_t writeRequest = { IT_I2C_SLAVE_ADDR,
                                             itI2c_TxBuf,
                                             (i2c_DataCnt_t)( IT_I2C_LONG_SIZE + 1u ),
                                             I2C_NULL_PTR,
                                             0u };
    const i2c_XferRequest_t readRequest  = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, 1u, itI2c_RxBuf, IT_I2C_LONG_SIZE };

    ItI2c_Init_RegSlave( I2C_ADDR_MODE_7BIT, IT_I2C_FREQ_FAST_HZ );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_FAST_HZ );

        itI2c_TxBuf[ 0u ] = 0u;

        for( uint32_t byteIdx = 0u; IT_I2C_LONG_SIZE > byteIdx; byteIdx++ )
        {
            itI2c_TxBuf[ byteIdx + 1u ] = (i2c_Data_t)( 0xA5u ^ byteIdx ^ ( modeIdx << 6u ) );
            itI2c_RxBuf[ byteIdx ]      = 0u;
        }

        ItI2c_Check_Xfer( &writeRequest );

        for( uint32_t regIdx = 0u; IT_I2C_REG_CNT > regIdx; regIdx++ )
        {
            TEST_ASSERT_EQUAL_HEX8_MESSAGE( itI2c_TxBuf[ regIdx + 1u ], itI2c_Regs[ regIdx ], "Register" );
        }

        ItI2c_Check_Xfer( &readRequest );

        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE( &itI2c_TxBuf[ 1u ], itI2c_RxBuf, IT_I2C_LONG_SIZE, "Read back data" );
    }
}

/*-------------------------- Slave (module) as tested unit -------------------*/

/**
 * \brief   Master write is received by the slave in ISR mode.
 *
 * \details Master (ISR, 100 kHz) writes 8 bytes to own address 1 of the slave
 *          (ISR mode).
 *
 * \par Expected results
 * - Master transfer complete without error.
 * - Slave: one address match (write, own address 1), one transfer complete with
 *   8 received bytes equal to the written bytes, no byte beyond the buffer.
 */
void It_I2c_Slave_IsrWrite_DataReceived( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_SlaveTx, IT_I2C_LONG_SIZE };
    const i2c_XferRequest_t request     = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    ItI2c_Set_Pattern( itI2c_TxBuf, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itI2c_Slave.AddrCnt );
    TEST_ASSERT_EQUAL( I2C_SLAVE_DIR_WRITE, itI2c_Slave.LastDir );
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_SLAVE_ADDR, itI2c_Slave.LastAddr );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_TxBuf, itI2c_SlaveRx, IT_I2C_SHORT_SIZE );
}


/**
 * \brief   Register read of the master is answered by the polling slave.
 *
 * \details Slave (POLL mode, I2c_Task) emulates a register device: the register
 *          address received in the write phase selects the transmit buffer in the
 *          address callback of the read phase. Master (ISR) writes register address
 *          5, repeated START, reads 4 bytes.
 *
 * \par Expected results
 * - Master transfer complete, read bytes equal registers 5 - 8.
 * - Slave: two address matches (write, read), result 1 received byte and 4
 *   transmitted bytes, nothing beyond the buffers.
 */
void It_I2c_Slave_PollRegisterRead_RegistersTransmitted( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_Data_t        regAddr     = IT_I2C_REG_ADDR;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_Regs, IT_I2C_REG_CNT };
    const i2c_XferRequest_t request     = { IT_I2C_SLAVE_ADDR, &regAddr, 1u, itI2c_RxBuf, IT_I2C_REG_READ_CNT };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    ItI2c_Set_Pattern( itI2c_Regs, IT_I2C_REG_CNT, IT_I2C_SEED_REGS );
    itI2c_Slave.RegMode = I2C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( 1u, IT_I2C_REG_READ_CNT );
    TEST_ASSERT_EQUAL_UINT32( IT_I2C_REG_READ_ADDR_CNT, itI2c_Slave.AddrCnt );
    TEST_ASSERT_EQUAL( I2C_SLAVE_DIR_READ, itI2c_Slave.LastDir );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itI2c_Regs[ IT_I2C_REG_ADDR ], itI2c_RxBuf, IT_I2C_REG_READ_CNT );
}


/**
 * \brief   Register read of the master is answered by the slave in DMA mode.
 *
 * \details As the polling register read, slave in DMA mode (DMA1 streams 7 / 2),
 *          master in polling mode (I2c_Task).
 *
 * \par Expected results
 * - Master transfer complete, read bytes equal registers 5 - 8.
 * - Slave: two address matches, result 1 received byte and 4 transmitted bytes.
 */
void It_I2c_Slave_DmaRegisterRead_RegistersTransmitted( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_Data_t        regAddr     = IT_I2C_REG_ADDR;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_Regs, IT_I2C_REG_CNT };
    const i2c_XferRequest_t request     = { IT_I2C_SLAVE_ADDR, &regAddr, 1u, itI2c_RxBuf, IT_I2C_REG_READ_CNT };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );
    ItI2c_Set_Pattern( itI2c_Regs, IT_I2C_REG_CNT, IT_I2C_SEED_REGS );
    itI2c_Slave.RegMode = I2C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( 1u, IT_I2C_REG_READ_CNT );
    TEST_ASSERT_EQUAL_UINT32( IT_I2C_REG_READ_ADDR_CNT, itI2c_Slave.AddrCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itI2c_Regs[ IT_I2C_REG_ADDR ], itI2c_RxBuf, IT_I2C_REG_READ_CNT );
}


/**
 * \brief   Long write and read (more than 255 bytes) between master DMA and slave DMA.
 *
 * \details Master (DMA, 400 kHz) writes 300 bytes to the slave (DMA), then reads
 *          300 bytes of the transmit buffer of the slave (two transfers).
 *
 * \par Expected results
 * - Both master transfers complete without error (NBYTES reload of the master).
 * - Slave received the 300 written bytes; the master read the 300 bytes of the slave.
 * - Slave results: 300 received bytes (first transfer), 300 transmitted bytes (second).
 */
void It_I2c_Slave_DmaLongWriteRead_DataExchanged( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_SlaveTx, IT_I2C_LONG_SIZE };
    const i2c_XferRequest_t writeReq    = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, IT_I2C_LONG_SIZE, I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t readReq     = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, itI2c_RxBuf, IT_I2C_LONG_SIZE };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_DMA, IT_I2C_FREQ_FAST_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_DMA, IT_I2C_FREQ_FAST_HZ );
    ItI2c_Set_Pattern( itI2c_TxBuf, IT_I2C_LONG_SIZE, IT_I2C_SEED_MASTER );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_LONG_SIZE, IT_I2C_SEED_SLAVE );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &writeReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_LONG_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_TxBuf, itI2c_SlaveRx, IT_I2C_LONG_SIZE );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &readReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( 0u, IT_I2C_LONG_SIZE );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_SlaveTx, itI2c_RxBuf, IT_I2C_LONG_SIZE );
}


/**
 * \brief   Read beyond the transmit buffer of the slave (ISR mode) gets the fill byte.
 *
 * \details Slave (ISR) transmit buffer of 2 bytes, fill byte 0xA5; master (POLL)
 *          reads 6 bytes.
 *
 * \par Expected results
 * - Master read: 2 bytes of the buffer, then 4 fill bytes.
 * - Slave result: 2 transmitted bytes, read beyond the buffer reported.
 */
void It_I2c_Slave_IsrReadBeyondBuffer_FillByteSent( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    i2c_Data_t              expected[ IT_I2C_BEYOND_READ_CNT ];
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_SlaveTx, IT_I2C_TX_PART_SIZE };
    const i2c_XferRequest_t request     = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, itI2c_RxBuf, IT_I2C_BEYOND_READ_CNT };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.FillByte = IT_I2C_FILL_BYTE;
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_TX_PART_SIZE, IT_I2C_SEED_SLAVE );

    for( uint32_t byteIdx = 0u; IT_I2C_BEYOND_READ_CNT > byteIdx; byteIdx++ )
    {
        expected[ byteIdx ] = IT_I2C_FILL_BYTE;
    }

    expected[ 0u ] = itI2c_SlaveTx[ 0u ];
    expected[ 1u ] = itI2c_SlaveTx[ 1u ];

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX8_ARRAY( expected, itI2c_RxBuf, IT_I2C_BEYOND_READ_CNT );
    TEST_ASSERT_EQUAL_UINT16( IT_I2C_TX_PART_SIZE, itI2c_Slave.Result.TxCnt );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, itI2c_Slave.Result.TxBeyondBuffer );
}


/**
 * \brief   Read and write beyond the buffers of the slave in DMA mode.
 *
 * \details Slave (DMA) transmit buffer 2 bytes, receive buffer 4 bytes, fill byte
 *          0xA5; master (ISR) reads 6 bytes, then writes 8 bytes.
 *
 * \par Expected results
 * - Master read: 2 bytes of the buffer, then 4 fill bytes (DMA transfer complete
 *   switches the slave to the interrupt); slave result 2 transmitted bytes, read
 *   beyond the buffer reported.
 * - Master write complete (bytes beyond the buffer acknowledged); slave received the
 *   first 4 bytes, result 4 received bytes, write beyond the buffer reported.
 */
void It_I2c_Slave_DmaBeyondBuffers_FillAndDiscard( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_RX_PART_SIZE, itI2c_SlaveTx, IT_I2C_TX_PART_SIZE };
    const i2c_XferRequest_t readReq     = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, itI2c_RxBuf, IT_I2C_BEYOND_READ_CNT };
    const i2c_XferRequest_t writeReq    = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.FillByte = IT_I2C_FILL_BYTE;
    ItI2c_Init_Slave( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_TX_PART_SIZE, IT_I2C_SEED_SLAVE );
    ItI2c_Set_Pattern( itI2c_TxBuf, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &readReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_SlaveTx, itI2c_RxBuf, IT_I2C_TX_PART_SIZE );

    for( uint32_t byteIdx = IT_I2C_TX_PART_SIZE; IT_I2C_BEYOND_READ_CNT > byteIdx; byteIdx++ )
    {
        TEST_ASSERT_EQUAL_HEX8( IT_I2C_FILL_BYTE, itI2c_RxBuf[ byteIdx ] );
    }

    TEST_ASSERT_EQUAL_UINT16( IT_I2C_TX_PART_SIZE, itI2c_Slave.Result.TxCnt );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, itI2c_Slave.Result.TxBeyondBuffer );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &writeReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_TxBuf, itI2c_SlaveRx, IT_I2C_RX_PART_SIZE );
    TEST_ASSERT_EQUAL_UINT16( IT_I2C_RX_PART_SIZE, itI2c_Slave.Result.RxCnt );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, itI2c_Slave.Result.RxBeyondBuffer );
}


/**
 * \brief   10-bit addressing between master and slave.
 *
 * \details Slave (ISR) with 10-bit own address 0x1B7; master (ISR, 10-bit mode)
 *          writes 8 bytes, then reads 8 bytes.
 *
 * \par Expected results
 * - Both master transfers complete, data exchanged.
 * - Slave reports matched address 0x1B7 (10-bit header decoded).
 */
void It_I2c_Slave_Isr10BitAddress_DataExchanged( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_SlaveTx, IT_I2C_LONG_SIZE };
    const i2c_XferRequest_t writeReq    = { IT_I2C_SLAVE_ADDR10, itI2c_TxBuf, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t readReq     = { IT_I2C_SLAVE_ADDR10, I2C_NULL_PTR, 0u, itI2c_RxBuf, IT_I2C_SHORT_SIZE };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.OwnAddr     = IT_I2C_SLAVE_ADDR10;
    slaveConfig.OwnAddrMode = I2C_ADDR_MODE_10BIT;
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_10BIT );
    ItI2c_Set_Pattern( itI2c_TxBuf, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_SHORT_SIZE, IT_I2C_SEED_SLAVE );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &writeReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_SLAVE_ADDR10, itI2c_Slave.Result.MatchedAddr );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_TxBuf, itI2c_SlaveRx, IT_I2C_SHORT_SIZE );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &readReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_SLAVE_ADDR10, itI2c_Slave.Result.MatchedAddr );
    TEST_ASSERT_EQUAL_UINT16( IT_I2C_SHORT_SIZE, itI2c_Slave.Result.TxCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_SlaveTx, itI2c_RxBuf, IT_I2C_SHORT_SIZE );
}


/**
 * \brief   Own address 2 and general call of the slave are answered.
 *
 * \details Slave (ISR) with own address 2 0x3A and general call enabled; master
 *          (ISR) writes 8 bytes to 0x3A, then 8 bytes to the general call address.
 *
 * \par Expected results
 * - Both master transfers complete.
 * - Slave reports matched address 0x3A, then 0x00, the bytes are received.
 */
void It_I2c_Slave_OwnAddr2GeneralCall_Answered( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_SlaveTx, IT_I2C_LONG_SIZE };
    const i2c_XferRequest_t addr2Req    = { IT_I2C_SLAVE_ADDR2, itI2c_TxBuf, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t gcReq       = { I2C_SLAVE_ADDR_GENERAL_CALL,
                                            itI2c_TxBuf,
                                            IT_I2C_SHORT_SIZE,
                                            I2C_NULL_PTR,
                                            0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.OwnAddr2State = I2C_FUNCTION_ACTIVE;
    slaveConfig.OwnAddr2      = IT_I2C_SLAVE_ADDR2;
    slaveConfig.GeneralCall   = I2C_FUNCTION_ACTIVE;
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    ItI2c_Set_Pattern( itI2c_TxBuf, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &addr2Req ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_SLAVE_ADDR2, itI2c_Slave.Result.MatchedAddr );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &gcReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX16( I2C_SLAVE_ADDR_GENERAL_CALL, itI2c_Slave.Result.MatchedAddr );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_TxBuf, itI2c_SlaveRx, IT_I2C_SHORT_SIZE );
}


/**
 * \brief   A slave which does not listen does not acknowledge its address.
 *
 * \details Slave initialized (ISR) but not listening: master write gets NACK. The
 *          slave listens: the write completes. The slave stops listening: NACK again.
 *
 * \par Expected results
 * - First and last master transfer: NACK, no slave callback.
 * - Second master transfer complete, one slave transfer complete.
 * - Listen state reported accordingly.
 */
void It_I2c_Slave_ListenStartStop_AddressAcknowledgedOnlyWhileListening( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    i2c_FunctionState_t     listenState = I2C_FUNCTION_INACTIVE;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_SlaveTx, IT_I2C_LONG_SIZE };
    const i2c_XferRequest_t request     = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );
    TEST_ASSERT_EQUAL_UINT32( 0u, itI2c_Slave.AddrCnt );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_ListenState( IT_I2C_SLAVE_PERIPH, &listenState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_ACTIVE, listenState );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStop( IT_I2C_SLAVE_PERIPH ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_ListenState( IT_I2C_SLAVE_PERIPH, &listenState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, listenState );
    ItI2c_Check_NackTransfer( &request );
    TEST_ASSERT_EQUAL_UINT32( 0u, itI2c_Slave.AddrCnt );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes the master I2C1 on board pins with data handling (PCLK1 kernel clock,
 *        7-bit addressing).
 *
 * \param xferMode [in]: Data transfer mode
 * \param busFreq  [in]: SCL frequency [Hz]
 */
static void ItI2c_Init( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq )
{
    ItI2c_Init_Master( xferMode, busFreq, I2C_ADDR_MODE_7BIT );
}


/**
 * \brief Initializes the master I2C1 on board pins with internal pull-ups and data handling
 *        (PCLK1 kernel clock).
 *
 * \param xferMode [in]: Data transfer mode
 * \param busFreq  [in]: SCL frequency [Hz]
 * \param addrMode [in]: Addressing mode of the master
 */
static void ItI2c_Init_Master( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq, i2c_AddrMode_t addrMode )
{
    i2c_Config_t config;

    itI2c_DataConfig.XferMode                  = xferMode;
    itI2c_DataConfig.TxDma                     = IT_I2C_DMA_TX;
    itI2c_DataConfig.TxDmaPriority             = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.RxDma                     = IT_I2C_DMA_RX;
    itI2c_DataConfig.RxDmaPriority             = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.IrqPriority               = IT_I2C_IRQ_PRIO;
    itI2c_DataConfig.XferCompleteCallback      = ItI2c_XferCompleteCallback;
    itI2c_DataConfig.ErrorCallback             = ItI2c_ErrorCallback;
    itI2c_DataConfig.SlaveAddrCallback         = I2C_NULL_PTR;
    itI2c_DataConfig.SlaveXferCompleteCallback = I2C_NULL_PTR;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId   = IT_I2C_PERIPH;
    config.BusFreq    = busFreq;
    config.AddrMode   = addrMode;
    config.DataConfig = &itI2c_DataConfig;
    config.SclPin     = IT_I2C_SCL_PIN;
    config.SdaPin     = IT_I2C_SDA_PIN;
    config.PinPull    = I2C_PIN_PULL_UP;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
}


/**
 * \brief Initializes the slave I2C peripheral of the board with data handling (the pins of the
 *        slave have the external pull-up resistors of the bus).
 *
 * \param xferMode    [in]: Data transfer mode
 * \param busFreq     [in]: SCL frequency of the master [Hz]
 * \param slaveConfig [in]: Slave configuration
 */
static void ItI2c_Init_Slave( i2c_XferMode_t xferMode,
                              i2c_FreqHz_t busFreq,
                              const i2c_SlaveConfig_t * const slaveConfig )
{
    i2c_Config_t config;

    itI2c_SlaveDataConfig.XferMode                  = xferMode;
    itI2c_SlaveDataConfig.TxDma                     = IT_I2C_SLAVE_DMA_TX;
    itI2c_SlaveDataConfig.TxDmaPriority             = I2C_DMA_PRIORITY_HIGH;
    itI2c_SlaveDataConfig.RxDma                     = IT_I2C_SLAVE_DMA_RX;
    itI2c_SlaveDataConfig.RxDmaPriority             = I2C_DMA_PRIORITY_HIGH;
    itI2c_SlaveDataConfig.IrqPriority               = IT_I2C_SLAVE_IRQ_PRIO;
    itI2c_SlaveDataConfig.XferCompleteCallback      = I2C_NULL_PTR;
    itI2c_SlaveDataConfig.ErrorCallback             = ItI2c_SlaveErrorCallback;
    itI2c_SlaveDataConfig.SlaveAddrCallback         = ItI2c_SlaveAddrCallback;
    itI2c_SlaveDataConfig.SlaveXferCompleteCallback = ItI2c_SlaveCompleteCallback;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId    = IT_I2C_SLAVE_PERIPH;
    config.BusFreq     = busFreq;
    config.Role        = I2C_ROLE_SLAVE;
    config.SlaveConfig = *slaveConfig;
    config.DataConfig  = &itI2c_SlaveDataConfig;
    config.SclPin      = IT_I2C_SLAVE_SCL_PIN;
    config.SdaPin      = IT_I2C_SLAVE_SDA_PIN;
    config.PinPull     = I2C_PIN_PULL_NONE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
}


/**
 * \brief Initializes the slave as register device (interrupt mode, listening): the first written
 *        byte selects the register, following bytes are stored, read bytes are returned from the
 *        selected register.
 *
 * \note  The test is ignored on boards without the slave wiring.
 *
 * \param addrMode [in]: Own address mode of the slave (7-bit / 10-bit)
 * \param busFreq  [in]: SCL frequency of the master [Hz]
 */
static void ItI2c_Init_RegSlave( i2c_AddrMode_t addrMode, i2c_FreqHz_t busFreq )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_SLAVE_RX_SIZE, itI2c_Regs, IT_I2C_REG_CNT };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );

    if( I2C_ADDR_MODE_10BIT == addrMode )
    {
        slaveConfig.OwnAddr     = IT_I2C_SLAVE_ADDR10;
        slaveConfig.OwnAddrMode = I2C_ADDR_MODE_10BIT;
    }
    else
    {
        /* Own address 1 is the 7-bit default address */
    }

    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, busFreq, &slaveConfig );
    itI2c_Slave.RegMode = I2C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
}


/**
 * \brief Default slave configuration of the tests: own address 1 0x2C (7-bit).
 *
 * \param slaveConfig [out]: Slave configuration
 */
static void ItI2c_Get_SlaveConfig( i2c_SlaveConfig_t * const slaveConfig )
{
    i2c_Config_t config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );

    *slaveConfig         = config.SlaveConfig;
    slaveConfig->OwnAddr = IT_I2C_SLAVE_ADDR;
}


/**
 * \brief Waits until a master callback is called or timeout (I2c_Task called - polling mode).
 */
static void ItI2c_Wait_XferEnd( void )
{
    uint32_t masterDone = 0u;

    for( uint32_t loopIdx = 0u;
         ( IT_I2C_WAIT_LOOPS > loopIdx ) &&
         ( 0u == masterDone );
         loopIdx++ )
    {
        I2c_Task();
        masterDone = itI2c_Master.CompleteCnt + itI2c_Master.ErrorCnt;
    }
}


/**
 * \brief Waits until the master and the slave transfer ended or timeout (I2c_Task called).
 */
static void ItI2c_Wait_MasterSlaveEnd( void )
{
    uint32_t masterDone = 0u;
    uint32_t slaveDone  = 0u;

    for( uint32_t loopIdx = 0u;
         ( IT_I2C_WAIT_LOOPS > loopIdx ) &&
         ( ( 0u == masterDone ) ||
           ( 0u == slaveDone  )    );
         loopIdx++ )
    {
        I2c_Task();
        masterDone = itI2c_Master.CompleteCnt + itI2c_Master.ErrorCnt;
        slaveDone  = itI2c_SlaveCb.CompleteCnt + itI2c_SlaveCb.ErrorCnt;
    }
}


/**
 * \brief Executes a transfer to the slave of the module and checks its successful end (master
 *        and slave).
 *
 * \param xferRequest [in]: Transfer request
 */
static void ItI2c_Check_Xfer( const i2c_XferRequest_t * const xferRequest )
{
    i2c_FunctionState_t xferState = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t   xferError = I2C_XFER_ERROR_CNT;

    itI2c_Master.CompleteCnt  = 0u;
    itI2c_Master.ErrorCnt     = 0u;
    itI2c_Master.ErrorId      = I2C_XFER_ERROR_CNT;
    itI2c_SlaveCb.CompleteCnt = 0u;
    itI2c_SlaveCb.ErrorCnt    = 0u;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, xferRequest ) );
    ItI2c_Wait_MasterSlaveEnd();

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 0u, itI2c_Master.ErrorCnt, "Transfer error reported" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_Master.CompleteCnt, "Transfer not completed" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 0u, itI2c_SlaveCb.ErrorCnt, "Slave transfer error" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_SlaveCb.CompleteCnt, "Slave transfer not completed" );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( IT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NONE, xferError );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( IT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
}


/**
 * \brief Starts transfer to not present slave and checks NACK error and free bus.
 *
 * \param xferRequest [in]: Transfer request
 */
static void ItI2c_Check_NackTransfer( const i2c_XferRequest_t * const xferRequest )
{
    i2c_FunctionState_t xferState = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t   xferError = I2C_XFER_ERROR_NONE;
    i2c_FlagState_t     busBusy   = I2C_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, xferRequest ) );
    ItI2c_Wait_XferEnd();

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_Master.ErrorCnt, "NACK error not reported" );
    TEST_ASSERT_EQUAL_UINT32( 0u, itI2c_Master.CompleteCnt );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, itI2c_Master.ErrorId );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferError( IT_I2C_PERIPH, &xferError ) );
    TEST_ASSERT_EQUAL( I2C_XFER_ERROR_NACK, xferError );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( IT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );

    /* STOP condition is sent by the hardware after NACK - bus is released once it is sent */
    for( uint32_t loopIdx = 0u;
         ( IT_I2C_WAIT_LOOPS > loopIdx ) &&
         ( I2C_FLAG_INACTIVE != busBusy );
         loopIdx++ )
    {
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( IT_I2C_PERIPH, &busBusy ) );
    }

    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, busBusy );
}


/**
 * \brief Writes data to the registers of the slave, checks the register file and reads the
 *        data back (register address write, repeated START, read).
 *
 * \param slaveAddr [in]: Slave address
 * \param regAddr   [in]: First register
 * \param dataCnt   [in]: Count of data bytes (1 - IT_I2C_DATA_MAX)
 */
static void ItI2c_Check_WriteRead( i2c_SlaveAddr_t slaveAddr, uint8_t regAddr, i2c_DataCnt_t dataCnt )
{
    const i2c_XferRequest_t writeRequest = { slaveAddr,
                                             itI2c_TxBuf,
                                             (i2c_DataCnt_t)( dataCnt + 1u ),
                                             I2C_NULL_PTR,
                                             0u };
    const i2c_XferRequest_t readRequest  = { slaveAddr, itI2c_TxBuf, 1u, itI2c_RxBuf, dataCnt };

    TEST_ASSERT_LESS_OR_EQUAL_UINT32( IT_I2C_DATA_MAX, dataCnt );

    itI2c_TxBuf[ 0u ] = regAddr;

    for( uint32_t byteIdx = 0u; dataCnt > byteIdx; byteIdx++ )
    {
        itI2c_TxBuf[ byteIdx + 1u ] = (i2c_Data_t)( ( regAddr * 7u ) + ( byteIdx * 13u ) + dataCnt );
        itI2c_RxBuf[ byteIdx ]      = 0u;
    }

    ItI2c_Check_Xfer( &writeRequest );

    for( uint32_t byteIdx = 0u; dataCnt > byteIdx; byteIdx++ )
    {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE( itI2c_TxBuf[ byteIdx + 1u ], itI2c_Regs[ regAddr + byteIdx ], "Register" );
    }

    ItI2c_Check_Xfer( &readRequest );

    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE( &itI2c_TxBuf[ 1u ], itI2c_RxBuf, dataCnt, "Read back data" );
}


/**
 * \brief Checks that the master transfer completed without error and the bus is free.
 */
static void ItI2c_Check_MasterOk( void )
{
    i2c_FlagState_t busBusy = I2C_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 0u, itI2c_Master.ErrorCnt, "Master transfer error" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_Master.CompleteCnt, "Master transfer not complete" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 0u, itI2c_SlaveCb.ErrorCnt, "Slave transfer error" );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( IT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, busBusy );
}


/**
 * \brief Checks one complete slave transfer with the counts and the stored result.
 *
 * \param rxCnt [in]: Expected count of received bytes
 * \param txCnt [in]: Expected count of transmitted bytes
 */
static void ItI2c_Check_SlaveResult( i2c_DataCnt_t rxCnt, i2c_DataCnt_t txCnt )
{
    i2c_SlaveResult_t result;

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_SlaveCb.CompleteCnt, "Slave transfer not complete" );
    TEST_ASSERT_EQUAL_UINT16( rxCnt, itI2c_Slave.Result.RxCnt );
    TEST_ASSERT_EQUAL_UINT16( txCnt, itI2c_Slave.Result.TxCnt );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, itI2c_Slave.Result.RxBeyondBuffer );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, itI2c_Slave.Result.TxBeyondBuffer );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_SlaveResult( IT_I2C_SLAVE_PERIPH, &result ) );
    TEST_ASSERT_EQUAL_UINT16( rxCnt, result.RxCnt );
    TEST_ASSERT_EQUAL_UINT16( txCnt, result.TxCnt );
}


/**
 * \brief Fills a buffer with a data pattern.
 *
 * \param buffer  [out]: Buffer
 * \param dataCnt  [in]: Count of bytes
 * \param seed     [in]: First byte of the pattern
 */
static void ItI2c_Set_Pattern( i2c_Data_t * const buffer, i2c_DataCnt_t dataCnt, i2c_Data_t seed )
{
    for( i2c_DataCnt_t byteIdx = 0u; dataCnt > byteIdx; byteIdx++ )
    {
        buffer[ byteIdx ] = (i2c_Data_t)( seed + byteIdx );
    }
}


/**
 * \brief   Ignores the test on a board whose master I2C peripheral is not wired to the slave.
 *
 * \details Fixture of the board (EmBi_Platform, IntegrationTesting README "Board fixtures"):
 *          feature I2C_LOOP. A board without a fixture (IT_FIXTURE not defined) is expected
 *          to have the wiring of its board block.
 */
static void ItI2c_RequireLoop( void )
{
#if ( 0u == IT_I2C_SLAVE_AVAILABLE )
    TEST_IGNORE_MESSAGE( "No wiring of a slave I2C peripheral known for the board" );
#elif defined(IT_FEATURE_I2C_LOOP) || \
      !defined(IT_FIXTURE)
    /* Master and slave I2C peripheral are wired together */
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture I2C_LOOP (master I2C wired to slave I2C)" );
#endif
}


/** \brief Master transfer complete callback */
static void ItI2c_XferCompleteCallback( void )
{
    itI2c_Master.CompleteCnt++;
}


/**
 * \brief Master transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItI2c_ErrorCallback( i2c_XferErrorId_t errorId )
{
    itI2c_Master.ErrorId = errorId;
    itI2c_Master.ErrorCnt++;
}


/**
 * \brief Slave error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItI2c_SlaveErrorCallback( i2c_XferErrorId_t errorId )
{
    itI2c_SlaveCb.ErrorId = errorId;
    itI2c_SlaveCb.ErrorCnt++;
}


/**
 * \brief Slave address callback - in the register device pattern the transmit buffer of the
 *        read phase starts at the register received in the write phase (the register address
 *        is one byte, the register file has more registers).
 *
 * \param slaveDir    [in]: Direction of the phase
 * \param matchedAddr [in]: Matched own address
 */
static void ItI2c_SlaveAddrCallback( i2c_SlaveDir_t slaveDir, i2c_SlaveAddr_t matchedAddr )
{
    itI2c_Slave.AddrCnt++;
    itI2c_Slave.LastDir  = slaveDir;
    itI2c_Slave.LastAddr = matchedAddr;

    if( ( I2C_FUNCTION_ACTIVE == itI2c_Slave.RegMode ) &&
        ( I2C_SLAVE_DIR_READ  == slaveDir            )    )
    {
        const i2c_DataCnt_t     regAddr     = itI2c_SlaveRx[ 0u ];
        const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx,
                                                IT_I2C_SLAVE_RX_SIZE,
                                                &itI2c_Regs[ regAddr ],
                                                IT_I2C_REG_CNT - regAddr };

        (void)I2c_Set_SlaveBuffer( IT_I2C_SLAVE_PERIPH, &slaveBuffer );
    }
    else
    {
        /* Buffers are kept */
    }
}


/**
 * \brief Slave transfer complete callback - stores the result, in the register device pattern
 *        the written bytes following the register address are stored into the registers.
 *
 * \param slaveResult [in]: Result of the slave transfer
 */
static void ItI2c_SlaveCompleteCallback( const i2c_SlaveResult_t * const slaveResult )
{
    itI2c_Slave.Result = *slaveResult;

    if( ( I2C_FUNCTION_ACTIVE == itI2c_Slave.RegMode ) &&
        ( 1u                   < slaveResult->RxCnt   )    )
    {
        const i2c_DataCnt_t regAddr = itI2c_SlaveRx[ 0u ];

        for( i2c_DataCnt_t byteIdx = 1u; slaveResult->RxCnt > byteIdx; byteIdx++ )
        {
            const i2c_DataCnt_t regIdx = ( regAddr + byteIdx ) - 1u;

            if( IT_I2C_REG_CNT > regIdx )
            {
                itI2c_Regs[ regIdx ] = itI2c_SlaveRx[ byteIdx ];
            }
            else
            {
                /* Byte beyond the register file is dropped */
            }
        }
    }
    else
    {
        /* Result is only stored */
    }

    itI2c_SlaveCb.CompleteCnt++;
}
