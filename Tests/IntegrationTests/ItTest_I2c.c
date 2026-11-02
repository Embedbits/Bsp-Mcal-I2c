/**
 * \author Mr.Nobody
 * \file ItTest_I2c.c
 * \ingroup I2c
 * \brief Integration tests of I2C module on target (STM32L4 / STM32L4+).
 *
 * I2c module runs on the MCU together with real RCC, NVIC, GPIO and DMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): peripheral and pin initialization, kernel clock
 * selection, timing (SCL frequency) calculation from the real kernel clock,
 * Fast-mode Plus drive in SYSCFG, idle bus state, master transfers to a not
 * present slave (address NACK), master transfers to the slave of the module in
 * polling, interrupt and DMA mode (address, repeated START, NBYTES reload,
 * 10-bit addressing) and deinitialization.
 *
 * Master tests without a slave: the slave peripheral is not initialized, address
 * 0x50 (7-bit) and 0x2A5 (10-bit) are not present on the bus (NACK).
 *
 * Master + slave tests: the master I2C peripheral of the board is wired to the slave
 * I2C peripheral of the same MCU (SCL - SCL, SDA - SDA, external pull-up resistors
 * 4.7 kOhm, pins of the boards in the board blocks below). Fixture of the board
 * (EmBi_Platform IntegrationTesting, "Board fixtures"): the wiring is feature
 * I2C_LOOP - the master + slave tests are ignored on a board with a fixture without
 * it (a board without a fixture is expected to be wired) and on boards without a
 * known wiring. Both peripherals stretch the clock, the master and the slave can
 * share one CPU in every mode combination. The slave is the I2c module (slave mode),
 * a register device is emulated in its callbacks.
 *
 * Used resources: master I2C1, slave I2C2 on board specific pins; DMA1 channels
 * 6 / 7 (master DMA mode, STM32L4 request mapping of I2C1) and 4 / 5 (slave DMA
 * mode, request mapping of I2C2).
 *
 * \note Boards are named by the MCU detected by integration testing (DEV_ID and flash size).
 * \note The host halts the core for a few milliseconds on every mailbox access
 *       (probe-rs). Both peripherals stretch the clock, no byte is lost.
 * \note STM32L4 errata sheets not reviewed yet (errata workarounds of the STM32G4 module).
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


/** Address match record of the slave and state of the emulated register device */
typedef struct
{
    volatile uint32_t            AddrCnt;     /**< Address callbacks                     */
    volatile i2c_SlaveDir_t      LastDir;     /**< Direction of the last address match   */
    volatile i2c_SlaveAddr_t     LastAddr;    /**< Matched address of the last match     */
    volatile i2c_FunctionState_t RegMode;     /**< Register device pattern is emulated   */
    volatile i2c_DataCnt_t       RegPtr;      /**< Register selected by the last access  */
    i2c_SlaveResult_t            Result;      /**< Result of the last complete callback  */
}   it_I2cSlave_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void ItI2c_Init                 ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq );
static void ItI2c_Init_Master          ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq, i2c_AddrMode_t addrMode );
static void ItI2c_Init_Slave           ( i2c_XferMode_t xferMode,
                                         i2c_FreqHz_t busFreq,
                                         const i2c_SlaveConfig_t * const slaveConfig );
static void ItI2c_Get_SlaveConfig      ( i2c_SlaveConfig_t * const slaveConfig );
static void ItI2c_Start_RegDevice      ( i2c_XferMode_t xferMode,
                                         i2c_FreqHz_t busFreq,
                                         const i2c_SlaveConfig_t * const slaveConfig );
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
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection), MCUs
 * with the same DEV_ID and flash size share the block. Master I2C1 on every board. */
#if defined(IT_BOARD_STM32L4R5xI)

    /** NUCLEO-L4R5ZI: master I2C1 PB8 (SCL) / PB9 (SDA) -> slave I2C2 PB10 (SCL) / PB11 (SDA) */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C2_PB10 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C2_PB11 )
    #define IT_I2C_SLAVE_TX_DMA             ( I2C_TX_DMA_I2C2_DMA1_CHANNEL4 )
    #define IT_I2C_SLAVE_RX_DMA             ( I2C_RX_DMA_I2C2_DMA1_CHANNEL5 )

#elif defined(IT_BOARD_STM32L412xB) || \
      defined(IT_BOARD_STM32L422xB) || \
      defined(IT_BOARD_STM32L431xC) || \
      defined(IT_BOARD_STM32L432xC) || \
      defined(IT_BOARD_STM32L433xC) || \
      defined(IT_BOARD_STM32L442xC) || \
      defined(IT_BOARD_STM32L443xC)

    /** NUCLEO-L412KB / L432KC (Nucleo-32) and NUCLEO-L412RB-P / L433RC-P (Nucleo-64-P): master
     *  I2C1 on PA9 (SCL) / PA10 (SDA) - no wiring of a slave known for the boards */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PA9 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PA10 )
    #define IT_I2C_SLAVE_AVAILABLE          ( 0u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_3 )   /* Not used (STM32L432 / L442 have no I2C2) */
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_UNUSED )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_UNUSED )
    #define IT_I2C_SLAVE_TX_DMA             ( I2C_TX_DMA_UNUSED )
    #define IT_I2C_SLAVE_RX_DMA             ( I2C_RX_DMA_UNUSED )

#elif defined(IT_BOARD_STM32L451xE) || \
      defined(IT_BOARD_STM32L452xE) || \
      defined(IT_BOARD_STM32L462xE) || \
      defined(IT_BOARD_STM32L471xG) || \
      defined(IT_BOARD_STM32L475xG) || \
      defined(IT_BOARD_STM32L476xG) || \
      defined(IT_BOARD_STM32L485xG) || \
      defined(IT_BOARD_STM32L486xG) || \
      defined(IT_BOARD_STM32L496xG) || \
      defined(IT_BOARD_STM32L4A6xG) || \
      defined(IT_BOARD_STM32L4P5xG) || \
      defined(IT_BOARD_STM32L4Q5xG) || \
      defined(IT_BOARD_STM32L4R7xI) || \
      defined(IT_BOARD_STM32L4R9xI) || \
      defined(IT_BOARD_STM32L4S5xI) || \
      defined(IT_BOARD_STM32L4S7xI) || \
      defined(IT_BOARD_STM32L4S9xI)

    /** Nucleo-64 / Nucleo-144 boards: master I2C1 on PB8 (SCL, Arduino D15) / PB9 (SDA, Arduino D14)
     *  - no wiring of a slave known for the boards */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_SLAVE_AVAILABLE          ( 0u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_UNUSED )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_UNUSED )
    #define IT_I2C_SLAVE_TX_DMA             ( I2C_TX_DMA_UNUSED )
    #define IT_I2C_SLAVE_RX_DMA             ( I2C_RX_DMA_UNUSED )

#else
    #error "Board of I2c integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** I2C1 master (pins of the board block) */
#define IT_I2C_PERIPH                       ( I2C_PERIPH_1 )
#define IT_I2C_RCC_PCLK                     ( RCC_PERIPH_I2C1_PCLK1 )
#define IT_I2C_RCC_HSI                      ( RCC_PERIPH_I2C1_HSI )
#define IT_I2C_FMP_BIT                      ( SYSCFG_CFGR1_I2C1_FMP )

/** DMA1 channels of I2C1 (STM32L4 request mapping, DMAMUX1 on STM32L4+) */
#define IT_I2C_DMA_TX                       ( I2C_TX_DMA_I2C1_DMA1_CHANNEL6 )
#define IT_I2C_DMA_RX                       ( I2C_RX_DMA_I2C1_DMA1_CHANNEL7 )

/** Address of a not present slave (7-bit and 10-bit) */
#define IT_I2C_ABSENT_ADDR                  ( 0x50u )
#define IT_I2C_ABSENT_ADDR10                ( 0x2A5u )

/** Own address 1 of the slave of the module (7-bit and 10-bit) */
#define IT_I2C_OWN_ADDR                     ( 0x2Cu )
#define IT_I2C_OWN_ADDR10                   ( 0x2B7u )

/** Own address 2 of the slave of the module */
#define IT_I2C_OWN_ADDR2                    ( 0x3Au )

/** Standard-mode, Fast-mode and Fast-mode Plus SCL frequency [Hz] */
#define IT_I2C_FREQ_STANDARD_HZ             ( 100000u )
#define IT_I2C_FREQ_FAST_HZ                 ( 400000u )
#define IT_I2C_FREQ_FAST_PLUS_HZ            ( 1000000u )

/** Tolerance of the SCL frequency read back from TIMINGR [%] */
#define IT_I2C_FREQ_TOL_PCT                 ( 10u )

/** Count of percent in one unit */
#define IT_I2C_PERCENT                      ( 100u )

/** Maximal count of wait loop iterations (I2c_Task called) */
#define IT_I2C_WAIT_LOOPS                   ( 2000000u )

/** Size of the data buffers (longer than one NBYTES chunk of 255 bytes) */
#define IT_I2C_BUF_SIZE                     ( 300u )

/** Count of bytes of a short transfer */
#define IT_I2C_SHORT_SIZE                   ( 8u )

/** Count of data bytes of the longest write / read back test transfer (+ register address) */
#define IT_I2C_DATA_MAX                     ( 16u )

/** Count of registers of the emulated register device (power of 2) */
#define IT_I2C_REG_CNT                      ( 32u )

/** Value of the first received byte which is not a register address */
#define IT_I2C_REG_NONE                     ( 0xFFu )

/** Size of the receive buffer of the master tests without a slave */
#define IT_I2C_RX_DATA_SIZE                 ( 4u )

/** First register of the Fast-mode write and read back test (+ mode and count index) */
#define IT_I2C_REG_FAST_BASE                ( 8u )

/** Value of register 0 of the read-only test (register j holds this value + j) */
#define IT_I2C_REGS_PRESET                  ( 0xC0u )

/** Count of bytes written and read back in the 10-bit test */
#define IT_I2C_WRITE_READ_CNT               ( 3u )

/** Count of address matches of a register read (write phase, read phase) */
#define IT_I2C_REG_READ_ADDR_CNT            ( 2u )

/** Multipliers of the register address and of the byte index in the write and read back pattern */
#define IT_I2C_PATTERN_REG_MUL              ( 7u )
#define IT_I2C_PATTERN_BYTE_MUL             ( 13u )

/** Register address read in the register device tests */
#define IT_I2C_REG_ADDR                     ( 5u )

/** Count of registers read in the register device tests */
#define IT_I2C_REG_READ_CNT                 ( 4u )

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

/** Priority of the I2C interrupts of the master and of the slave */
#define IT_I2C_IRQ_PRIO                     ( 5u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Data handling configuration of the master and of the slave (copied by the module) */
static i2c_DataConfig_t             itI2c_DataConfig;
static i2c_DataConfig_t             itI2c_SlaveDataConfig;

/** Transmit data of NACK transfers */
static const i2c_Data_t             itI2c_TxData[ 2u ] = { 0x00u, 0x10u };

/** Receive data of the master tests without a slave */
static i2c_Data_t                   itI2c_RxData[ IT_I2C_RX_DATA_SIZE ];

/** Data buffers of the master + slave tests */
static i2c_Data_t                   itI2c_MasterTx[ IT_I2C_BUF_SIZE ];
static i2c_Data_t                   itI2c_MasterRx[ IT_I2C_BUF_SIZE ];
static i2c_Data_t                   itI2c_SlaveTx[ IT_I2C_BUF_SIZE ];
static i2c_Data_t                   itI2c_SlaveRx[ IT_I2C_BUF_SIZE ];

/** Registers of the emulated register device */
static i2c_Data_t                   itI2c_Regs[ IT_I2C_REG_CNT ];

/** Callback records of the master and of the slave */
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
    itI2c_Slave.RegPtr        = 0u;
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
 * \details Initializes I2C3 (polling mode, 100 kHz, PCLK1 kernel clock, internal
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
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_STANDARD_HZ * IT_I2C_FREQ_TOL_PCT ) / IT_I2C_PERCENT,
                               IT_I2C_FREQ_STANDARD_HZ,
                               busFreq );

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
    TEST_ASSERT_EQUAL_HEX32( 0u, SYSCFG->CFGR1 & IT_I2C_FMP_BIT );
}


/**
 * \brief   I2c_Init() on target in fast mode - bus frequency read back.
 *
 * \details Initializes I2C3 with 400 kHz and reads the bus frequency.
 *
 * \par Expected results
 * - Bus frequency 400 kHz +- 10 %.
 */
void It_I2c_Init_FastMode_BusFreqReadBack( void )
{
    i2c_FreqHz_t busFreq = 0u;

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_HZ * IT_I2C_FREQ_TOL_PCT ) / IT_I2C_PERCENT,
                               IT_I2C_FREQ_FAST_HZ,
                               busFreq );
}


/**
 * \brief   Fast-mode Plus enables the 20 mA drive in SYSCFG, I2c_Deinit() releases it.
 *
 * \details Initializes I2C3 with 1 MHz (PCLK1 kernel clock), reads the bus frequency
 *          and SYSCFG_CFGR1.I2C3_FMP, then deinitializes the peripheral.
 *
 * \par Expected results
 * - Bus frequency 1 MHz +- 10 %, I2C3_FMP = 1.
 * - After I2c_Deinit(): I2C3_FMP = 0 (SYSCFG is not reset with the I2C).
 */
void It_I2c_Init_FastModePlus_FmpDriveEnabledAndReleased( void )
{
    i2c_FreqHz_t busFreq = 0u;

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_PLUS_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_PLUS_HZ * IT_I2C_FREQ_TOL_PCT ) / IT_I2C_PERCENT,
                               IT_I2C_FREQ_FAST_PLUS_HZ,
                               busFreq );
    TEST_ASSERT_EQUAL_HEX32( IT_I2C_FMP_BIT, SYSCFG->CFGR1 & IT_I2C_FMP_BIT );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( IT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, SYSCFG->CFGR1 & IT_I2C_FMP_BIT );
}


/**
 * \brief   HSI16 kernel clock - Fast-mode is accepted, Fast-mode Plus is refused.
 *
 * \details Initializes I2C3 with HSI16 kernel clock (16 MHz) and 400 kHz, reads the
 *          bus frequency, then initializes it with 1 MHz.
 *
 * \note    Device errata ES0430 2.15.1 ("wrong data sampling when data setup time
 *          (tSU;DAT) is shorter than one I2C kernel clock period") - Fast-mode Plus
 *          requires I2CCLK >= 20 MHz, Fast-mode I2CCLK >= 10 MHz.
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
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_HZ * IT_I2C_FREQ_TOL_PCT ) / IT_I2C_PERCENT,
                               IT_I2C_FREQ_FAST_HZ,
                               busFreq );

    config.BusFreq = IT_I2C_FREQ_FAST_PLUS_HZ;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Init( &config ) );
}


/**
 * \brief   Re-initialization with another kernel clock source changes the clock.
 *
 * \details Initializes I2C3 with PCLK1 kernel clock, then with HSI16 kernel clock
 *          without I2c_Deinit() and reads the kernel clock source from RCC.
 *
 * \note    Regression test of STM32H5 module bug AB#1098 - RCC changes the kernel
 *          clock multiplexer only of a released clock.
 *
 * \par Expected results
 * - Both initializations I2C_REQUEST_OK.
 * - Kernel clock source is HSI16, bus frequency 100 kHz +- 10 %.
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
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_STANDARD_HZ * IT_I2C_FREQ_TOL_PCT ) / IT_I2C_PERCENT,
                               IT_I2C_FREQ_STANDARD_HZ,
                               busFreq );
}


/**
 * \brief   Idle bus with pull-ups is reported free.
 *
 * \details Initializes I2C3 with internal pull-ups and reads bus state.
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
 * \details Initializes I2C3 in DMA mode, deinitializes it and tries to start an
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
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR, itI2c_TxData, 1u, itI2c_RxData, 2u };

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
    const i2c_XferRequest_t request = { IT_I2C_ABSENT_ADDR, I2C_NULL_PTR, 0u, itI2c_RxData, 4u };

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


/*-------------------------- Master + slave (I2C_LOOP) -----------------------*/

/**
 * \brief   Address-only transfer to the slave completes in all master modes.
 *
 * \details Slave (ISR) listens; address-only transfer of the master in POLL, ISR and DMA
 *          mode.
 *
 * \par Expected results
 * - Complete callback of the master and of the slave for every transfer, slave counted 3
 *   address matches.
 */
void It_I2c_Set_XferStart_AddressOnlyAllModes_Completed( void )
{
    const i2c_XferMode_t    modeLut[ ]  = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_XferRequest_t request     = { IT_I2C_OWN_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };
    i2c_SlaveConfig_t       slaveConfig;

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Start_RegDevice( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        ItI2c_Check_Xfer( &request );
    }

    TEST_ASSERT_EQUAL_UINT32( 3u, itI2c_Slave.AddrCnt );
}


/**
 * \brief   Write and read back of 1, 2, 3, 5 and 16 bytes in all master modes (100 kHz).
 *
 * \details Slave (ISR) emulates a register device. For every master mode (POLL, ISR, DMA)
 *          and count of bytes: register address + data are written, the register file of the
 *          slave is checked, then register address write + repeated START + read returns the
 *          data.
 *
 * \par Expected results
 * - Every transfer completes, slave registers and read data equal the written data
 *   (DMA channels are re-armed for every transfer).
 */
void It_I2c_Set_XferStart_WriteReadStandardModeAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 3u, 5u, IT_I2C_DATA_MAX };
    i2c_SlaveConfig_t    slaveConfig;

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Start_RegDevice( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            ItI2c_Check_WriteRead( IT_I2C_OWN_ADDR, (uint8_t)( modeIdx + cntIdx ), cntLut[ cntIdx ] );
        }
    }
}


/**
 * \brief   Write and read back in all master modes in Fast-mode (400 kHz).
 *
 * \details Same as the Standard-mode test with 1, 2, 3 and 16 bytes at 400 kHz.
 *
 * \par Expected results
 * - Every transfer completes, slave registers and read data equal the written data.
 */
void It_I2c_Set_XferStart_WriteReadFastModeAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 3u, IT_I2C_DATA_MAX };
    i2c_SlaveConfig_t    slaveConfig;

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Start_RegDevice( I2C_XFER_MODE_ISR, IT_I2C_FREQ_FAST_HZ, &slaveConfig );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_FAST_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            const uint8_t regAddr = (uint8_t)( IT_I2C_REG_FAST_BASE + modeIdx + cntIdx );

            ItI2c_Check_WriteRead( IT_I2C_OWN_ADDR, regAddr, cntLut[ cntIdx ] );
        }
    }
}


/**
 * \brief   Read-only transfers (no repeated START) return data from the selected
 *          register in all master modes.
 *
 * \details Slave (ISR) registers are preset, register 4 is selected by a 1 byte write,
 *          then 1, 2 and 6 bytes are read by read-only transfers, the next register is
 *          selected before every read, in POLL, ISR and DMA mode of the master.
 *
 * \par Expected results
 * - Read data equal consecutive slave registers 4, 5 - 6, 7 - 12.
 */
void It_I2c_Set_XferStart_ReadOnlyAllModes_ConsecutiveRegisters( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 6u };
    i2c_SlaveConfig_t    slaveConfig;

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Start_RegDevice( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );

    for( uint32_t regIdx = 0u; IT_I2C_REG_CNT > regIdx; regIdx++ )
    {
        itI2c_Regs[ regIdx ] = (i2c_Data_t)( IT_I2C_REGS_PRESET + regIdx );
    }

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        const i2c_XferRequest_t selRequest = { IT_I2C_OWN_ADDR, itI2c_MasterTx, 1u, I2C_NULL_PTR, 0u };
        uint8_t                 regAddr    = 4u;

        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            const i2c_XferRequest_t readRequest = { IT_I2C_OWN_ADDR,
                                                    I2C_NULL_PTR,
                                                    0u,
                                                    itI2c_MasterRx,
                                                    cntLut[ cntIdx ] };

            itI2c_MasterTx[ 0u ] = regAddr;
            ItI2c_Check_Xfer( &selRequest );

            ItI2c_Check_Xfer( &readRequest );

            for( uint32_t byteIdx = 0u; cntLut[ cntIdx ] > byteIdx; byteIdx++ )
            {
                TEST_ASSERT_EQUAL_HEX8( IT_I2C_REGS_PRESET + regAddr + byteIdx, itI2c_MasterRx[ byteIdx ] );
            }

            regAddr += (uint8_t)cntLut[ cntIdx ];
        }
    }
}


/**
 * \brief   10-bit addressing: write, write + read and read-only transfers.
 *
 * \details Slave (ISR) with 10-bit own address 0x2B7 emulates a register device, master in
 *          10-bit mode: write + read back of 3 bytes and read-only transfer of 2 bytes
 *          (complete 10-bit read sequence - address in write direction, repeated START,
 *          read header) in POLL, ISR and DMA mode of the master.
 *
 * \par Expected results
 * - Every transfer completes, read data equal the slave registers.
 */
void It_I2c_Set_XferStart_10BitAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    i2c_SlaveConfig_t    slaveConfig;

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.OwnAddr     = IT_I2C_OWN_ADDR10;
    slaveConfig.OwnAddrMode = I2C_ADDR_MODE_10BIT;
    ItI2c_Start_RegDevice( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        const i2c_XferRequest_t readRequest = { IT_I2C_OWN_ADDR10, I2C_NULL_PTR, 0u, itI2c_MasterRx, 2u };
        const uint8_t           regAddr     = (uint8_t)( 20u + modeIdx );

        ItI2c_Init_Master( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_10BIT );

        ItI2c_Check_WriteRead( IT_I2C_OWN_ADDR10, regAddr, IT_I2C_WRITE_READ_CNT );

        /* Read-only transfer starts at the register selected here */
        itI2c_Slave.RegPtr = regAddr;
        ItI2c_Check_Xfer( &readRequest );
        TEST_ASSERT_EQUAL_HEX8( itI2c_Regs[ regAddr ],      itI2c_MasterRx[ 0u ] );
        TEST_ASSERT_EQUAL_HEX8( itI2c_Regs[ regAddr + 1u ], itI2c_MasterRx[ 1u ] );
    }
}


/**
 * \brief   Transfers longer than 255 bytes use NBYTES reload in all master modes.
 *
 * \details Slave (ISR) with buffers of 300 bytes. For every master mode (POLL, ISR, DMA, 400 kHz):
 *          write of 300 bytes, then read of 300 bytes.
 *
 * \par Expected results
 * - Every transfer completes (one slave transfer each), the slave received the written
 *   bytes and the master read the transmit buffer of the slave (all 300 bytes).
 */
void It_I2c_Set_XferStart_LongTransferAllModes_ReloadChunks( void )
{
    const i2c_XferMode_t    modeLut[ ]  = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_XferRequest_t writeReq    = { IT_I2C_OWN_ADDR, itI2c_MasterTx, IT_I2C_BUF_SIZE, I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t readReq     = { IT_I2C_OWN_ADDR, I2C_NULL_PTR, 0u, itI2c_MasterRx, IT_I2C_BUF_SIZE };
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_SlaveTx, IT_I2C_BUF_SIZE };
    i2c_SlaveConfig_t       slaveConfig;

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_FAST_HZ, &slaveConfig );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        ItI2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_FAST_HZ );

        ItI2c_Set_Pattern( itI2c_MasterTx, IT_I2C_BUF_SIZE, (i2c_Data_t)( IT_I2C_SEED_MASTER + modeIdx ) );
        ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_BUF_SIZE, (i2c_Data_t)( IT_I2C_SEED_SLAVE + modeIdx ) );

        for( uint32_t byteIdx = 0u; IT_I2C_BUF_SIZE > byteIdx; byteIdx++ )
        {
            itI2c_SlaveRx[ byteIdx ]  = 0u;
            itI2c_MasterRx[ byteIdx ] = 0u;
        }

        ItI2c_Check_Xfer( &writeReq );
        TEST_ASSERT_EQUAL_UINT16( IT_I2C_BUF_SIZE, itI2c_Slave.Result.RxCnt );
        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE( itI2c_MasterTx, itI2c_SlaveRx, IT_I2C_BUF_SIZE, "Slave received data" );

        ItI2c_Check_Xfer( &readReq );
        TEST_ASSERT_EQUAL_UINT16( IT_I2C_BUF_SIZE, itI2c_Slave.Result.TxCnt );
        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE( itI2c_SlaveTx, itI2c_MasterRx, IT_I2C_BUF_SIZE, "Read back data" );
    }
}

/*-------------------------- Master + slave (I2C_LOOP) -----------------------*/

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
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_SlaveTx, IT_I2C_BUF_SIZE };
    const i2c_XferRequest_t request     = { IT_I2C_OWN_ADDR, itI2c_MasterTx, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_7BIT );
    ItI2c_Set_Pattern( itI2c_MasterTx, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_UINT32( 1u, itI2c_Slave.AddrCnt );
    TEST_ASSERT_EQUAL( I2C_SLAVE_DIR_WRITE, itI2c_Slave.LastDir );
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_OWN_ADDR, itI2c_Slave.LastAddr );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_MasterTx, itI2c_SlaveRx, IT_I2C_SHORT_SIZE );
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
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_Regs, IT_I2C_REG_CNT };
    const i2c_XferRequest_t request     = { IT_I2C_OWN_ADDR, &regAddr, 1u, itI2c_MasterRx, IT_I2C_REG_READ_CNT };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_7BIT );
    ItI2c_Set_Pattern( itI2c_Regs, IT_I2C_REG_CNT, IT_I2C_SEED_REGS );
    itI2c_Slave.RegMode = I2C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( 1u, IT_I2C_REG_READ_CNT );
    TEST_ASSERT_EQUAL_UINT32( IT_I2C_REG_READ_ADDR_CNT, itI2c_Slave.AddrCnt );
    TEST_ASSERT_EQUAL( I2C_SLAVE_DIR_READ, itI2c_Slave.LastDir );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itI2c_Regs[ IT_I2C_REG_ADDR ], itI2c_MasterRx, IT_I2C_REG_READ_CNT );
}


/**
 * \brief   Register read of the master is answered by the slave in DMA mode.
 *
 * \details As the polling register read, slave in DMA mode (DMA1 channels 4 / 5),
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
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_Regs, IT_I2C_REG_CNT };
    const i2c_XferRequest_t request     = { IT_I2C_OWN_ADDR, &regAddr, 1u, itI2c_MasterRx, IT_I2C_REG_READ_CNT };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_7BIT );
    ItI2c_Set_Pattern( itI2c_Regs, IT_I2C_REG_CNT, IT_I2C_SEED_REGS );
    itI2c_Slave.RegMode = I2C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( 1u, IT_I2C_REG_READ_CNT );
    TEST_ASSERT_EQUAL_UINT32( IT_I2C_REG_READ_ADDR_CNT, itI2c_Slave.AddrCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itI2c_Regs[ IT_I2C_REG_ADDR ], itI2c_MasterRx, IT_I2C_REG_READ_CNT );
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
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_SlaveTx, IT_I2C_BUF_SIZE };
    const i2c_XferRequest_t writeReq    = { IT_I2C_OWN_ADDR, itI2c_MasterTx, IT_I2C_BUF_SIZE, I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t readReq     = { IT_I2C_OWN_ADDR, I2C_NULL_PTR, 0u, itI2c_MasterRx, IT_I2C_BUF_SIZE };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_DMA, IT_I2C_FREQ_FAST_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_DMA, IT_I2C_FREQ_FAST_HZ, I2C_ADDR_MODE_7BIT );
    ItI2c_Set_Pattern( itI2c_MasterTx, IT_I2C_BUF_SIZE, IT_I2C_SEED_MASTER );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_BUF_SIZE, IT_I2C_SEED_SLAVE );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &writeReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_BUF_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_MasterTx, itI2c_SlaveRx, IT_I2C_BUF_SIZE );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &readReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( 0u, IT_I2C_BUF_SIZE );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_SlaveTx, itI2c_MasterRx, IT_I2C_BUF_SIZE );
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
    i2c_Data_t              expected[ IT_I2C_SHORT_SIZE ];
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_SlaveTx, IT_I2C_TX_PART_SIZE };
    const i2c_XferRequest_t request     = { IT_I2C_OWN_ADDR, I2C_NULL_PTR, 0u, itI2c_MasterRx, IT_I2C_BEYOND_READ_CNT };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.FillByte = IT_I2C_FILL_BYTE;
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_7BIT );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_TX_PART_SIZE, IT_I2C_SEED_SLAVE );

    for( uint32_t byteIdx = 0u; IT_I2C_SHORT_SIZE > byteIdx; byteIdx++ )
    {
        expected[ byteIdx ] = IT_I2C_FILL_BYTE;
    }
    expected[ 0u ] = itI2c_SlaveTx[ 0u ];
    expected[ 1u ] = itI2c_SlaveTx[ 1u ];

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &request ) );
    ItI2c_Wait_MasterSlaveEnd();

    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX8_ARRAY( expected, itI2c_MasterRx, IT_I2C_BEYOND_READ_CNT );
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
    const i2c_XferRequest_t readReq     = { IT_I2C_OWN_ADDR, I2C_NULL_PTR, 0u, itI2c_MasterRx, IT_I2C_BEYOND_READ_CNT };
    const i2c_XferRequest_t writeReq    = { IT_I2C_OWN_ADDR, itI2c_MasterTx, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.FillByte = IT_I2C_FILL_BYTE;
    ItI2c_Init_Slave( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_7BIT );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_TX_PART_SIZE, IT_I2C_SEED_SLAVE );
    ItI2c_Set_Pattern( itI2c_MasterTx, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &readReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_SlaveTx, itI2c_MasterRx, IT_I2C_TX_PART_SIZE );
    for( uint32_t byteIdx = IT_I2C_TX_PART_SIZE; IT_I2C_BEYOND_READ_CNT > byteIdx; byteIdx++ )
    {
        TEST_ASSERT_EQUAL_HEX8( IT_I2C_FILL_BYTE, itI2c_MasterRx[ byteIdx ] );
    }
    TEST_ASSERT_EQUAL_UINT16( IT_I2C_TX_PART_SIZE, itI2c_Slave.Result.TxCnt );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, itI2c_Slave.Result.TxBeyondBuffer );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &writeReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_MasterTx, itI2c_SlaveRx, IT_I2C_RX_PART_SIZE );
    TEST_ASSERT_EQUAL_UINT16( IT_I2C_RX_PART_SIZE, itI2c_Slave.Result.RxCnt );
    TEST_ASSERT_EQUAL( I2C_FLAG_ACTIVE, itI2c_Slave.Result.RxBeyondBuffer );
}


/**
 * \brief   10-bit addressing between master and slave.
 *
 * \details Slave (ISR) with 10-bit own address 0x2B7; master (ISR, 10-bit mode)
 *          writes 8 bytes, then reads 8 bytes.
 *
 * \par Expected results
 * - Both master transfers complete, data exchanged.
 * - Slave reports matched address 0x2B7 (10-bit header decoded).
 */
void It_I2c_Slave_Isr10BitAddress_DataExchanged( void )
{
    i2c_SlaveConfig_t       slaveConfig;
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_SlaveTx, IT_I2C_BUF_SIZE };
    const i2c_XferRequest_t writeReq    = { IT_I2C_OWN_ADDR10, itI2c_MasterTx, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t readReq     = { IT_I2C_OWN_ADDR10, I2C_NULL_PTR, 0u, itI2c_MasterRx, IT_I2C_SHORT_SIZE };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.OwnAddr     = IT_I2C_OWN_ADDR10;
    slaveConfig.OwnAddrMode = I2C_ADDR_MODE_10BIT;
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_10BIT );
    ItI2c_Set_Pattern( itI2c_MasterTx, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );
    ItI2c_Set_Pattern( itI2c_SlaveTx, IT_I2C_SHORT_SIZE, IT_I2C_SEED_SLAVE );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &writeReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_OWN_ADDR10, itI2c_Slave.Result.MatchedAddr );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_MasterTx, itI2c_SlaveRx, IT_I2C_SHORT_SIZE );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &readReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_OWN_ADDR10, itI2c_Slave.Result.MatchedAddr );
    TEST_ASSERT_EQUAL_UINT16( IT_I2C_SHORT_SIZE, itI2c_Slave.Result.TxCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_SlaveTx, itI2c_MasterRx, IT_I2C_SHORT_SIZE );
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
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_SlaveTx, IT_I2C_BUF_SIZE };
    const i2c_XferRequest_t addr2Req    = { IT_I2C_OWN_ADDR2, itI2c_MasterTx, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t gcReq       = { I2C_SLAVE_ADDR_GENERAL_CALL,
                                            itI2c_MasterTx,
                                            IT_I2C_SHORT_SIZE,
                                            I2C_NULL_PTR,
                                            0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    slaveConfig.OwnAddr2State = I2C_FUNCTION_ACTIVE;
    slaveConfig.OwnAddr2      = IT_I2C_OWN_ADDR2;
    slaveConfig.GeneralCall   = I2C_FUNCTION_ACTIVE;
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_7BIT );
    ItI2c_Set_Pattern( itI2c_MasterTx, IT_I2C_SHORT_SIZE, IT_I2C_SEED_MASTER );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &addr2Req ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX16( IT_I2C_OWN_ADDR2, itI2c_Slave.Result.MatchedAddr );

    setUp();
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &gcReq ) );
    ItI2c_Wait_MasterSlaveEnd();
    ItI2c_Check_MasterOk();
    ItI2c_Check_SlaveResult( IT_I2C_SHORT_SIZE, 0u );
    TEST_ASSERT_EQUAL_HEX16( I2C_SLAVE_ADDR_GENERAL_CALL, itI2c_Slave.Result.MatchedAddr );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itI2c_MasterTx, itI2c_SlaveRx, IT_I2C_SHORT_SIZE );
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
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_SlaveTx, IT_I2C_BUF_SIZE };
    const i2c_XferRequest_t request     = { IT_I2C_OWN_ADDR, itI2c_MasterTx, IT_I2C_SHORT_SIZE, I2C_NULL_PTR, 0u };

    ItI2c_RequireLoop();

    ItI2c_Get_SlaveConfig( &slaveConfig );
    ItI2c_Init_Slave( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, &slaveConfig );
    ItI2c_Init_Master( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ, I2C_ADDR_MODE_7BIT );

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
 * \brief Initializes I2C1 with internal pull-ups and data handling (master tests without slave).
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
 * \brief Initializes the slave I2C peripheral of the board with data handling.
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
    itI2c_SlaveDataConfig.TxDma                     = IT_I2C_SLAVE_TX_DMA;
    itI2c_SlaveDataConfig.TxDmaPriority             = I2C_DMA_PRIORITY_HIGH;
    itI2c_SlaveDataConfig.RxDma                     = IT_I2C_SLAVE_RX_DMA;
    itI2c_SlaveDataConfig.RxDmaPriority             = I2C_DMA_PRIORITY_HIGH;
    itI2c_SlaveDataConfig.IrqPriority               = IT_I2C_IRQ_PRIO;
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
 * \brief Default slave configuration of the tests: own address 1 0x2C (7-bit).
 *
 * \param slaveConfig [out]: Slave configuration
 */
static void ItI2c_Get_SlaveConfig( i2c_SlaveConfig_t * const slaveConfig )
{
    i2c_Config_t config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );

    *slaveConfig         = config.SlaveConfig;
    slaveConfig->OwnAddr = IT_I2C_OWN_ADDR;
}


/**
 * \brief Initializes the slave as a register device (registers preset) and starts listening.
 *
 * \param xferMode    [in]: Data transfer mode of the slave
 * \param busFreq     [in]: SCL frequency of the master [Hz]
 * \param slaveConfig [in]: Slave configuration
 */
static void ItI2c_Start_RegDevice( i2c_XferMode_t xferMode,
                                   i2c_FreqHz_t busFreq,
                                   const i2c_SlaveConfig_t * const slaveConfig )
{
    const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx, IT_I2C_BUF_SIZE, itI2c_Regs, IT_I2C_REG_CNT };

    ItI2c_Init_Slave( xferMode, busFreq, slaveConfig );
    ItI2c_Set_Pattern( itI2c_Regs, IT_I2C_REG_CNT, IT_I2C_SEED_REGS );
    itI2c_SlaveRx[ 0u ] = IT_I2C_REG_NONE;
    itI2c_Slave.RegMode = I2C_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_ListenStart( IT_I2C_SLAVE_PERIPH, &slaveBuffer ) );
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
 * \brief Executes a transfer to the slave of the module and checks the successful end of the
 *        master and of the slave transfer.
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

    ItI2c_Check_MasterOk();
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_SlaveCb.CompleteCnt, "Slave transfer not complete" );

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
                                             itI2c_MasterTx,
                                             (i2c_DataCnt_t)( dataCnt + 1u ),
                                             I2C_NULL_PTR,
                                             0u };
    const i2c_XferRequest_t readRequest  = { slaveAddr, itI2c_MasterTx, 1u, itI2c_MasterRx, dataCnt };

    TEST_ASSERT_LESS_OR_EQUAL_UINT32( IT_I2C_DATA_MAX, dataCnt );

    itI2c_MasterTx[ 0u ] = regAddr;

    for( uint32_t byteIdx = 0u; dataCnt > byteIdx; byteIdx++ )
    {
        itI2c_MasterTx[ byteIdx + 1u ] = (i2c_Data_t)( ( regAddr * IT_I2C_PATTERN_REG_MUL )   +
                                                       ( byteIdx * IT_I2C_PATTERN_BYTE_MUL ) +
                                                       dataCnt );
        itI2c_MasterRx[ byteIdx ]      = 0u;
    }

    ItI2c_Check_Xfer( &writeRequest );

    for( uint32_t byteIdx = 0u; dataCnt > byteIdx; byteIdx++ )
    {
        const uint32_t regIdx = ( regAddr + byteIdx ) & ( IT_I2C_REG_CNT - 1u );

        TEST_ASSERT_EQUAL_HEX8_MESSAGE( itI2c_MasterTx[ byteIdx + 1u ], itI2c_Regs[ regIdx ], "Slave register" );
    }

    ItI2c_Check_Xfer( &readRequest );

    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE( &itI2c_MasterTx[ 1u ], itI2c_MasterRx, dataCnt, "Read back data" );
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
 *        read phase starts at the register received in the write phase of the same transfer
 *        (read-only transfer: at the register selected by the previous access).
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
        i2c_DataCnt_t regAddr = itI2c_Slave.RegPtr;

        if( IT_I2C_REG_CNT > itI2c_SlaveRx[ 0u ] )
        {
            /* Register address written before the repeated START of the transfer */
            regAddr = itI2c_SlaveRx[ 0u ];
        }
        else
        {
            /* Read-only transfer - the register selected by the previous access */
        }

        if( IT_I2C_REG_CNT > regAddr )
        {
            const i2c_SlaveBuffer_t slaveBuffer = { itI2c_SlaveRx,
                                                    IT_I2C_BUF_SIZE,
                                                    &itI2c_Regs[ regAddr ],
                                                    IT_I2C_REG_CNT - regAddr };

            (void)I2c_Set_SlaveBuffer( IT_I2C_SLAVE_PERIPH, &slaveBuffer );
        }
        else
        {
            /* Buffers are kept */
        }

        itI2c_Slave.RegPtr = regAddr;
    }
    else
    {
        /* Write phase (the first received byte is the register address) - buffers are kept */
    }
}


/**
 * \brief Slave transfer complete callback - the register device stores the written data and
 *        moves the register pointer.
 *
 * \param slaveResult [in]: Result of the slave transfer
 */
static void ItI2c_SlaveCompleteCallback( const i2c_SlaveResult_t * const slaveResult )
{
    itI2c_Slave.Result = *slaveResult;

    if( ( I2C_FUNCTION_ACTIVE == itI2c_Slave.RegMode ) &&
        ( 0u                   < slaveResult->RxCnt  )    )
    {
        const i2c_DataCnt_t regAddr = itI2c_SlaveRx[ 0u ];

        for( i2c_DataCnt_t byteIdx = 1u; slaveResult->RxCnt > byteIdx; byteIdx++ )
        {
            itI2c_Regs[ ( regAddr + byteIdx - 1u ) & ( IT_I2C_REG_CNT - 1u ) ] = itI2c_SlaveRx[ byteIdx ];
        }

        itI2c_Slave.RegPtr = ( regAddr + slaveResult->RxCnt - 1u ) & ( IT_I2C_REG_CNT - 1u );
    }
    else
    {
        /* No register access */
    }

    if( ( I2C_FUNCTION_ACTIVE == itI2c_Slave.RegMode ) &&
        ( 0u                   < slaveResult->TxCnt  )    )
    {
        itI2c_Slave.RegPtr = ( itI2c_Slave.RegPtr + slaveResult->TxCnt ) & ( IT_I2C_REG_CNT - 1u );
    }
    else
    {
        /* No register read */
    }

    if( I2C_FUNCTION_ACTIVE == itI2c_Slave.RegMode )
    {
        /* End of the transfer - the next transfer starts without a register address */
        itI2c_SlaveRx[ 0u ] = IT_I2C_REG_NONE;
    }
    else
    {
        /* Receive buffer is kept */
    }

    itI2c_SlaveCb.CompleteCnt++;
}
