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
 * polling, interrupt and DMA mode and deinitialization.
 *
 * Master: I2C1 on PB8 (SCL) / PB9 (SDA) - Arduino D15 / D14 of both board types -
 * with internal pull-ups. Address 0x50 (7-bit) and 0x2A5 (10-bit) are not present
 * on the bus (NACK). DMA1 stream 6 (TX) / stream 0 (RX), channel 1.
 *
 * Slave (boards with wiring only, otherwise the slave tests are ignored): I2C2
 * of the same MCU is connected to the master bus and emulates a register device
 * in the interrupt of the test set (LL access): the first written byte selects
 * the register, following written bytes are stored and read bytes are returned
 * from the selected register with auto increment.
 *
 * Boards (named by the MCU as the detection of the connected boards does):
 * - STM32F745xG / STM32F746xG - 32F746GDISCOVERY (STM32F746NG); the detection
 *   names every board with ID 0x449 and 1 MB flash STM32F745xG
 * - STM32F722xE, STM32F756xG, STM32F765xI / STM32F767xI - NUCLEO-F722ZE,
 *   NUCLEO-F756ZG (board file override with the name STM32F756xG), NUCLEO-F767ZI
 *   (Nucleo-144 boards, common pinout)
 *
 * Nucleo-144: I2C2 slave PF1 (SCL) / PF0 (SDA), wiring PB8 - PF1 and PB9 - PF0.
 * 32F746GDISCOVERY: no second I2C on the connectors - the slave tests are ignored.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "I2c_Port.h"                       /* Module under test              */
#include "Rcc_Port.h"                       /* Kernel clock source read back  */
#include "Nvic_Port.h"                      /* Interrupt of the slave         */
#include "Stm32_i2c.h"                      /* Slave registers (LL)           */
#include "Stm32_system.h"                   /* SYSCFG Fast-mode Plus bits     */
/* ============================= TYPEDEFS =================================== */

/** \brief State of the emulated register slave */
typedef struct
{
    volatile uint8_t  Regs[ 32u ];   /**< Register file                                */
    volatile uint8_t  RegPtr;        /**< Selected register                            */
    volatile uint8_t  PtrPending;    /**< Next written byte selects the register       */
    volatile uint32_t AddrCnt;       /**< Count of address matches                     */
}   it_I2cSlave_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void It_I2c_Init                 ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq );
static void It_I2c_Init_Slave           ( i2c_AddrMode_t addrMode );
static void It_I2c_Set_SlaveAddr        ( i2c_AddrMode_t addrMode );
static void It_I2c_Slave_IsrHandler     ( void );
static void It_I2c_Wait_XferEnd         ( void );
static void It_I2c_Check_Xfer           ( const i2c_XferRequest_t * const xferRequest );
static void It_I2c_Check_NackTransfer   ( const i2c_XferRequest_t * const xferRequest );
static void It_I2c_Check_WriteRead      ( i2c_SlaveAddr_t slaveAddr, uint8_t regAddr, i2c_DataCnt_t dataCnt );

static void It_I2c_XferCompleteCallback ( void );
static void It_I2c_ErrorCallback        ( i2c_XferErrorId_t errorId );

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

    /** DMA1 streams of I2C1 (channel 1) */
    #define IT_I2C_DMA_TX                   ( I2C_TX_DMA_I2C1_DMA1_STREAM6 )
    #define IT_I2C_DMA_RX                   ( I2C_RX_DMA_I2C1_DMA1_STREAM0 )

#elif defined(IT_BOARD_STM32F722xE) || \
      defined(IT_BOARD_STM32F756xG) || \
      defined(IT_BOARD_STM32F765xI) || \
      defined(IT_BOARD_STM32F767xI)

    /* NUCLEO-F722ZE / NUCLEO-F756ZG / NUCLEO-F767ZI (Nucleo-144) */

    /** I2C1 master on Arduino D15 (PB8, SCL) / D14 (PB9, SDA) */
    #define IT_I2C_PERIPH                   ( I2C_PERIPH_1 )
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_RCC_PCLK                 ( RCC_PERIPH_I2C1_PCLK1 )
    #define IT_I2C_RCC_HSI                  ( RCC_PERIPH_I2C1_HSI )
    #define IT_I2C_FMP_BIT                  ( SYSCFG_PMC_I2C1_FMP )

    /** I2C2 slave on PF1 (SCL) / PF0 (SDA), wired to the master pins */
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_REG                ( I2C2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C2_PF1 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C2_PF0 )
    #define IT_I2C_SLAVE_NVIC_EV            ( NVIC_PERIPH_IRQ_I2C2_EV )
    #define IT_I2C_SLAVE_NVIC_ER            ( NVIC_PERIPH_IRQ_I2C2_ER )
    #define IT_I2C_SLAVE_WIRING             "PB8 (D15) - PF1, PB9 (D14) - PF0"

    /** DMA1 streams of I2C1 (channel 1) */
    #define IT_I2C_DMA_TX                   ( I2C_TX_DMA_I2C1_DMA1_STREAM6 )
    #define IT_I2C_DMA_RX                   ( I2C_RX_DMA_I2C1_DMA1_STREAM0 )

#else
    #error "Board of I2c integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** Address of the emulated slave (7-bit and 10-bit) */
#define IT_I2C_SLAVE_ADDR                   ( 0x2Cu )
#define IT_I2C_SLAVE_ADDR10                 ( 0x1B7u )

/** Address of a not present slave (7-bit and 10-bit) */
#define IT_I2C_ABSENT_ADDR                  ( 0x50u )
#define IT_I2C_ABSENT_ADDR10                ( 0x2A5u )

/** Count of registers of the emulated slave (power of 2) */
#define IT_I2C_SLAVE_REG_CNT                ( 32u )

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

/** Slave interrupt sources (address match, data, NACK, STOP, errors) */
#define IT_I2C_SLAVE_IT                     ( I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Data handling configuration (copied by the module) */
static i2c_DataConfig_t             itI2c_DataConfig;

/** Transmit data of NACK transfers */
static const i2c_Data_t             itI2c_TxData[ 2u ] = { 0x00u, 0x10u };

/** Transmit buffer (register address followed by data) and receive buffer */
static i2c_Data_t                   itI2c_TxBuf[ IT_I2C_LONG_SIZE + 1u ];
static i2c_Data_t                   itI2c_RxBuf[ IT_I2C_LONG_SIZE ];

/** Counts of callback calls */
static volatile uint32_t            itI2c_CompleteCnt;
static volatile uint32_t            itI2c_ErrorCnt;

/** Error of the last error callback */
static volatile i2c_XferErrorId_t   itI2c_ErrorId;

/** Emulated register slave */
static it_I2cSlave_t                itI2c_Slave;

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itI2c_CompleteCnt = 0u;
    itI2c_ErrorCnt    = 0u;
    itI2c_ErrorId     = I2C_XFER_ERROR_CNT;

    for( uint32_t byteIdx = 0u; IT_I2C_LONG_SIZE > byteIdx; byteIdx++ )
    {
        itI2c_TxBuf[ byteIdx ] = 0u;
        itI2c_RxBuf[ byteIdx ] = 0u;
    }

    itI2c_TxBuf[ IT_I2C_LONG_SIZE ] = 0u;

    for( uint32_t regIdx = 0u; IT_I2C_SLAVE_REG_CNT > regIdx; regIdx++ )
    {
        itI2c_Slave.Regs[ regIdx ] = 0u;
    }

    itI2c_Slave.RegPtr     = 0u;
    itI2c_Slave.PtrPending = 0u;
    itI2c_Slave.AddrCnt    = 0u;
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

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_HZ );

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

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_PLUS_HZ );

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

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

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

    It_I2c_Init( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ );

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

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );
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

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_FAST_HZ );

    It_I2c_Check_NackTransfer( &request );
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

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );
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

    It_I2c_Init( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );
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

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( IT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );

    It_I2c_Check_NackTransfer( &request );
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

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    It_I2c_Check_NackTransfer( &request );

    itI2c_ErrorCnt = 0u;
    itI2c_ErrorId  = I2C_XFER_ERROR_CNT;
    It_I2c_Check_NackTransfer( &request );
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

    It_I2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( IT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );

    It_I2c_Check_NackTransfer( &request );

    itI2c_ErrorCnt = 0u;
    itI2c_ErrorId  = I2C_XFER_ERROR_CNT;
    It_I2c_Check_NackTransfer( &request );
}

/*------------------------- Transfers with slave -----------------------------*/

/**
 * \brief   Address-only transfer to the slave completes in all modes.
 *
 * \details Address-only transfer to the slave in POLL, ISR and DMA mode.
 *
 * \par Expected results
 * - Complete callback for every transfer, slave counted 3 address matches.
 */
void It_I2c_Set_XferStart_AddressOnlyAllModes_Completed( void )
{
    const i2c_XferMode_t    modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_XferRequest_t request    = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };

    It_I2c_Init_Slave( I2C_ADDR_MODE_7BIT );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        It_I2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        It_I2c_Check_Xfer( &request );
    }

    TEST_ASSERT_EQUAL_UINT32( 3u, itI2c_Slave.AddrCnt );
}


/**
 * \brief   Write and read back of 1, 2, 3, 5 and 16 bytes in all modes (100 kHz).
 *
 * \details For every mode (POLL, ISR, DMA) and count of bytes: register address +
 *          data are written, the slave register file is checked, then register
 *          address write + repeated START + read returns the data.
 *
 * \par Expected results
 * - Every transfer completes, slave registers and read data equal the written data
 *   (DMA channels are re-armed for every transfer).
 */
void It_I2c_Set_XferStart_WriteReadStandardModeAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 3u, 5u, IT_I2C_DATA_MAX };

    It_I2c_Init_Slave( I2C_ADDR_MODE_7BIT );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        It_I2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            It_I2c_Check_WriteRead( IT_I2C_SLAVE_ADDR, (uint8_t)( modeIdx + cntIdx ), cntLut[ cntIdx ] );
        }
    }
}


/**
 * \brief   Write and read back in all modes in Fast-mode (400 kHz).
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

    It_I2c_Init_Slave( I2C_ADDR_MODE_7BIT );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        It_I2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_FAST_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            It_I2c_Check_WriteRead( IT_I2C_SLAVE_ADDR, (uint8_t)( 8u + modeIdx + cntIdx ), cntLut[ cntIdx ] );
        }
    }
}


/**
 * \brief   Read-only transfers (no repeated START) return data from the selected
 *          register in all modes.
 *
 * \details Slave registers are preset, register 4 is selected by a 1 byte write,
 *          then 1, 2 and 6 bytes are read by read-only transfers, the next register
 *          is selected before every read (the slave transmitter prepares one byte
 *          more than the master reads) in POLL, ISR and DMA mode.
 *
 * \par Expected results
 * - Read data equal consecutive slave registers 4, 5 - 6, 7 - 12.
 */
void It_I2c_Set_XferStart_ReadOnlyAllModes_ConsecutiveRegisters( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_DataCnt_t  cntLut[ ]  = { 1u, 2u, 6u };

    It_I2c_Init_Slave( I2C_ADDR_MODE_7BIT );

    for( uint32_t regIdx = 0u; IT_I2C_SLAVE_REG_CNT > regIdx; regIdx++ )
    {
        itI2c_Slave.Regs[ regIdx ] = (uint8_t)( 0xC0u + regIdx );
    }

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        const i2c_XferRequest_t selRequest = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, 1u, I2C_NULL_PTR, 0u };
        uint8_t                 regAddr    = 4u;

        It_I2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );

        for( uint32_t cntIdx = 0u; ( sizeof( cntLut ) / sizeof( cntLut[ 0u ] ) ) > cntIdx; cntIdx++ )
        {
            const i2c_XferRequest_t readRequest = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, itI2c_RxBuf, cntLut[ cntIdx ] };

            itI2c_TxBuf[ 0u ] = regAddr;
            It_I2c_Check_Xfer( &selRequest );

            It_I2c_Check_Xfer( &readRequest );

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
 * - Every transfer completes, read data equal the slave registers.
 */
void It_I2c_Set_XferStart_10BitAllModes_DataEqual( void )
{
    const i2c_XferMode_t modeLut[ ] = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };

    It_I2c_Init_Slave( I2C_ADDR_MODE_10BIT );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        const i2c_XferRequest_t readRequest = { IT_I2C_SLAVE_ADDR10, I2C_NULL_PTR, 0u, itI2c_RxBuf, 2u };
        const uint8_t           regAddr     = (uint8_t)( 20u + modeIdx );

        It_I2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_STANDARD_HZ );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_AddrMode( IT_I2C_PERIPH, I2C_ADDR_MODE_10BIT ) );

        It_I2c_Check_WriteRead( IT_I2C_SLAVE_ADDR10, regAddr, 3u );

        /* Read-only transfer starts at the register selected here */
        itI2c_Slave.RegPtr = regAddr;
        It_I2c_Check_Xfer( &readRequest );
        TEST_ASSERT_EQUAL_HEX8( itI2c_Slave.Regs[ regAddr ],      itI2c_RxBuf[ 0u ] );
        TEST_ASSERT_EQUAL_HEX8( itI2c_Slave.Regs[ regAddr + 1u ], itI2c_RxBuf[ 1u ] );
    }
}


/**
 * \brief   Transfers longer than 255 bytes use NBYTES reload in all modes.
 *
 * \details Write of register address + 300 bytes, then register address write +
 *          repeated START + read of 300 bytes in POLL, ISR and DMA mode (400 kHz).
 *          The register file of the slave wraps after 32 registers - byte j carries
 *          the value of register j mod 32.
 *
 * \par Expected results
 * - Every transfer completes, slave registers contain the written values, read data
 *   equal the written data (all 300 bytes).
 */
void It_I2c_Set_XferStart_LongTransferAllModes_ReloadChunks( void )
{
    const i2c_XferMode_t    modeLut[ ]   = { I2C_XFER_MODE_POLL, I2C_XFER_MODE_ISR, I2C_XFER_MODE_DMA };
    const i2c_XferRequest_t writeRequest = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, (i2c_DataCnt_t)( IT_I2C_LONG_SIZE + 1u ), I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t readRequest  = { IT_I2C_SLAVE_ADDR, itI2c_TxBuf, 1u, itI2c_RxBuf, IT_I2C_LONG_SIZE };

    It_I2c_Init_Slave( I2C_ADDR_MODE_7BIT );

    for( uint32_t modeIdx = 0u; ( sizeof( modeLut ) / sizeof( modeLut[ 0u ] ) ) > modeIdx; modeIdx++ )
    {
        It_I2c_Init( modeLut[ modeIdx ], IT_I2C_FREQ_FAST_HZ );

        itI2c_TxBuf[ 0u ] = 0u;

        for( uint32_t byteIdx = 0u; IT_I2C_LONG_SIZE > byteIdx; byteIdx++ )
        {
            itI2c_TxBuf[ byteIdx + 1u ] = (i2c_Data_t)( 0xA5u ^ ( byteIdx & ( IT_I2C_SLAVE_REG_CNT - 1u ) ) ^ ( modeIdx << 6u ) );
            itI2c_RxBuf[ byteIdx ]      = 0u;
        }

        It_I2c_Check_Xfer( &writeRequest );

        for( uint32_t regIdx = 0u; IT_I2C_SLAVE_REG_CNT > regIdx; regIdx++ )
        {
            TEST_ASSERT_EQUAL_HEX8_MESSAGE( itI2c_TxBuf[ regIdx + 1u ], itI2c_Slave.Regs[ regIdx ], "Slave register" );
        }

        It_I2c_Check_Xfer( &readRequest );

        TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE( &itI2c_TxBuf[ 1u ], itI2c_RxBuf, IT_I2C_LONG_SIZE, "Read back data" );
    }
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes the master I2C1 on board pins with data handling (PCLK1 kernel clock).
 *
 * \param xferMode [in]: Data transfer mode
 * \param busFreq  [in]: SCL frequency [Hz]
 */
static void It_I2c_Init( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq )
{
    i2c_Config_t config;

    itI2c_DataConfig.XferMode             = xferMode;
    itI2c_DataConfig.TxDma                = IT_I2C_DMA_TX;
    itI2c_DataConfig.TxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.RxDma                = IT_I2C_DMA_RX;
    itI2c_DataConfig.RxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.IrqPriority          = IT_I2C_IRQ_PRIO;
    itI2c_DataConfig.XferCompleteCallback = It_I2c_XferCompleteCallback;
    itI2c_DataConfig.ErrorCallback        = It_I2c_ErrorCallback;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId   = IT_I2C_PERIPH;
    config.BusFreq    = busFreq;
    config.AddrMode   = I2C_ADDR_MODE_7BIT;
    config.DataConfig = &itI2c_DataConfig;
    config.SclPin     = IT_I2C_SCL_PIN;
    config.SdaPin     = IT_I2C_SDA_PIN;
    config.PinPull    = I2C_PIN_PULL_UP;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
}


/**
 * \brief Initializes I2C2 as register slave - clock, pins and timing by the I2c module
 *        (no data handling), own address and interrupts by LL (test set ISR). The board
 *        wiring is checked by an address-only transfer of the master (7-bit address).
 *
 * \note  The test is ignored on boards without the slave wiring or when the slave does
 *        not acknowledge its address (wiring missing).
 *
 * \param addrMode [in]: Own address mode of the slave (7-bit / 10-bit)
 */
static void It_I2c_Init_Slave( i2c_AddrMode_t addrMode )
{
#if ( 0u != IT_I2C_SLAVE_AVAILABLE )
    const i2c_XferRequest_t probeRequest = { IT_I2C_SLAVE_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };
    i2c_Config_t            config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = IT_I2C_SLAVE_PERIPH;
    config.SclPin   = IT_I2C_SLAVE_SCL_PIN;
    config.SdaPin   = IT_I2C_SLAVE_SDA_PIN;
    config.PinPull  = I2C_PIN_PULL_UP;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    It_I2c_Set_SlaveAddr( I2C_ADDR_MODE_7BIT );
    SET_BIT( IT_I2C_SLAVE_REG->CR1, IT_I2C_SLAVE_IT );

    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Handler( IT_I2C_SLAVE_NVIC_EV, It_I2c_Slave_IsrHandler ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Handler( IT_I2C_SLAVE_NVIC_ER, It_I2c_Slave_IsrHandler ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Prio( IT_I2C_SLAVE_NVIC_EV, IT_I2C_SLAVE_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Prio( IT_I2C_SLAVE_NVIC_ER, IT_I2C_SLAVE_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Active( IT_I2C_SLAVE_NVIC_EV ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Active( IT_I2C_SLAVE_NVIC_ER ) );

    /* Board wiring check - the slave acknowledges its 7-bit address */
    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, &probeRequest ) );
    It_I2c_Wait_XferEnd();

    if( 1u != itI2c_CompleteCnt )
    {
        TEST_IGNORE_MESSAGE( "I2C2 slave does not answer - wiring " IT_I2C_SLAVE_WIRING " missing" );
    }
    else
    {
        /* Slave is connected */
    }

    It_I2c_Set_SlaveAddr( addrMode );

    itI2c_CompleteCnt   = 0u;
    itI2c_ErrorCnt      = 0u;
    itI2c_Slave.AddrCnt = 0u;
#else
    (void)addrMode;
    (void)It_I2c_Slave_IsrHandler;
    (void)It_I2c_Set_SlaveAddr;

    TEST_IGNORE_MESSAGE( "Board has no I2C slave wiring" );
#endif
}


/**
 * \brief Sets own address 1 of the slave (OA1 is written with OA1EN = 0).
 *
 * \param addrMode [in]: Own address mode of the slave (7-bit / 10-bit)
 */
static void It_I2c_Set_SlaveAddr( i2c_AddrMode_t addrMode )
{
#if ( 0u != IT_I2C_SLAVE_AVAILABLE )
    LL_I2C_DisableOwnAddress1( IT_I2C_SLAVE_REG );

    if( I2C_ADDR_MODE_10BIT == addrMode )
    {
        LL_I2C_SetOwnAddress1( IT_I2C_SLAVE_REG, IT_I2C_SLAVE_ADDR10, LL_I2C_OWNADDRESS1_10BIT );
    }
    else
    {
        LL_I2C_SetOwnAddress1( IT_I2C_SLAVE_REG, IT_I2C_SLAVE_ADDR << 1u, LL_I2C_OWNADDRESS1_7BIT );
    }

    LL_I2C_EnableOwnAddress1( IT_I2C_SLAVE_REG );
#else
    (void)addrMode;
#endif
}


/**
 * \brief Interrupt of the emulated register slave (event and error).
 *
 * - RXNE: register selection or data stored with auto increment
 * - ADDR: write direction - next byte selects the register; read direction - TXDR
 *   is flushed (byte prepared for a previous read is dropped); flag cleared
 * - TXIS (transmitter): data of the selected register with auto increment
 * - NACKF (NACK of the last read byte), STOPF, BERR / ARLO / OVR: cleared
 */
static void It_I2c_Slave_IsrHandler( void )
{
#if ( 0u != IT_I2C_SLAVE_AVAILABLE )
    I2C_TypeDef * const slaveReg = IT_I2C_SLAVE_REG;
    const uint32_t      isr      = LL_I2C_ReadReg( slaveReg, ISR );

    if( 0u != ( isr & ( I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR ) ) )
    {
        LL_I2C_WriteReg( slaveReg, ICR, I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF );
    }
    else
    {
        /* No bus error */
    }

    /* Received byte first - it may be pending together with ADDR of a repeated START */
    if( 0u != ( isr & I2C_ISR_RXNE ) )
    {
        const uint8_t rxData = LL_I2C_ReceiveData8( slaveReg );

        if( 0u != itI2c_Slave.PtrPending )
        {
            itI2c_Slave.RegPtr     = rxData;
            itI2c_Slave.PtrPending = 0u;
        }
        else
        {
            itI2c_Slave.Regs[ itI2c_Slave.RegPtr & ( IT_I2C_SLAVE_REG_CNT - 1u ) ] = rxData;
            itI2c_Slave.RegPtr++;
        }
    }
    else
    {
        /* No received byte */
    }

    if( 0u != ( isr & I2C_ISR_ADDR ) )
    {
        itI2c_Slave.AddrCnt++;

        if( 0u != ( isr & I2C_ISR_DIR ) )
        {
            /* Read transfer (slave transmitter) - stale byte of a previous read is flushed */
            itI2c_Slave.PtrPending = 0u;
            LL_I2C_ClearFlag_TXE( slaveReg );
        }
        else
        {
            itI2c_Slave.PtrPending = 1u;
        }

        LL_I2C_ClearFlag_ADDR( slaveReg );
    }
    else if( 0u != ( isr & I2C_ISR_TXIS ) )
    {
        LL_I2C_TransmitData8( slaveReg, itI2c_Slave.Regs[ itI2c_Slave.RegPtr & ( IT_I2C_SLAVE_REG_CNT - 1u ) ] );
        itI2c_Slave.RegPtr++;
    }
    else
    {
        /* No data event */
    }

    if( 0u != ( isr & I2C_ISR_NACKF ) )
    {
        LL_I2C_ClearFlag_NACK( slaveReg );
    }
    else
    {
        /* Byte acknowledged by the master */
    }

    if( 0u != ( isr & I2C_ISR_STOPF ) )
    {
        LL_I2C_ClearFlag_STOP( slaveReg );
    }
    else
    {
        /* No STOP condition */
    }
#endif
}


/**
 * \brief Waits until a callback is called or timeout (I2c_Task called - polling mode).
 */
static void It_I2c_Wait_XferEnd( void )
{
    for( uint32_t loopIdx = 0u;
         ( IT_I2C_WAIT_LOOPS > loopIdx ) &&
         ( 0u == ( itI2c_CompleteCnt + itI2c_ErrorCnt ) );
         loopIdx++ )
    {
        I2c_Task();
    }
}


/**
 * \brief Executes a transfer and checks its successful end.
 *
 * \param xferRequest [in]: Transfer request
 */
static void It_I2c_Check_Xfer( const i2c_XferRequest_t * const xferRequest )
{
    i2c_FunctionState_t xferState = I2C_FUNCTION_ACTIVE;
    i2c_XferErrorId_t   xferError = I2C_XFER_ERROR_CNT;

    itI2c_CompleteCnt = 0u;
    itI2c_ErrorCnt    = 0u;
    itI2c_ErrorId     = I2C_XFER_ERROR_CNT;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_XferStart( IT_I2C_PERIPH, xferRequest ) );
    It_I2c_Wait_XferEnd();

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 0u, itI2c_ErrorCnt, "Transfer error reported" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itI2c_CompleteCnt, "Transfer not completed" );

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
 * \brief Writes data to slave registers, checks the slave register file and reads the
 *        data back (register address write, repeated START, read).
 *
 * \param slaveAddr [in]: Slave address
 * \param regAddr   [in]: First register
 * \param dataCnt   [in]: Count of data bytes (1 - IT_I2C_DATA_MAX)
 */
static void It_I2c_Check_WriteRead( i2c_SlaveAddr_t slaveAddr, uint8_t regAddr, i2c_DataCnt_t dataCnt )
{
    const i2c_XferRequest_t writeRequest = { slaveAddr, itI2c_TxBuf, (i2c_DataCnt_t)( dataCnt + 1u ), I2C_NULL_PTR, 0u };
    const i2c_XferRequest_t readRequest  = { slaveAddr, itI2c_TxBuf, 1u, itI2c_RxBuf, dataCnt };

    TEST_ASSERT_LESS_OR_EQUAL_UINT32( IT_I2C_DATA_MAX, dataCnt );

    itI2c_TxBuf[ 0u ] = regAddr;

    for( uint32_t byteIdx = 0u; dataCnt > byteIdx; byteIdx++ )
    {
        itI2c_TxBuf[ byteIdx + 1u ] = (i2c_Data_t)( ( regAddr * 7u ) + ( byteIdx * 13u ) + dataCnt );
        itI2c_RxBuf[ byteIdx ]      = 0u;
    }

    It_I2c_Check_Xfer( &writeRequest );

    for( uint32_t byteIdx = 0u; dataCnt > byteIdx; byteIdx++ )
    {
        TEST_ASSERT_EQUAL_HEX8_MESSAGE( itI2c_TxBuf[ byteIdx + 1u ], itI2c_Slave.Regs[ ( regAddr + byteIdx ) & ( IT_I2C_SLAVE_REG_CNT - 1u ) ], "Slave register" );
    }

    It_I2c_Check_Xfer( &readRequest );

    TEST_ASSERT_EQUAL_HEX8_ARRAY_MESSAGE( &itI2c_TxBuf[ 1u ], itI2c_RxBuf, dataCnt, "Read back data" );
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
