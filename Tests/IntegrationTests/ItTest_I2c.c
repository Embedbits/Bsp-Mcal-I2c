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
 * master transfers to a not present slave (address NACK), master transfers to the
 * slave of the module in polling, interrupt and DMA mode and deinitialization.
 *
 * Master tests without a slave: the slave peripheral is not initialized, every
 * address is not acknowledged.
 *
 * Master + slave tests: the master I2C peripheral of the board is wired to the slave
 * I2C peripheral of the same MCU (SCL - SCL, SDA - SDA, external pull-up resistors
 * 4.7 kOhm, pins of the boards in the board blocks below). Fixture of the board
 * (EmBi_Platform IntegrationTesting, "Board fixtures"): the wiring is feature
 * I2C_LOOP - the master + slave tests are ignored on a board with a fixture without
 * it (a board without a fixture is expected to be wired). Both peripherals stretch
 * the clock, the master and the slave can share one CPU in every mode combination.
 *
 * Used resources: master I2C1, slave I2C2 / I2C4 on board specific pins; GPDMA1
 * channels 5 / 6 (master DMA mode) and 3 / 4 (slave DMA mode).
 *
 * \note The host halts the core for a few milliseconds on every mailbox access
 *       (probe-rs). Both peripherals stretch the clock, no byte is lost.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "I2c_Port.h"                       /* Module under test              */
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
    volatile uint32_t           AddrCnt;      /**< Address callbacks                     */
    volatile i2c_SlaveDir_t     LastDir;      /**< Direction of the last address match   */
    volatile i2c_SlaveAddr_t    LastAddr;     /**< Matched address of the last match     */
    volatile i2c_FunctionState_t RegMode;     /**< Register device pattern is emulated   */
    i2c_SlaveResult_t           Result;       /**< Result of the last complete callback  */
}   it_I2cSlave_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void ItI2c_Init                 ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq );
static void ItI2c_Init_Master          ( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq, i2c_AddrMode_t addrMode );
static void ItI2c_Init_Slave           ( i2c_XferMode_t xferMode,
                                         i2c_FreqHz_t busFreq,
                                         const i2c_SlaveConfig_t * const slaveConfig );
static void ItI2c_Get_SlaveConfig      ( i2c_SlaveConfig_t * const slaveConfig );
static void ItI2c_Wait_XferEnd         ( void );
static void ItI2c_Wait_MasterSlaveEnd  ( void );
static void ItI2c_Check_NackTransfer   ( const i2c_XferRequest_t * const xferRequest );
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
#if defined(IT_BOARD_STM32H503xB)

    /** NUCLEO-H503RB: master I2C1 PB6 (SCL) / PB7 (SDA) -> slave I2C2 PB10 (SCL) / PB8 (SDA) */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB6 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB7 )
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C2_PB10 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C2_PB8 )

#elif defined(IT_BOARD_STM32H523xE) || \
      defined(IT_BOARD_STM32H533xE)

    /** NUCLEO-H533RE: master I2C1 PB8 (SCL) / PB7 (SDA) -> slave I2C2 PB10 (SCL) / PB12 (SDA) */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB7 )
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C2_PB10 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C2_PB12 )

#elif defined(IT_BOARD_STM32H562xI) || \
      defined(IT_BOARD_STM32H563xI) || \
      defined(IT_BOARD_STM32H573xI)

    /** NUCLEO-H563ZI: master I2C1 PB8 (SCL) / PB9 (SDA) -> slave I2C4 PF5 (SCL) / PB7 (SDA) */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_4 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C4_PF5 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C4_PB7 )

#elif defined(IT_BOARD_STM32H5E5xJ)

    /** I2C1 on PB8 (SCL) / PB9 (SDA) - no wiring of a slave known for the board */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )
    #define IT_I2C_SLAVE_AVAILABLE          ( 0u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_UNUSED )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_UNUSED )

#else
    #error "Board of I2c integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** Master I2C peripheral */
#define IT_I2C_PERIPH                       ( I2C_PERIPH_1 )

/** Address of a not present slave (EEPROM address range) */
#define IT_I2C_SLAVE_ADDR                   ( 0x50u )

/** Own address 1 of the slave of the module (7-bit) */
#define IT_I2C_OWN_ADDR                     ( 0x2Cu )

/** Own address 1 of the slave of the module (10-bit) */
#define IT_I2C_OWN_ADDR10                   ( 0x2B7u )

/** Own address 2 of the slave of the module */
#define IT_I2C_OWN_ADDR2                    ( 0x3Au )

/** Standard-mode and Fast-mode SCL frequency [Hz] */
#define IT_I2C_FREQ_STANDARD_HZ             ( 100000u )
#define IT_I2C_FREQ_FAST_HZ                 ( 400000u )

/** Tolerance of the SCL frequency read back from timing register [%] */
#define IT_I2C_FREQ_TOL_PCT                 ( 10u )

/** Count of percent in one unit */
#define IT_I2C_PERCENT                      ( 100u )

/** Maximal count of wait loop iterations (I2c_Task called) */
#define IT_I2C_WAIT_LOOPS                   ( 500000u )

/** Size of the data buffers (longer than one NBYTES chunk of 255 bytes) */
#define IT_I2C_BUF_SIZE                     ( 300u )

/** Count of bytes of a short transfer */
#define IT_I2C_SHORT_SIZE                   ( 8u )

/** Count of registers of the emulated register device */
#define IT_I2C_REG_CNT                      ( 16u )

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

/** Transmit and receive data of the master tests without a slave */
static const i2c_Data_t             itI2c_TxData[ 2u ] = { 0x00u, 0x10u };
static i2c_Data_t                   itI2c_RxData[ 4u ];

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

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_FAST_HZ );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_FAST_HZ * IT_I2C_FREQ_TOL_PCT ) / IT_I2C_PERCENT,
                               IT_I2C_FREQ_FAST_HZ,
                               busFreq );
}


/**
 * \brief   Idle bus with pull-ups is reported free.
 *
 * \details Initializes I2C1 (lines pulled up, no transfer) and reads bus state.
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
 * \details Calls I2c_Init() with NULL, bus frequency 0, invalid peripheral and a
 *          slave with a reserved own address.
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

    /* Default own address 0x00 is the reserved general call address */
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = IT_I2C_PERIPH;
    config.Role     = I2C_ROLE_SLAVE;
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

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

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

    ItI2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );
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

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_FAST_HZ );

    ItI2c_Check_NackTransfer( &request );
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

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );
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

    ItI2c_Init( I2C_XFER_MODE_DMA, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );
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

    ItI2c_Init( I2C_XFER_MODE_ISR, IT_I2C_FREQ_STANDARD_HZ );

    ItI2c_Check_NackTransfer( &request );

    itI2c_Master.ErrorCnt = 0u;
    itI2c_Master.ErrorId  = I2C_XFER_ERROR_CNT;
    ItI2c_Check_NackTransfer( &request );
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
    TEST_ASSERT_EQUAL_UINT32( 2u, itI2c_Slave.AddrCnt );
    TEST_ASSERT_EQUAL( I2C_SLAVE_DIR_READ, itI2c_Slave.LastDir );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itI2c_Regs[ IT_I2C_REG_ADDR ], itI2c_MasterRx, IT_I2C_REG_READ_CNT );
}


/**
 * \brief   Register read of the master is answered by the slave in DMA mode.
 *
 * \details As the polling register read, slave in DMA mode (GPDMA1 channels 3 / 4),
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
    TEST_ASSERT_EQUAL_UINT32( 2u, itI2c_Slave.AddrCnt );
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
 * - Master read: 2 bytes of the buffer, then 4 fill bytes (GPDMA transfer complete
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
 * \brief Initializes the master I2C1 with internal pull-ups and data handling.
 *
 * \param xferMode [in]: Data transfer mode
 * \param busFreq  [in]: SCL frequency [Hz]
 * \param addrMode [in]: Addressing mode of the master
 */
static void ItI2c_Init_Master( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq, i2c_AddrMode_t addrMode )
{
    i2c_Config_t config;

    itI2c_DataConfig.XferMode                  = xferMode;
    itI2c_DataConfig.TxDmaPeriphId             = I2C_DMA_PERIPH_1;
    itI2c_DataConfig.TxDmaChannelId            = I2C_DMA_CHANNEL_5;
    itI2c_DataConfig.TxDmaPriority             = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.RxDmaPeriphId             = I2C_DMA_PERIPH_1;
    itI2c_DataConfig.RxDmaChannelId            = I2C_DMA_CHANNEL_6;
    itI2c_DataConfig.RxDmaPriority             = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.IrqPriority               = IT_I2C_IRQ_PRIO;
    itI2c_DataConfig.XferCompleteCallback      = ItI2c_XferCompleteCallback;
    itI2c_DataConfig.ErrorCallback             = ItI2c_ErrorCallback;
    itI2c_DataConfig.SlaveAddrCallback         = I2C_NULL_PTR;
    itI2c_DataConfig.SlaveXferCompleteCallback = I2C_NULL_PTR;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId      = IT_I2C_PERIPH;
    config.BusFreq       = busFreq;
    config.AnalogFilter  = I2C_ANALOG_FILTER_ENABLED;
    config.DigitalFilter = I2C_DIGITAL_FILTER_OFF;
    config.AddrMode      = addrMode;
    config.DataConfig    = &itI2c_DataConfig;
    config.SclPin        = IT_I2C_SCL_PIN;
    config.SdaPin        = IT_I2C_SDA_PIN;
    config.PinPull       = I2C_PIN_PULL_UP;

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
    itI2c_SlaveDataConfig.TxDmaPeriphId             = I2C_DMA_PERIPH_1;
    itI2c_SlaveDataConfig.TxDmaChannelId            = I2C_DMA_CHANNEL_3;
    itI2c_SlaveDataConfig.TxDmaPriority             = I2C_DMA_PRIORITY_HIGH;
    itI2c_SlaveDataConfig.RxDmaPeriphId             = I2C_DMA_PERIPH_1;
    itI2c_SlaveDataConfig.RxDmaChannelId            = I2C_DMA_CHANNEL_4;
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

    /* STOP condition was generated after NACK - bus is released */
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusState( IT_I2C_PERIPH, &busBusy ) );
    TEST_ASSERT_EQUAL( I2C_FLAG_INACTIVE, busBusy );
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
 *        read phase starts at the register received in the write phase.
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
        ( I2C_SLAVE_DIR_READ  == slaveDir            ) &&
        ( IT_I2C_REG_CNT       > itI2c_SlaveRx[ 0u ] )    )
    {
        const i2c_DataCnt_t     regAddr     = itI2c_SlaveRx[ 0u ];
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
}


/**
 * \brief Slave transfer complete callback.
 *
 * \param slaveResult [in]: Result of the slave transfer
 */
static void ItI2c_SlaveCompleteCallback( const i2c_SlaveResult_t * const slaveResult )
{
    itI2c_Slave.Result = *slaveResult;
    itI2c_SlaveCb.CompleteCnt++;
}
