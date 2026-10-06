/**
 * \author Mr.Nobody
 * \file ItTest_I2c.c
 * \ingroup I2c
 * \brief Integration tests of I2C module on target (STM32F4, I2C v1).
 *
 * I2c module runs on the MCU together with real RCC, NVIC, GPIO and DMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): peripheral and pin initialization, SCL timing
 * calculated from the real PCLK1, idle bus state, master sequencing of the
 * hardware (address, ACK / NACK, repeated START, BTF reception of 1, 2, 3 and
 * more bytes, 10-bit addressing) in polling, interrupt and DMA mode and
 * deinitialization.
 *
 * Master: I2C1 with internal pull-ups. Address 0x50 (7-bit) and 0x2A5 (10-bit)
 * are not present on the bus (NACK).
 *
 * Slave (boards with wiring only, otherwise the slave tests are ignored): I2C2
 * of the same MCU is connected to the master bus and emulates a register device
 * in the interrupt of the test set (LL access): the first written byte selects
 * the register, following written bytes are stored and read bytes are returned
 * from the selected register with auto increment.
 *
 * Board NUCLEO_F411RE (STM32F411xE): I2C1 master PB8 (SCL) / PB9 (SDA), I2C2 slave PB10 (SCL) /
 * PB3 (SDA), wiring PB8 - PB10 and PB9 - PB3. DMA1 stream 6 (TX) / stream 0 (RX).
 * Board STM32F4DISCOVERY (STM32F407): I2C1 master PB8 / PB9, I2C2 slave PB10 / PB11,
 * wiring PB8 - PB10 and PB9 - PB11.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "I2c_Port.h"                       /* Module under test              */
#include "Nvic_Port.h"                      /* Interrupt of the slave         */
#include "Stm32_i2c.h"                      /* Slave registers (LL)           */
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
static void It_I2c_Slave_IsrHandler     ( void );
static void It_I2c_Wait_XferEnd         ( void );
static void It_I2c_Check_Xfer           ( const i2c_XferRequest_t * const xferRequest );
static void It_I2c_Check_NackTransfer   ( const i2c_XferRequest_t * const xferRequest );
static void It_I2c_Check_WriteRead      ( i2c_SlaveAddr_t slaveAddr, uint8_t regAddr, i2c_DataCnt_t dataCnt );

static void It_I2c_XferCompleteCallback ( void );
static void It_I2c_ErrorCallback        ( i2c_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
#if defined(IT_BOARD_NUCLEO_F411RE)

    /** I2C1 master on PB8 (SCL) / PB9 (SDA) */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )

    /** I2C2 slave on PB10 (SCL) / PB3 (SDA), wired to the master pins */
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_REG                ( I2C2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C2_PB10 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C2_PB3 )
    #define IT_I2C_SLAVE_NVIC_EV            ( NVIC_PERIPH_IRQ_I2C2_EV )
    #define IT_I2C_SLAVE_NVIC_ER            ( NVIC_PERIPH_IRQ_I2C2_ER )

    /** DMA1 streams of I2C1 (DMA1 request mapping, channel 1) */
    #define IT_I2C_DMA_TX_STREAM            ( I2C_DMA_CHANNEL_6 )
    #define IT_I2C_DMA_RX_STREAM            ( I2C_DMA_CHANNEL_0 )

#elif defined(IT_BOARD_STM32F4DISCOVERY)

    /** I2C1 master on PB8 (SCL) / PB9 (SDA) */
    #define IT_I2C_SCL_PIN                  ( I2C_SCL_PIN_I2C1_PB8 )
    #define IT_I2C_SDA_PIN                  ( I2C_SDA_PIN_I2C1_PB9 )

    /** I2C2 slave on PB10 (SCL) / PB11 (SDA), wired to the master pins */
    #define IT_I2C_SLAVE_AVAILABLE          ( 1u )
    #define IT_I2C_SLAVE_PERIPH             ( I2C_PERIPH_2 )
    #define IT_I2C_SLAVE_REG                ( I2C2 )
    #define IT_I2C_SLAVE_SCL_PIN            ( I2C_SCL_PIN_I2C2_PB10 )
    #define IT_I2C_SLAVE_SDA_PIN            ( I2C_SDA_PIN_I2C2_PB11 )
    #define IT_I2C_SLAVE_NVIC_EV            ( NVIC_PERIPH_IRQ_I2C2_EV )
    #define IT_I2C_SLAVE_NVIC_ER            ( NVIC_PERIPH_IRQ_I2C2_ER )

    /** DMA1 streams of I2C1 (DMA1 request mapping, channel 1) */
    #define IT_I2C_DMA_TX_STREAM            ( I2C_DMA_CHANNEL_6 )
    #define IT_I2C_DMA_RX_STREAM            ( I2C_DMA_CHANNEL_0 )

#else
    #error "Board of I2c integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** I2C peripheral under test */
#define IT_I2C_PERIPH                       ( I2C_PERIPH_1 )

/** Address of the emulated slave (7-bit and 10-bit) */
#define IT_I2C_SLAVE_ADDR                   ( 0x2Cu )
#define IT_I2C_SLAVE_ADDR10                 ( 0x1B7u )

/** Address of a not present slave (7-bit and 10-bit) */
#define IT_I2C_ABSENT_ADDR                  ( 0x50u )
#define IT_I2C_ABSENT_ADDR10                ( 0x2A5u )

/** Count of registers of the emulated slave (power of 2) */
#define IT_I2C_SLAVE_REG_CNT                ( 32u )

/** Bit 14 of OAR1 required by RM */
#define IT_I2C_OAR1_BIT14                   ( 0x4000u )

/** Standard-mode and Fast-mode SCL frequency [Hz] */
#define IT_I2C_FREQ_STANDARD_HZ             ( 100000u )
#define IT_I2C_FREQ_FAST_HZ                 ( 400000u )

/** Tolerance of the SCL frequency read back from CCR [%] */
#define IT_I2C_FREQ_TOL_PCT                 ( 10u )

/** Maximal count of wait loop iterations (I2c_Task called) */
#define IT_I2C_WAIT_LOOPS                   ( 500000u )

/** Interrupt priority of the master and of the slave */
#define IT_I2C_IRQ_PRIO                     ( 5u )
#define IT_I2C_SLAVE_IRQ_PRIO               ( 4u )

/** Count of data bytes of the longest test transfer */
#define IT_I2C_DATA_MAX                     ( 16u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Data handling configuration (copied by the module) */
static i2c_DataConfig_t             itI2c_DataConfig;

/** Transmit data of NACK transfers */
static const i2c_Data_t             itI2c_TxData[ 2u ] = { 0x00u, 0x10u };

/** Transmit buffer (register address followed by data) and receive buffer */
static i2c_Data_t                   itI2c_TxBuf[ IT_I2C_DATA_MAX + 1u ];
static i2c_Data_t                   itI2c_RxBuf[ IT_I2C_DATA_MAX + 1u ];

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

    for( uint32_t byteIdx = 0u; ( IT_I2C_DATA_MAX + 1u ) > byteIdx; byteIdx++ )
    {
        itI2c_TxBuf[ byteIdx ] = 0u;
        itI2c_RxBuf[ byteIdx ] = 0u;
    }

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
 * \details Initializes I2C1 (polling mode, 100 kHz, board pins) and reads the
 *          configuration back.
 *
 * \par Expected results
 * - Bus frequency 100 kHz +- 10 % (calculated from real PCLK1).
 * - Default filters of the device, digital filter off, 7-bit addressing.
 * - Peripheral is enabled.
 */
void It_I2c_Init_StandardMode_ConfigurationReadBack( void )
{
    i2c_Config_t        defConfig;
    i2c_FreqHz_t        busFreq       = 0u;
    i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_CNT;
    i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_CNT;
    i2c_AddrMode_t      addrMode      = I2C_ADDR_MODE_CNT;
    i2c_FlagState_t     periphState   = I2C_FLAG_INACTIVE;

    It_I2c_Init( I2C_XFER_MODE_POLL, IT_I2C_FREQ_STANDARD_HZ );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &defConfig ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_BusFreq( IT_I2C_PERIPH, &busFreq ) );
    TEST_ASSERT_UINT32_WITHIN( ( IT_I2C_FREQ_STANDARD_HZ * IT_I2C_FREQ_TOL_PCT ) / 100u, IT_I2C_FREQ_STANDARD_HZ, busFreq );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_AnalogFilter( IT_I2C_PERIPH, &analogFilter ) );
    TEST_ASSERT_EQUAL( defConfig.AnalogFilter, analogFilter );

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
 * \details Initializes I2C1 and reads bus state.
 *
 * \par Expected results
 * - Bus busy flag is INACTIVE.
 */
void It_I2c_Get_BusState_IdleBus_BusFree( void )
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
    i2c_XferRequest_t request     = { IT_I2C_ABSENT_ADDR, I2C_NULL_PTR, 0u, I2C_NULL_PTR, 0u };

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
 *   (reception of 1 byte, 2 bytes with POS, 3 and more bytes with BTF procedure,
 *   DMA with LAST and single byte by buffer interrupt).
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
 *          back of 3 bytes and read-only transfer of 2 bytes (address in write
 *          direction, repeated START, read header) in POLL, ISR and DMA mode.
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

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes I2C1 master on board pins with data handling.
 *
 * \param xferMode [in]: Data transfer mode
 * \param busFreq  [in]: SCL frequency [Hz]
 */
static void It_I2c_Init( i2c_XferMode_t xferMode, i2c_FreqHz_t busFreq )
{
    i2c_Config_t config;

    itI2c_DataConfig.XferMode             = xferMode;
    itI2c_DataConfig.TxDmaPeriphId        = I2C_DMA_PERIPH_1;
    itI2c_DataConfig.TxDmaChannelId       = IT_I2C_DMA_TX_STREAM;
    itI2c_DataConfig.TxDmaPriority        = I2C_DMA_PRIORITY_LOW;
    itI2c_DataConfig.RxDmaPeriphId        = I2C_DMA_PERIPH_1;
    itI2c_DataConfig.RxDmaChannelId       = IT_I2C_DMA_RX_STREAM;
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
 * \brief Initializes I2C2 as register slave - clock, pins and FREQ by the I2c module
 *        (no data handling), own address, ACK and interrupts by LL (test set ISR).
 *
 * \note  The test is ignored on boards without the slave wiring.
 *
 * \param addrMode [in]: Own address mode of the slave (7-bit / 10-bit)
 */
static void It_I2c_Init_Slave( i2c_AddrMode_t addrMode )
{
#if ( 0u != IT_I2C_SLAVE_AVAILABLE )
    i2c_Config_t config;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_DefaultConfig( &config ) );
    config.PeriphId = IT_I2C_SLAVE_PERIPH;
    config.SclPin   = IT_I2C_SLAVE_SCL_PIN;
    config.SdaPin   = IT_I2C_SLAVE_SDA_PIN;
    config.PinPull  = I2C_PIN_PULL_UP;

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    if( I2C_ADDR_MODE_10BIT == addrMode )
    {
        IT_I2C_SLAVE_REG->OAR1 = IT_I2C_OAR1_BIT14 | I2C_OAR1_ADDMODE | IT_I2C_SLAVE_ADDR10;
    }
    else
    {
        IT_I2C_SLAVE_REG->OAR1 = IT_I2C_OAR1_BIT14 | ( IT_I2C_SLAVE_ADDR << 1u );
    }

    LL_I2C_AcknowledgeNextData( IT_I2C_SLAVE_REG, LL_I2C_ACK );
    SET_BIT( IT_I2C_SLAVE_REG->CR2, I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN );

    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Handler( IT_I2C_SLAVE_NVIC_EV, It_I2c_Slave_IsrHandler ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Handler( IT_I2C_SLAVE_NVIC_ER, It_I2c_Slave_IsrHandler ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Prio( IT_I2C_SLAVE_NVIC_EV, IT_I2C_SLAVE_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Prio( IT_I2C_SLAVE_NVIC_ER, IT_I2C_SLAVE_IRQ_PRIO ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Active( IT_I2C_SLAVE_NVIC_EV ) );
    TEST_ASSERT_EQUAL( NVIC_REQUEST_OK, Nvic_Set_PeriphIrq_Active( IT_I2C_SLAVE_NVIC_ER ) );
#else
    (void)addrMode;
    (void)It_I2c_Slave_IsrHandler;

    TEST_IGNORE_MESSAGE( "Board has no I2C slave wiring" );
#endif
}


/**
 * \brief Interrupt of the emulated register slave (event and error).
 *
 * - ADDR: cleared (SR1 + SR2 read), write direction - next byte selects the register,
 *   buffer interrupt enabled
 * - RXNE: register selection or data stored with auto increment
 * - TXE (transmitter): data of the selected register with auto increment
 * - AF (NACK of the last read byte): cleared, buffer interrupt disabled until the next
 *   address match
 * - STOPF: cleared (SR1 read + CR1 write), bus errors cleared
 */
static void It_I2c_Slave_IsrHandler( void )
{
#if ( 0u != IT_I2C_SLAVE_AVAILABLE )
    I2C_TypeDef * const slaveReg = IT_I2C_SLAVE_REG;
    const uint32_t      sr1      = slaveReg->SR1;

    if( 0u != ( sr1 & ( I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR ) ) )
    {
        CLEAR_BIT( slaveReg->SR1, I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR );
    }

    if( 0u != ( sr1 & I2C_SR1_AF ) )
    {
        CLEAR_BIT( slaveReg->SR1, I2C_SR1_AF );
        CLEAR_BIT( slaveReg->CR2, I2C_CR2_ITBUFEN );
    }

    /* Received byte first - it may be pending together with ADDR of a repeated START */
    if( 0u != ( sr1 & I2C_SR1_RXNE ) )
    {
        const uint8_t rxData = (uint8_t)slaveReg->DR;

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

    if( 0u != ( sr1 & I2C_SR1_ADDR ) )
    {
        const uint32_t sr2 = slaveReg->SR2;

        itI2c_Slave.AddrCnt++;
        itI2c_Slave.PtrPending = ( 0u == ( sr2 & I2C_SR2_TRA ) ) ? 1u : 0u;
        SET_BIT( slaveReg->CR2, I2C_CR2_ITBUFEN );
    }
    else if( ( 0u != ( sr1 & I2C_SR1_TXE ) ) && ( 0u != ( slaveReg->SR2 & I2C_SR2_TRA ) ) )
    {
        slaveReg->DR = itI2c_Slave.Regs[ itI2c_Slave.RegPtr & ( IT_I2C_SLAVE_REG_CNT - 1u ) ];
        itI2c_Slave.RegPtr++;
    }
    else
    {
        /* No data event */
    }

    if( 0u != ( sr1 & I2C_SR1_STOPF ) )
    {
        /* STOPF is cleared by SR1 read followed by CR1 write */
        SET_BIT( slaveReg->CR1, I2C_CR1_PE );
    }
#endif
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

    /* STOP condition is requested after NACK - bus is released once it is sent */
    for( uint32_t loopIdx = 0u; ( IT_I2C_WAIT_LOOPS > loopIdx ) && ( I2C_FLAG_INACTIVE != busBusy ); loopIdx++ )
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
