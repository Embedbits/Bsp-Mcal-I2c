/**
 * \author Mr.Nobody
 * \file Test_I2c.c
 * \ingroup I2c
 * \brief Unit tests of Inter-Integrated Circuit (I2C) module (STM32L4 / STM32L4+).
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
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_i2c.h"                      /* I2C registers definition       */
#include "Stm32_system.h"                   /* SYSCFG registers definition    */
/* ============================= TYPEDEFS =================================== */

/** \brief Row of the expected DMA channel lists (STM32CubeMX DMA database - RM0351 / RM0394 / RM0432 request mapping) */
typedef struct
{
    uint32_t            Item;      /**< Item of the list                                  */
    uint32_t            Direction; /**< Direction (UT_I2C_DMA_DIR_TX / UT_I2C_DMA_DIR_RX) */
    i2c_PeriphId_t      PeriphId;  /**< I2C peripheral                                    */
    uint32_t            DmaId;     /**< DMA peripheral index (0 - DMA1, 1 - DMA2)         */
    uint32_t            Channel;   /**< DMA channel index (0 - channel 1)                 */
    uint32_t            Selection; /**< DMA_CSELR request selection (0 with DMAMUX1)      */
    dma_PeriphReqId_t   Request;   /**< DMA request of the peripheral and direction       */
}   utI2c_DmaRow_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_I2c_NvicSetHandlerStub   ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetClkSrcStub     ( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetClkStub        ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetAnyClkSrcStub  ( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccGetAnyClkStub     ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_I2c_RccResetHsiStub      ( rcc_PeriphId_t periphId, int callCnt );
static gpio_RequestState_t  Ut_I2c_GpioInitStub         ( gpio_Config_t *gpioConfig, int callCnt );
static dma_RequestState_t   Ut_I2c_DmaInitStub          ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_I2c_DmaGetCountStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt );
static dma_RequestState_t   Ut_I2c_DmaIrqOffStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
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

/** SYSCFG_CFGR1 Fast-mode Plus bit of the test peripheral */
#define UT_I2C_FMP_BIT                      ( SYSCFG_CFGR1_I2C1_FMP )

/** DMA channels of the test data configurations (I2C1 TX DMA1 channel 6, RX DMA1 channel 7 - STM32L4 request mapping) */
#define UT_I2C_DMA_TX                       ( I2C_TX_DMA_I2C1_DMA1_CHANNEL6 )
#define UT_I2C_DMA_RX                       ( I2C_RX_DMA_I2C1_DMA1_CHANNEL7 )

/** Kernel clock (PCLK1) returned by RCC mock [Hz] */
#define UT_I2C_CLK_HZ                       ( 64000000u )

/** HSI16 kernel clock returned by RCC mock [Hz] */
#define UT_I2C_HSI_HZ                       ( 16000000u )

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

/** All I2C interrupt sources used by the module (CR1) */
#define UT_I2C_CR1_IT_ALL                       ( I2C_CR1_TXIE | I2C_CR1_RXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE )

/** I2C interrupt sources of transfer sequencing events (DMA mode, CR1) */
#define UT_I2C_CR1_IT_EVENTS                    ( I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_ERRIE )

/** Encoded DMA channel from the I2C peripheral, DMA peripheral index (0 - DMA1), channel index (0 - channel 1)
 *  and request selection (bit-fields written independently of I2C_DMA_ENCODE) */
#define UT_I2C_DMA_CODE( PERIPH, DMA, CHANNEL, SEL )    ( ( (PERIPH) << 15u ) | ( (DMA) << 10u ) | ( (CHANNEL) << 5u ) | (SEL) )

/** Directions of the rows of the expected DMA channel lists */
#define UT_I2C_DMA_DIR_TX                    ( 0u )
#define UT_I2C_DMA_DIR_RX                    ( 1u )

/** Row of the expected DMA channel lists */
#define UT_I2C_DMA_ROW( ITEM, DIR, PERIPH, DMA, CHANNEL, SEL, REQ )   { (uint32_t)(ITEM), (DIR), (PERIPH), (DMA), (CHANNEL), (SEL), (REQ) }

/** Count of the rows of the expected DMA channel lists */
#define UT_I2C_DMA_ROW_CNT                   ( sizeof( utI2c_DmaRows ) / sizeof( utI2c_DmaRows[ 0u ] ) )

/* ========================== LOCAL VARIABLES =============================== */

/** Expected DMA channel lists (rows follow the lists of the module header; DMA peripheral and channel indexes are 0 based) */
static const utI2c_DmaRow_t utI2c_DmaRows[] =
{
#if !defined(DMAMUX1)
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 5u, 3u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 6u, 5u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 6u, 3u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 5u, 5u, DMA_REQ_I2C1_RX ),
#if defined(I2C2)
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 3u, 3u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 4u, 3u, DMA_REQ_I2C2_RX ),
#endif /* I2C2 */
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 1u, 3u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 2u, 3u, DMA_REQ_I2C3_RX ),
#if defined(I2C4)
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 1u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 0u, 0u, DMA_REQ_I2C4_RX ),
#endif /* I2C4 */
#else
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 0u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 1u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 2u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 3u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 4u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 5u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 0u, 6u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 0u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 1u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 2u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 3u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 4u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 5u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C1_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_1, 1u, 6u, 0u, DMA_REQ_I2C1_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 0u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 1u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 2u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 3u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 4u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 5u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 0u, 6u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 0u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 1u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 2u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 3u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 4u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 5u, 0u, DMA_REQ_I2C1_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C1_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_1, 1u, 6u, 0u, DMA_REQ_I2C1_RX ),
#if defined(I2C2)
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 0u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 1u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 2u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 3u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 4u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 5u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 0u, 6u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 1u, 0u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 1u, 1u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 1u, 2u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 1u, 3u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 1u, 4u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 1u, 5u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C2_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_2, 1u, 6u, 0u, DMA_REQ_I2C2_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 0u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 1u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 2u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 3u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 4u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 5u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 0u, 6u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 1u, 0u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 1u, 1u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 1u, 2u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 1u, 3u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 1u, 4u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 1u, 5u, 0u, DMA_REQ_I2C2_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C2_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_2, 1u, 6u, 0u, DMA_REQ_I2C2_RX ),
#endif /* I2C2 */
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 0u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 1u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 2u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 3u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 4u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 5u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 0u, 6u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 1u, 0u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 1u, 1u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 1u, 2u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 1u, 3u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 1u, 4u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 1u, 5u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C3_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_3, 1u, 6u, 0u, DMA_REQ_I2C3_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 0u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 1u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 2u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 3u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 4u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 5u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 0u, 6u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 1u, 0u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 1u, 1u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 1u, 2u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 1u, 3u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 1u, 4u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 1u, 5u, 0u, DMA_REQ_I2C3_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C3_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_3, 1u, 6u, 0u, DMA_REQ_I2C3_RX ),
#if defined(I2C4)
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 0u, 0u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 0u, 1u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 0u, 2u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 0u, 3u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 0u, 4u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 0u, 5u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 0u, 6u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 0u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 1u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 2u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 3u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 4u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 5u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_TX_DMA_I2C4_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_TX, I2C_PERIPH_4, 1u, 6u, 0u, DMA_REQ_I2C4_TX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA1_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 0u, 0u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA1_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 0u, 1u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA1_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 0u, 2u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA1_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 0u, 3u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA1_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 0u, 4u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA1_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 0u, 5u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA1_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 0u, 6u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL1       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 0u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL2       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 1u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL3       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 2u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL4       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 3u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL5       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 4u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL6       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 5u, 0u, DMA_REQ_I2C4_RX ),
    UT_I2C_DMA_ROW( I2C_RX_DMA_I2C4_DMA2_CHANNEL7       , UT_I2C_DMA_DIR_RX, I2C_PERIPH_4, 1u, 6u, 0u, DMA_REQ_I2C4_RX ),
#endif /* I2C4 */
#endif /* DMAMUX1 */
};

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

/** Kernel clock source returned by Rcc_Get_PeriphClkSrc stub */
static rcc_PeriphId_t           utI2c_ClkSrc;

/** Frequencies returned by Rcc_Get_PeriphClk stub - PCLK1 (APB1) and HSI16 [Hz] */
static rcc_FreqHz_t             utI2c_PclkHz;
static rcc_FreqHz_t             utI2c_HsiHz;

/** DMA configurations of Dma_Init calls (TX, RX) and count of the calls */
static dma_ConfigStruct_t       utI2c_DmaConfig[ 2u ];
static uint32_t                 utI2c_DmaInitCnt;

/** Count of data not moved by DMA returned by Dma_Get_DataCount stub */
static dma_DataCount_t          utI2c_DmaRemaining;

/** Return value of Dma_Init() stub for the RX channel */
static dma_RequestState_t       utI2c_DmaInitRxState;

/** Count of Dma_Set_InterruptInactive() calls and their return value */
static uint32_t                 utI2c_DmaIrqOffCnt;
static dma_RequestState_t       utI2c_DmaIrqOffState;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    utI2c_ClkSrc         = UT_I2C_RCC_PCLK;
    utI2c_PclkHz         = UT_I2C_CLK_HZ;
    utI2c_HsiHz          = UT_I2C_HSI_HZ;
    utI2c_DmaIrqOffState = DMA_REQUEST_OK;
    utI2c_DmaInitRxState = DMA_REQUEST_OK;

    Ut_I2c_Release();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utI2c_Isr          = NULL;
    utI2c_CompleteCnt  = 0u;
    utI2c_ErrorCnt     = 0u;
    utI2c_LastError    = I2C_XFER_ERROR_CNT;
    utI2c_DmaInitCnt   = 0u;
    utI2c_DmaRemaining = 0u;
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
 * \details Initializes I2C1 with SCL pin of I2C3 (PC0, the code is built by the encoding macro -
 *          the item of the pin table does not exist on every device line).
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, no other module is called.
 */
void Ut_I2c_Init_PinOfOtherPeripheral_ReturnsErrorWithoutAccess( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.SclPin = (i2c_SclPin_t)I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3, GPIO_PORT_C, GPIO_PIN_ID_0, GPIO_ALT_FUNC_4 );

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
    TEST_ASSERT_EQUAL_HEX32( 0u, SYSCFG->CFGR1 & UT_I2C_FMP_BIT );
}


/**
 * \brief   I2c_Init() activates the selected HSI kernel clock.
 *
 * \details Initializes I2C1 with HSI clock source (HSI16 16 MHz, PCLK1 64 MHz).
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
 * \details Initializes I2C1 with SCL PB6, SDA PB7 (available on every STM32L4 / L4+) and
 *          pull-up. GPIO initialization is captured by stub.
 *
 * \par Expected results
 * - I2C_REQUEST_OK.
 * - Last configured pin is PB7: alternate mode, open-drain, pull-up, AF4.
 */
void Ut_I2c_Init_Pins_GpioOpenDrainAlternateWithPull( void )
{
    i2c_Config_t config = Ut_I2c_Get_Config();

    config.SclPin  = I2C_SCL_PIN_I2C1_PB6;
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
 * \details Initializes I2C1, sets SYSCFG_CFGR1.I2C1_FMP (1 MHz configuration) and
 *          deinitializes the peripheral. SYSCFG is not reset with the I2C.
 *
 * \par Expected results
 * - SYSCFG clock is activated, then reset pulse and clock deactivation of
 *   RCC_PERIPH_I2C1_PCLK1.
 * - I2C_REQUEST_OK, SYSCFG_CFGR1.I2C1_FMP = 0.
 */
void Ut_I2c_Deinit_FastModePlusActive_FmpDriverDisabled( void )
{
    Ut_I2c_Init( NULL );
    Ut_I2c_Reset_Mocks();
    SYSCFG->CFGR1 |= UT_I2C_FMP_BIT;

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_I2C_RCC_PCLK, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, SYSCFG->CFGR1 & UT_I2C_FMP_BIT );
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
 * \brief   I2c_Set_BusFreq() controls Fast-mode Plus driver (SYSCFG_CFGR1.I2C1_FMP).
 *
 * \details Sets 1 MHz twice, then 400 kHz twice. Rcc_Set_PeriphActive() is a strict
 *          expectation - SYSCFG clock is activated only when the bit has to change.
 *
 * \par Expected results
 * - 1st 1 MHz: SYSCFG clock activated, I2C1_FMP = 1. 2nd 1 MHz: no RCC call.
 * - 1st 400 kHz: SYSCFG clock activated, I2C1_FMP = 0. 2nd 400 kHz: no RCC call.
 * - Other SYSCFG_CFGR1 bits are not changed.
 */
void Ut_I2c_Set_BusFreq_FastModePlus_FmpBitControlled( void )
{
    SYSCFG->CFGR1 = SYSCFG_CFGR1_I2C3_FMP;

    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );
    TEST_ASSERT_EQUAL_HEX32( UT_I2C_FMP_BIT | SYSCFG_CFGR1_I2C3_FMP, SYSCFG->CFGR1 );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );

    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
    TEST_ASSERT_EQUAL_HEX32( SYSCFG_CFGR1_I2C3_FMP, SYSCFG->CFGR1 );
    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Set_BusFreq( UT_I2C_PERIPH, 400000u ) );
}


/**
 * \brief   I2c_Set_BusFreq() reports failed SYSCFG clock activation.
 *
 * \details Sets 1 MHz, Rcc_Set_PeriphActive( RCC_PERIPH_SYSCFG ) returns error.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR, SYSCFG_CFGR1.I2C1_FMP stays 0.
 */
void Ut_I2c_Set_BusFreq_SyscfgClockError_ReturnsError( void )
{
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetClkStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( RCC_PERIPH_SYSCFG, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_BusFreq( UT_I2C_PERIPH, 1000000u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, SYSCFG->CFGR1 & UT_I2C_FMP_BIT );
}


/**
 * \brief   I2c_Get_BusFreq() uses Fast-mode Plus rise / fall times when FMP is set.
 *
 * \details Sets 1 MHz and reads it back; clears SYSCFG_CFGR1.I2C1_FMP and reads the
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

    SYSCFG->CFGR1 &= ~UT_I2C_FMP_BIT;
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
 * \details Kernel clock HSI16 (16 MHz), PCLK1 64 MHz (ratio 4 is allowed):
 *          1 MHz, 400 kHz; kernel clock 3 MHz, PCLK1 64 MHz: 100 kHz.
 *
 * \note    STM32G4 device errata ES0430 2.15.1 / ES0431 2.11.1 / ES0523 2.11.1 ("wrong data
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
 * \details Kernel clock HSI16 (16 MHz), 100 kHz, PCLK1: 24 MHz (ratio 1.5),
 *          24.5 MHz, 32 MHz (ratio 2), 47.5 MHz, 48 MHz (ratio 3), 16 MHz (ratio 1).
 *
 * \note    STM32G4 device errata ES0430 2.15.5 / ES0431 2.11.5 / ES0523 2.11.4 ("I2C
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
 * \details DMA mode with the same RX and TX channel, unused items, item of another I2C
 *          peripheral, DMA peripheral and channel of the item out of range.
 *
 * \par Expected results
 * - I2C_REQUEST_ERROR in all cases, DMA not called.
 */
void Ut_I2c_Set_DataConfig_DmaInvalidChannels_ReturnsErrorWithoutAccess( void )
{
    i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    dataConfig.RxDma = (i2c_RxDma_t)dataConfig.TxDma;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDma = I2C_TX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = I2C_RX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDma = I2C_TX_DMA_I2C3_DMA1_CHANNEL2;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = I2C_RX_DMA_I2C3_DMA1_CHANNEL3;
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.TxDma = (i2c_TxDma_t)I2C_DMA_ENCODE( UT_I2C_PERIPH, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_CNT, 3u );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    dataConfig       = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    dataConfig.RxDma = (i2c_RxDma_t)I2C_DMA_ENCODE( UT_I2C_PERIPH, I2C_DMA_PERIPH_CNT, I2C_DMA_CHANNEL_7, 3u );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );
}


/**
 * \brief   Items of the DMA channel lists carry I2C peripheral, DMA peripheral, channel and request
 *          selection of the channel.
 *
 * \details Expected values are the rows of the table utI2c_DmaRows (STM32CubeMX DMA database -
 *          channels of the DMA_CSELR request mapping, every channel of DMA1 / DMA2 on devices with
 *          DMAMUX1) written as (I2C peripheral, DMA peripheral index, channel index, request
 *          selection), independently of the encoding macro.
 *
 * \par Expected results
 * - Every row of the table: the item carries the expected bit-fields and the decoding macros
 *   return the fields.
 * - Unused items of both lists equal I2C_DMA_CODE_UNUSED.
 */
void Ut_I2c_DmaLists_Items_EncodePeriphDmaChannelAndSelection( void )
{
    TEST_ASSERT_TRUE( 0u < UT_I2C_DMA_ROW_CNT );

    for( uint32_t rowIdx = 0u; UT_I2C_DMA_ROW_CNT > rowIdx; rowIdx++ )
    {
        const utI2c_DmaRow_t * const row = &utI2c_DmaRows[ rowIdx ];

        TEST_ASSERT_EQUAL_HEX32( UT_I2C_DMA_CODE( (uint32_t)row->PeriphId, row->DmaId, row->Channel, row->Selection ), row->Item );
        TEST_ASSERT_EQUAL_UINT32( row->PeriphId,  I2C_DMA_BIT_MASK_DECODE_PERIPH( row->Item ) );
        TEST_ASSERT_EQUAL_UINT32( row->DmaId,     I2C_DMA_BIT_MASK_DECODE_DMA( row->Item ) );
        TEST_ASSERT_EQUAL_UINT32( row->Channel,   I2C_DMA_BIT_MASK_DECODE_CHANNEL( row->Item ) );
        TEST_ASSERT_EQUAL_UINT32( row->Selection, I2C_DMA_BIT_MASK_DECODE_CSELR( row->Item ) );
    }

    /* Unused channel */
    TEST_ASSERT_EQUAL_HEX32( I2C_DMA_CODE_UNUSED, I2C_TX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_HEX32( I2C_DMA_CODE_UNUSED, I2C_RX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_UINT32( I2C_PERIPH_CNT,      I2C_DMA_BIT_MASK_DECODE_PERIPH( I2C_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( I2C_DMA_PERIPH_CNT,  I2C_DMA_BIT_MASK_DECODE_DMA( I2C_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( I2C_DMA_CHANNEL_CNT, I2C_DMA_BIT_MASK_DECODE_CHANNEL( I2C_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( 0u,                  I2C_DMA_BIT_MASK_DECODE_CSELR( I2C_TX_DMA_UNUSED ) );
}


/**
 * \brief   DMA channel and request of every item of the DMA channel lists are passed to the Dma module.
 *
 * \details Every row of the table utI2c_DmaRows is configured as the channel of its direction, the
 *          channel of the opposite direction is the first item of the same I2C peripheral on another
 *          channel. Dma_Init() calls are captured, RCC clock sources of the peripherals are echoed.
 *
 * \par Expected results
 * - I2C_REQUEST_OK, two Dma_Init() calls (transmit, receive) with the DMA peripheral and channel
 *   decoded from the item and the I2C request of the direction.
 * - I2c_Deinit(): I2C_REQUEST_OK.
 */
void Ut_I2c_DmaLists_Items_ChannelAndRequestPassedToDma( void )
{
    Ut_I2c_Ignore_PeriphMocks();
    Rcc_Get_PeriphClkSrc_StubWithCallback( Ut_I2c_RccGetAnyClkSrcStub );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_I2c_RccGetAnyClkStub );
    Nvic_Set_PeriphIrq_Handler_IgnoreAndReturn( NVIC_REQUEST_OK );

    for( uint32_t rowIdx = 0u; UT_I2C_DMA_ROW_CNT > rowIdx; rowIdx++ )
    {
        const utI2c_DmaRow_t * const row     = &utI2c_DmaRows[ rowIdx ];
        const utI2c_DmaRow_t *       partner = NULL;

        for( uint32_t partnerIdx = 0u; UT_I2C_DMA_ROW_CNT > partnerIdx; partnerIdx++ )
        {
            const utI2c_DmaRow_t * const candidate = &utI2c_DmaRows[ partnerIdx ];
            const uint32_t               differs   = ( candidate->DmaId ^ row->DmaId ) | ( candidate->Channel ^ row->Channel );

            if( ( candidate->Direction != row->Direction ) &&
                ( candidate->PeriphId  == row->PeriphId   ) &&
                ( 0u                   != differs         ) &&
                ( NULL                 == partner         )    )
            {
                partner = candidate;
            }
            else
            {
                /* Item of the same direction, of another peripheral or on the same channel */
            }
        }

        TEST_ASSERT_NOT_NULL( partner );

        const utI2c_DmaRow_t * txRow = row;
        const utI2c_DmaRow_t * rxRow = partner;

        if( UT_I2C_DMA_DIR_RX == row->Direction )
        {
            txRow = partner;
            rxRow = row;
        }
        else
        {
            /* Row is the transmit item */
        }

        i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
        i2c_Config_t     config     = Ut_I2c_Get_Config();

        dataConfig.TxDma  = (i2c_TxDma_t)txRow->Item;
        dataConfig.RxDma  = (i2c_RxDma_t)rxRow->Item;
        config.PeriphId   = row->PeriphId;
        config.DataConfig = &dataConfig;

        utI2c_DmaInitCnt = 0u;

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );
        TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );
        TEST_ASSERT_EQUAL_UINT32( txRow->DmaId,   utI2c_DmaConfig[ 0u ].DmaPeriphId );
        TEST_ASSERT_EQUAL_UINT32( txRow->Channel, utI2c_DmaConfig[ 0u ].DmaChannel );
        TEST_ASSERT_EQUAL( txRow->Request,        utI2c_DmaConfig[ 0u ].PeripheralReqId );
        TEST_ASSERT_EQUAL_UINT32( rxRow->DmaId,   utI2c_DmaConfig[ 1u ].DmaPeriphId );
        TEST_ASSERT_EQUAL_UINT32( rxRow->Channel, utI2c_DmaConfig[ 1u ].DmaChannel );
        TEST_ASSERT_EQUAL( rxRow->Request,        utI2c_DmaConfig[ 1u ].PeripheralReqId );

        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( row->PeriphId ) );
    }
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
 * \details Initializes I2C1 in DMA mode (TX DMA1 channel 6 low priority, RX DMA1
 *          channel 7 high priority - STM32L4 request mapping of I2C1).
 *
 * \par Expected results
 * - Dma_Init() twice: TX memory to peripheral with request I2C1_TX and
 *   peripheral address TXDR, RX peripheral to memory with request I2C1_RX and
 *   peripheral address RXDR, configured priorities; 8-bit, normal mode, static
 *   peripheral / incremented memory address; only transfer error callback.
 * - Transfer error interrupt and channel interrupt enabled for TX, then RX channel.
 * - CR1: TXDMAEN, RXDMAEN and I2C interrupt sources disabled.
 */
void Ut_I2c_Set_DataConfig_Dma_ChannelsInitialized( void )
{
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
    Dma_Set_TransferErrorIrqActive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, DMA_REQUEST_OK );
    Dma_Set_InterruptActive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, DMA_REQUEST_OK );
    Dma_Set_TransferErrorIrqActive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_7, DMA_REQUEST_OK );
    Dma_Set_InterruptActive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_7, DMA_REQUEST_OK );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Init( &config ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );

    TEST_ASSERT_EQUAL( DMA_CHANNEL_6,             utI2c_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH,  utI2c_DmaConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW,          utI2c_DmaConfig[ 0u ].Priority );
    TEST_ASSERT_EQUAL( DMA_REQ_I2C1_TX,           utI2c_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL_HEX32( (dma_PeriphAddr_t)(uintptr_t)&UT_I2C_REG->TXDR, utI2c_DmaConfig[ 0u ].PeriphAddress );

    TEST_ASSERT_EQUAL( DMA_CHANNEL_7,             utI2c_DmaConfig[ 1u ].DmaChannel );
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
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    i2c_DataConfig_t       readConfig;

    Ut_I2c_Init( NULL );

    utI2c_DmaInitRxState = DMA_REQUEST_ERROR;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_DataConfig( UT_I2C_PERIPH, &dataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utI2c_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Get_DataConfig( UT_I2C_PERIPH, &readConfig ) );
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
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );

    utI2c_DmaIrqOffState = DMA_REQUEST_ERROR;
    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );

    Ut_I2c_Reset_Mocks();
    utI2c_DmaIrqOffState = DMA_REQUEST_OK;
    Dma_Set_Priority_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, DMA_PRIORITY_LOW, DMA_REQUEST_OK );
    Dma_Set_Priority_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_7, DMA_PRIORITY_HIGH, DMA_REQUEST_OK );

    Ut_I2c_Init( &dataConfig );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaInitCnt );
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
 * \brief   Bus error is ignored, arbitration lost is reported.
 *
 * \details Starts 2 byte write and processes BERR, then TXIS, TXIS and STOPF;
 *          starts it again and processes ARLO.
 *
 * \note    STM32G4 device errata ES0430 2.15.2 / ES0431 2.11.2 / ES0523 2.11.2 ("spurious
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
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = utI2c_RxBuf, .RxSize = 3u };

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();

    Dma_Set_TransferInactive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, (dma_MemoryAddr_t)(uintptr_t)utI2c_TxBuf, DMA_REQUEST_OK );
    Dma_Set_DataCount_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, 2u, DMA_REQUEST_OK );
    Dma_Set_TransferActive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, DMA_REQUEST_OK );
    Dma_Set_TransferInactive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_7, DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_7, (dma_MemoryAddr_t)(uintptr_t)utI2c_RxBuf, DMA_REQUEST_OK );
    Dma_Set_DataCount_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_7, 3u, DMA_REQUEST_OK );
    Dma_Set_TransferActive_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_7, DMA_REQUEST_OK );

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
    const i2c_DataConfig_t  dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );
    const i2c_XferRequest_t request    = { .SlaveAddr = UT_I2C_SLAVE_ADDR, .TxData = utI2c_TxBuf, .TxSize = 2u, .RxData = NULL, .RxSize = 0u };
    i2c_FunctionState_t     xferState  = I2C_FUNCTION_ACTIVE;

    Ut_I2c_Init( &dataConfig );
    Ut_I2c_Reset_Mocks();
    Dma_Set_TransferInactive_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_MemoryAddr_ExpectAndReturn( DMA_PERIPH_1, DMA_CHANNEL_6, (dma_MemoryAddr_t)(uintptr_t)utI2c_TxBuf, DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( I2C_REQUEST_ERROR, I2c_Set_XferStart( UT_I2C_PERIPH, &request ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Get_XferState( UT_I2C_PERIPH, &xferState ) );
    TEST_ASSERT_EQUAL( I2C_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR2 & I2C_CR2_START );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_CompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, utI2c_ErrorCnt );
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
    const i2c_DataConfig_t dataConfig = Ut_I2c_Get_DataConfig( I2C_XFER_MODE_DMA );

    Ut_I2c_Init( &dataConfig );
    UT_I2C_REG->CR1 |= I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL;

    Dma_Set_InterruptInactive_StubWithCallback( Ut_I2c_DmaIrqOffStub );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_I2C_REG->CR1 & ( I2C_CR1_TXDMAEN | I2C_CR1_RXDMAEN | UT_I2C_CR1_IT_ALL ) );

    TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( UT_I2C_PERIPH ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utI2c_DmaIrqOffCnt );
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
 * \brief   Interrupt service routine of every I2C ends with DSB.
 *
 * \details Every I2C of the MCU initialized in ISR mode, the captured interrupt (event and error
 *          line share one handler) is called once without running transfer, the peripheral is
 *          deinitialized.
 *
 * \note    Cortex-M4 r0p1 erratum 838869 "Store immediate overlapping exception return
 *          operation might vector to incorrect interrupt" (STM32G4 Bug AB#1104, STM32L4 errata
 *          sheets not reviewed yet): a buffered store with immediate offset still pending at the
 *          exception return may vector to an incorrect interrupt. Workaround - DSB before the
 *          exception return of every handler.
 *
 * \par Expected results
 * - Every ISR executes exactly one DSB.
 */
void Ut_I2c_Isr_AllPeriphs_EndWithDsb( void )
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

        TEST_ASSERT_EQUAL_UINT32( dsbCnt + 1u, CmsisHost_Get_InstrCnt( CMSISHOST_INSTR_DSB ) );
        TEST_ASSERT_EQUAL( I2C_REQUEST_OK, I2c_Deinit( (i2c_PeriphId_t)periphIdx ) );
    }
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
 *        APB1 clock) and \ref utI2c_HsiHz for HSI16.
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
 * \brief RCC clock source stub of any I2C peripheral - the kernel clock source of the peripheral
 *        is its APB1 clock (the identification asked for is returned).
 */
static rcc_RequestState_t Ut_I2c_RccGetAnyClkSrcStub( rcc_PeriphId_t periphId, rcc_PeriphId_t * const periphClkSrc, int callCnt )
{
    (void)callCnt;

    *periphClkSrc = periphId;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief RCC clock stub of any I2C peripheral - \ref utI2c_PclkHz for every clock.
 */
static rcc_RequestState_t Ut_I2c_RccGetAnyClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)periphId;
    (void)callCnt;

    *periphClk = utI2c_PclkHz;

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
 * \brief Returns data handling configuration with test callbacks.
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
