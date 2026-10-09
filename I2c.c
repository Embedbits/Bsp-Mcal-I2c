/**
 * \author Mr.Nobody
 * \file I2c.c
 * \ingroup I2c
 * \brief Inter-Integrated Circuit (I2C) MCAL module common functionality
 *
 * The module drives I2C peripherals in master mode:
 * - configuration: kernel clock source, SCL frequency (TIMINGR is calculated from the I2C
 *   specification timings), analog / digital noise filter, addressing mode, SCL / SDA pins
 * - transfers: write, read, write followed by read with repeated START and address only
 *   transfer (\ref i2c_XferRequest_t). Transfers longer than 255 bytes use NBYTES reload.
 *
 * Transfer sequencing is common for all data transfer modes (I2c_Set_XferEvents):
 * - TCR (reload):   next chunk of the phase is programmed (max. 255 bytes per chunk)
 * - TC (soft end):  write phase finished, read phase is started with repeated START
 * - NACKF:          error is stored, HW sends STOP condition - transfer ends on STOPF
 * - STOPF:          end of the transfer (success or stored NACK error)
 * - ARLO:           transfer ends immediately with error (bus is released by HW)
 * - BERR:           flag is cleared, the transfer continues (spurious bus error detection in
 *                   master mode, device errata ES0430 2.15.2 / ES0431 2.11.2 / ES0523 2.11.2)
 *
 * Device errata handled by the configuration checks (I2c_Check_KernelClk):
 * - wrong data sampling with tSU;DAT shorter than one I2CCLK period (ES0430 2.15.1 / ES0431
 *   2.11.1 / ES0523 2.11.1): I2CCLK >= 4 / 10 / 20 MHz for Standard / Fast / Fast-mode Plus,
 * - transmission stalled after the first byte (ES0430 2.15.5 / ES0431 2.11.5 / ES0523 2.11.4):
 *   APB1 / I2CCLK clock ratio between 1.5 and 3 is refused.
 *
 */
/* ============================== INCLUDES ================================== */
#include "I2c.h"                            /* Module private interface       */
#include "I2c_Port.h"                       /* Own port file include          */
#include "I2c_Types.h"                      /* Module types definitions       */
#include "I2c_Dma.h"                        /* DMA data transfer handler      */
#include "I2c_Isr.h"                        /* ISR data transfer handler      */
#include "I2c_Poll.h"                       /* Polling data transfer handler  */
#include "Stm32_i2c.h"                      /* I2C RAL functionality          */
#include "Rcc_Port.h"                       /* RCC Mcal layer include         */
#include "Gpio_Port.h"                      /* GPIO Mcal layer include        */
#include "Nvic_Port.h"                      /* NVIC Mcal layer include        */
#include "Dma_Port.h"                       /* DMA Mcal layer include         */
#include "Stm32_system.h"                   /* SYSCFG Fast-mode Plus control  */
/* ============================== TYPEDEFS ================================== */

/** \brief Type representing time in picoseconds (timing calculation) */
typedef uint64_t i2c_TimePs_t;


/** \brief Type representing time in nanoseconds (I2C specification values) */
typedef uint32_t i2c_TimeNs_t;


/** \brief Type representing count of clock periods in TIMINGR fields */
typedef uint32_t i2c_TimingCnt_t;


/** \brief Type representing encoded pin value (\ref i2c_SclPin_t / \ref i2c_SdaPin_t) */
typedef uint32_t i2c_PinCode_t;


/** \brief I2C bus speed modes (I2C-bus specification UM10204) */
typedef enum
{
    I2C_SPEED_MODE_STANDARD = 0u, /**< Standard-mode, up to 100 kHz    */
    I2C_SPEED_MODE_FAST,          /**< Fast-mode, up to 400 kHz        */
    I2C_SPEED_MODE_FAST_PLUS,     /**< Fast-mode Plus, up to 1 MHz     */
    I2C_SPEED_MODE_CNT            /**< Count of speed modes            */
}   i2c_SpeedMode_t;


/** \brief Timing characteristics of one speed mode used for TIMINGR calculation */
typedef struct
{
    i2c_FreqHz_t FreqMax;      /**< Maximum SCL frequency                                   */
    i2c_TimeNs_t SclLowMin;    /**< Minimum SCL low period (tLOW)                           */
    i2c_TimeNs_t SclHighMin;   /**< Minimum SCL high period (tHIGH)                         */
    i2c_TimeNs_t DataSetupMin; /**< Minimum data setup time (tSU;DAT)                       */
    i2c_TimeNs_t DataHoldMin;  /**< Minimum data hold time (tHD;DAT)                        */
    i2c_TimeNs_t DataValidMax; /**< Maximum data valid time (tVD;DAT)                       */
    i2c_FreqHz_t KernelClkMin; /**< Minimum I2CCLK frequency (device errata, tSU;DAT sampling) */
    i2c_TimeNs_t RiseTime;     /**< Assumed SCL / SDA rise time (bus capacitance dependent) */
    i2c_TimeNs_t FallTime;     /**< Assumed SCL / SDA fall time                             */
}   i2c_SpeedCharac_t;


/** \brief Configuration of one I2C peripheral */
typedef struct
{
    I2C_TypeDef          *PeriphReg;                    /**< Peripheral registers                        */
    rcc_PeriphId_t        PeriphRcc[ I2C_CLK_SRC_CNT ]; /**< RCC identification per kernel clock source  */
    nvic_PeriphIrqList_t  PeriphEvNvic;                 /**< Event interrupt NVIC identification         */
    nvic_PeriphIrqList_t  PeriphErNvic;                 /**< Error interrupt NVIC identification         */
    nvic_IsrCallback_t    PeriphIsr;                    /**< Interrupt service routine (event and error) */
    dma_PeriphReqId_t     PeriphDmaTxReq;               /**< DMAMUX request ID for transmission          */
    dma_PeriphReqId_t     PeriphDmaRxReq;               /**< DMAMUX request ID for reception             */
    uint32_t              PeriphFmpMask;                /**< SYSCFG_CFGR1 Fast-mode Plus bit of the I2C  */
}   i2c_PeriphConfigStruct_t;

/* ======================== FORWARD DECLARATIONS ============================ */

#ifdef I2C1
static void I2c_I2c1_IsrHandler( void );
#endif /* I2C1 */
#ifdef I2C2
static void I2c_I2c2_IsrHandler( void );
#endif /* I2C2 */
#ifdef I2C3
static void I2c_I2c3_IsrHandler( void );
#endif /* I2C3 */
#ifdef I2C4
static void I2c_I2c4_IsrHandler( void );
#endif /* I2C4 */

static i2c_RequestState_t  I2c_Check_Config         ( const i2c_Config_t * const i2cConfig );
static i2c_RequestState_t  I2c_Check_Pin            ( i2c_PeriphId_t periphId, i2c_PinCode_t pinCode, i2c_PinCode_t unusedCode );
static i2c_RequestState_t  I2c_Check_PeriphDisabled ( i2c_PeriphId_t periphId );
static i2c_RequestState_t  I2c_Check_XferStopped    ( i2c_PeriphId_t periphId );
static i2c_RequestState_t  I2c_Check_XferRequest    ( i2c_PeriphId_t periphId, const i2c_XferRequest_t * const xferRequest );
static i2c_RequestState_t  I2c_Check_DataConfig     ( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig );

static i2c_RequestState_t  I2c_Set_Enable           ( i2c_PeriphId_t periphId, i2c_FunctionState_t enableState );
static i2c_RequestState_t  I2c_Set_Pin              ( i2c_PeriphId_t periphId, i2c_PinCode_t pinCode, i2c_PinCode_t unusedCode, i2c_PinPull_t pinPull );
static i2c_RequestState_t  I2c_Set_FastModePlus     ( i2c_PeriphId_t periphId, i2c_FunctionState_t fmpState );
static i2c_RequestState_t  I2c_Get_KernelClk        ( i2c_PeriphId_t periphId, i2c_FreqHz_t * const clkFreq );
static i2c_RequestState_t  I2c_Check_KernelClk      ( i2c_PeriphId_t periphId, i2c_FreqHz_t clkFreq, i2c_SpeedMode_t speedMode );

static i2c_RequestState_t  I2c_Get_SpeedMode        ( i2c_FreqHz_t busFreq, i2c_SpeedMode_t * const speedMode );
static i2c_RequestState_t  I2c_Get_TimingReg        ( i2c_FreqHz_t clkFreq,
                                                      i2c_FreqHz_t busFreq,
                                                      i2c_AnalogFilter_t analogFilter,
                                                      i2c_DigitalFilter_t digitalFilter,
                                                      uint32_t * const timingReg );
static i2c_TimePs_t        I2c_Get_SyncDelay        ( i2c_TimePs_t clkPeriod, i2c_AnalogFilter_t analogFilter, i2c_DigitalFilter_t digitalFilter );
static i2c_TimePs_t        I2c_Get_SclPeriod        ( i2c_TimePs_t syncDelay, i2c_TimePs_t prescPeriod, i2c_TimingCnt_t sclCnt, i2c_SpeedMode_t speedMode );

static i2c_RequestState_t  I2c_Set_XferInit         ( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig );
static i2c_RequestState_t  I2c_Set_XferDeinit       ( i2c_PeriphId_t periphId );
static i2c_FunctionState_t I2c_Get_IrqUsed          ( const i2c_DataConfig_t * const dataConfig );
static i2c_RequestState_t  I2c_Set_IrqInit          ( i2c_PeriphId_t periphId, i2c_IrqPrio_t irqPrio );
static i2c_RequestState_t  I2c_Set_IrqDeinit        ( i2c_PeriphId_t periphId );

static i2c_RequestState_t  I2c_Set_XferChunk        ( i2c_PeriphId_t periphId, i2c_FunctionState_t startCond );
static i2c_RequestState_t  I2c_Set_XferEnd          ( i2c_PeriphId_t periphId, i2c_XferErrorId_t errorId );
static i2c_RequestState_t  I2c_Set_XferAbort        ( i2c_PeriphId_t periphId );
static i2c_RequestState_t  I2c_Set_Cr2Reset         ( i2c_PeriphId_t periphId );
static i2c_RequestState_t  I2c_Set_TxFlush          ( i2c_PeriphId_t periphId );

static i2c_RequestState_t  I2c_None_Check_Config    ( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig );
static i2c_RequestState_t  I2c_None_Xfer            ( i2c_PeriphId_t periphId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define I2C_MAJOR_VERSION               ( 1u )

/** Value of minor version of SW module */
#define I2C_MINOR_VERSION               ( 0u )

/** Value of patch version of SW module */
#define I2C_PATCH_VERSION               ( 0u )


/** Default SCL frequency used by \ref I2c_Get_DefaultConfig (Standard-mode) */
#define I2C_DEFAULT_BUS_FREQ_HZ         ( 100000u )

/** Maximum count of bytes of one NBYTES chunk (larger phases use reload) */
#define I2C_NBYTES_MAX                  ( 255u )

/** 7-bit slave address is placed in SADD[7:1] */
#define I2C_SLAVE_ADDR_7BIT_SHIFT       ( 1u )


/** Count of picoseconds in one second */
#define I2C_PS_PER_S                    ( (i2c_TimePs_t)1000000000000u )

/** Count of picoseconds in one nanosecond */
#define I2C_PS_PER_NS                   ( (i2c_TimePs_t)1000u )

/** Minimum input delay of the analog filter (tAF min, refer to device datasheet) */
#define I2C_AF_DELAY_MIN_NS             ( 50u )

/** Maximum input delay of the analog filter (tAF max, refer to device datasheet) */
#define I2C_AF_DELAY_MAX_NS             ( 260u )

/** Delay of SCL synchronization to I2CCLK in I2CCLK periods (per SCL edge, RM: 2 - 3) */
#define I2C_SYNC_CLK_CYCLES             ( 2u )

/** SDADEL minimum constraint: tSDADEL >= tf + tHD;DAT(min) - tAF(min) - (DNF + 3) x tI2CCLK */
#define I2C_SDADEL_MIN_CLK_CYCLES       ( 3u )

/** SDADEL maximum constraint: tSDADEL <= tVD;DAT(max) - tr - tAF(max) - (DNF + 4) x tI2CCLK */
#define I2C_SDADEL_MAX_CLK_CYCLES       ( 4u )

/** SCL low period constraint: tI2CCLK < ( tSCLL - tAF - tDNF ) / 4 */
#define I2C_SCLL_MIN_CLK_CYCLES         ( 4u )

/** Maximum value of TIMINGR PRESC field */
#define I2C_TIMING_PRESC_MAX            ( 15u )

/** Maximum value of TIMINGR SCLDEL / SDADEL fields */
#define I2C_TIMING_DEL_MAX              ( 15u )

/** Maximum count of prescaled periods of SCL low / high phase (SCLL / SCLH = 255) */
#define I2C_TIMING_SCL_CNT_MAX          ( 256u )

/** Offset between count of periods and TIMINGR field value (tSCLL = ( SCLL + 1 ) x tPRESC) */
#define I2C_TIMING_REG_OFFSET           ( 1u )

/** Accepted deviation of the resulting SCL frequency from the required one in percent */
#define I2C_BUS_FREQ_TOLERANCE_PCT      ( 10u )

/** Count of percent in one unit */
#define I2C_PERCENT                     ( 100u )

/** APB1 / I2CCLK ratio window (in halves) causing stalled transmission (device errata ES0430 2.15.5):
 *  1.5 < APB1 / I2CCLK < 3 */
#define I2C_ERRATA_RATIO_MIN_HALVES     ( 3u )
#define I2C_ERRATA_RATIO_MAX_HALVES     ( 6u )
#define I2C_ERRATA_RATIO_HALF           ( 2u )

/** Count of SCL edges (low and high phase) - synchronization delay is applied per edge */
#define I2C_SCL_EDGES_CNT               ( 2u )

/* =============================== MACROS =================================== */

/** Integer division rounded up */
#define I2C_DIV_ROUND_UP( dividend, divisor )       ( ( (dividend) + (divisor) - 1u ) / (divisor) )

/** Integer division rounded to the nearest integer */
#define I2C_DIV_ROUND_NEAREST( dividend, divisor )  ( ( (dividend) + ( (divisor) / 2u ) ) / (divisor) )

/** CR2 fields programmed for every transfer chunk (verified by read-back, START is cleared by HW) */
#define I2C_CR2_XFER_MASK               ( I2C_CR2_SADD | I2C_CR2_ADD10 | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN )

/** CR2 fields cleared at the end of a transfer (ADD10 keeps the addressing mode) */
#define I2C_CR2_RESET_MASK              ( I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN )

/** Flags of a previous transfer cleared before the transfer start */
#define I2C_ICR_XFER_FLAGS              ( LL_I2C_ICR_NACKCF | LL_I2C_ICR_STOPCF | LL_I2C_ICR_BERRCF | LL_I2C_ICR_ARLOCF | LL_I2C_ICR_OVRCF )

/** Flags terminating the transfer immediately */


/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief I2C peripherals configuration array */
static const i2c_PeriphConfigStruct_t i2c_PeriphConf[ ] =
{
#ifdef I2C1
    { .PeriphReg      = I2C1,
      .PeriphRcc      = { [I2C_CLK_SRC_PCLK] = RCC_PERIPH_I2C1_PCLK1, [I2C_CLK_SRC_SYSCLK] = RCC_PERIPH_I2C1_SYSCLK, [I2C_CLK_SRC_HSI] = RCC_PERIPH_I2C1_HSI },
      .PeriphEvNvic   = NVIC_PERIPH_IRQ_I2C1_EV, .PeriphErNvic   = NVIC_PERIPH_IRQ_I2C1_ER, .PeriphIsr = I2c_I2c1_IsrHandler,
      .PeriphDmaTxReq = DMA_REQ_I2C1_TX,         .PeriphDmaRxReq = DMA_REQ_I2C1_RX,         .PeriphFmpMask = LL_SYSCFG_I2C_FASTMODEPLUS_I2C1 },
#endif
#ifdef I2C2
    { .PeriphReg      = I2C2,
      .PeriphRcc      = { [I2C_CLK_SRC_PCLK] = RCC_PERIPH_I2C2_PCLK1, [I2C_CLK_SRC_SYSCLK] = RCC_PERIPH_I2C2_SYSCLK, [I2C_CLK_SRC_HSI] = RCC_PERIPH_I2C2_HSI },
      .PeriphEvNvic   = NVIC_PERIPH_IRQ_I2C2_EV, .PeriphErNvic   = NVIC_PERIPH_IRQ_I2C2_ER, .PeriphIsr = I2c_I2c2_IsrHandler,
      .PeriphDmaTxReq = DMA_REQ_I2C2_TX,         .PeriphDmaRxReq = DMA_REQ_I2C2_RX,         .PeriphFmpMask = LL_SYSCFG_I2C_FASTMODEPLUS_I2C2 },
#endif
#ifdef I2C3
    { .PeriphReg      = I2C3,
      .PeriphRcc      = { [I2C_CLK_SRC_PCLK] = RCC_PERIPH_I2C3_PCLK1, [I2C_CLK_SRC_SYSCLK] = RCC_PERIPH_I2C3_SYSCLK, [I2C_CLK_SRC_HSI] = RCC_PERIPH_I2C3_HSI },
      .PeriphEvNvic   = NVIC_PERIPH_IRQ_I2C3_EV, .PeriphErNvic   = NVIC_PERIPH_IRQ_I2C3_ER, .PeriphIsr = I2c_I2c3_IsrHandler,
      .PeriphDmaTxReq = DMA_REQ_I2C3_TX,         .PeriphDmaRxReq = DMA_REQ_I2C3_RX,         .PeriphFmpMask = LL_SYSCFG_I2C_FASTMODEPLUS_I2C3 },
#endif
#ifdef I2C4
    { .PeriphReg      = I2C4,
      .PeriphRcc      = { [I2C_CLK_SRC_PCLK] = RCC_PERIPH_I2C4_PCLK1, [I2C_CLK_SRC_SYSCLK] = RCC_PERIPH_I2C4_SYSCLK, [I2C_CLK_SRC_HSI] = RCC_PERIPH_I2C4_HSI },
      .PeriphEvNvic   = NVIC_PERIPH_IRQ_I2C4_EV, .PeriphErNvic   = NVIC_PERIPH_IRQ_I2C4_ER, .PeriphIsr = I2c_I2c4_IsrHandler,
      .PeriphDmaTxReq = DMA_REQ_I2C4_TX,         .PeriphDmaRxReq = DMA_REQ_I2C4_RX,         .PeriphFmpMask = LL_SYSCFG_I2C_FASTMODEPLUS_I2C4 },
#endif
};

_Static_assert( I2C_PERIPH_CNT == ( sizeof(i2c_PeriphConf) / sizeof(i2c_PeriphConfigStruct_t) ), "I2c: size of i2c_PeriphConf is incorrect." );


/**
 * \brief i2c_SpeedMode_t -> timing characteristics
 *
 * tLOW / tHIGH / tSU;DAT / tHD;DAT / tVD;DAT from I2C-bus specification, rise and fall times
 * are typical values used by ST I2C timing configuration tool (bus with ~ 100 pF and
 * appropriate pull-up resistors). Minimum I2CCLK frequency from device errata ES0430 2.15.1 /
 * ES0431 2.11.1 / ES0523 2.11.1 (I2CCLK period within the minimum data setup time).
 */
static const i2c_SpeedCharac_t i2c_SpeedCharacLut[ I2C_SPEED_MODE_CNT ] =
{
    [I2C_SPEED_MODE_STANDARD]  = { .FreqMax = 100000u,  .SclLowMin = 4700u, .SclHighMin = 4000u, .DataSetupMin = 250u, .DataHoldMin = 0u, .DataValidMax = 3450u, .RiseTime = 640u, .FallTime = 20u,  .KernelClkMin = 4000000u  },
    [I2C_SPEED_MODE_FAST]      = { .FreqMax = 400000u,  .SclLowMin = 1300u, .SclHighMin = 600u,  .DataSetupMin = 100u, .DataHoldMin = 0u, .DataValidMax = 900u,  .RiseTime = 250u, .FallTime = 100u, .KernelClkMin = 10000000u },
    [I2C_SPEED_MODE_FAST_PLUS] = { .FreqMax = 1000000u, .SclLowMin = 500u,  .SclHighMin = 260u,  .DataSetupMin = 50u,  .DataHoldMin = 0u, .DataValidMax = 450u,  .RiseTime = 60u,  .FallTime = 100u, .KernelClkMin = 20000000u },
};


/** \brief i2c_AddrMode_t -> LL master addressing mode */
static const uint32_t i2c_AddrModeLut[ I2C_ADDR_MODE_CNT ] =
{
    [I2C_ADDR_MODE_7BIT]  = LL_I2C_ADDRESSING_MODE_7BIT,
    [I2C_ADDR_MODE_10BIT] = LL_I2C_ADDRESSING_MODE_10BIT,
};


/** \brief i2c_AddrMode_t -> maximum slave address */
static const i2c_SlaveAddr_t i2c_SlaveAddrMaxLut[ I2C_ADDR_MODE_CNT ] =
{
    [I2C_ADDR_MODE_7BIT]  = I2C_SLAVE_ADDR_7BIT_MAX,
    [I2C_ADDR_MODE_10BIT] = I2C_SLAVE_ADDR_10BIT_MAX,
};


/** \brief i2c_PinPull_t -> GPIO pull configuration */
static const gpio_PinPullCfg_t i2c_PinPullLut[ I2C_PIN_PULL_CNT ] =
{
    [I2C_PIN_PULL_NONE] = GPIO_PIN_PULL_NONE,
    [I2C_PIN_PULL_UP]   = GPIO_PIN_PULL_UP,
};


/** \brief i2c_XferMode_t -> data transfer mode handler */
static const i2c_XferModeIf_t i2c_XferModeLut[ I2C_XFER_MODE_CNT ] =
{
    [I2C_XFER_MODE_NONE] = { .CheckConfig = I2c_None_Check_Config, .Init = I2c_None_Xfer,     .Deinit = I2c_None_Xfer,       .Start = I2c_None_Xfer,      .Stop = I2c_None_Xfer,      .CheckDone = I2c_None_Xfer      },
    [I2C_XFER_MODE_DMA]  = { .CheckConfig = I2c_Dma_Check_Config,  .Init = I2c_Dma_XferInit,  .Deinit = I2c_Dma_XferDeinit,  .Start = I2c_Dma_XferStart,  .Stop = I2c_Dma_XferStop,   .CheckDone = I2c_Dma_Check_Done },
    [I2C_XFER_MODE_ISR]  = { .CheckConfig = I2c_Isr_Check_Config,  .Init = I2c_Isr_XferInit,  .Deinit = I2c_Isr_XferDeinit,  .Start = I2c_Isr_XferStart,  .Stop = I2c_Isr_XferStop,   .CheckDone = I2c_None_Xfer      },
    [I2C_XFER_MODE_POLL] = { .CheckConfig = I2c_Poll_Check_Config, .Init = I2c_Poll_XferInit, .Deinit = I2c_Poll_XferDeinit, .Start = I2c_Poll_XferStart, .Stop = I2c_Poll_XferStop,  .CheckDone = I2c_None_Xfer      },
};


/** \brief Data handling runtime context per peripheral */
static i2c_XferContext_t i2c_XferContext[ I2C_PERIPH_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
i2c_ModuleVersion_t I2c_Get_ModuleVersion( void )
{
    i2c_ModuleVersion_t retVersion;

    retVersion.Major = I2C_MAJOR_VERSION;
    retVersion.Minor = I2C_MINOR_VERSION;
    retVersion.Patch = I2C_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief I2C peripheral initialization through configuration structure
 *
 * Data handling of a previous initialization is released, the kernel clock source is selected
 * and the clock enabled, the peripheral is reset, SCL / SDA pins are configured, filters,
 * timing and addressing mode are configured, the peripheral is enabled and the data handling is
 * initialized (if DataConfig is set).
 *
 * \note  Pins are configured before the peripheral is enabled - external pull-up resistors keep
 *        the bus idle in the meantime.
 *
 * \param i2cConfig [in]: Pointer to configuration structure. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Init( const i2c_Config_t * const i2cConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Check_Config( i2cConfig );

    if( I2C_REQUEST_OK == retState )
    {
        const i2c_PeriphId_t periphId = i2cConfig->PeriphId;
        const rcc_PeriphId_t rccId    = i2c_PeriphConf[ periphId ].PeriphRcc[ i2cConfig->ClkSrc ];
        rcc_RequestState_t   rccState = RCC_REQUEST_ERROR;

        /*------------- Data handling of previous initialization -------------*/
        retState = I2c_Set_XferDeinit( periphId );

        /*------- Kernel clock source selection and clock activation --------*/
        if( I2C_REQUEST_OK == retState )
        {
            /* Active kernel clock is released first - RCC changes the multiplexer of a released
             * clock only (re-initialization with another source, STM32H5 module bug AB#1098) */
            rccState = Rcc_Set_PeriphInactive( i2c_PeriphConf[ periphId ].PeriphRcc[ I2C_CLK_SRC_PCLK ] );

            if( RCC_REQUEST_OK == rccState )
            {
                rccState = Rcc_Set_PeriphActive( rccId );
            }
            else
            {
                /* Active kernel clock could not be released */
            }

            if( RCC_REQUEST_OK == rccState )
            {
                rccState = Rcc_Set_ResetActive( rccId );
            }
            else
            {
                /* Peripheral clock activation failed */
            }

            if( RCC_REQUEST_OK == rccState )
            {
                rccState = Rcc_Set_ResetInactive( rccId );
            }
            else
            {
                /* Peripheral reset failed */
            }

            if( RCC_REQUEST_OK == rccState )
            {
                retState = I2C_REQUEST_OK;
            }
            else
            {
                retState = I2C_REQUEST_ERROR;
            }
        }
        else
        {
            /* Data handling of previous initialization could not be released */
        }

        /*------------------------ GPIO pins ---------------------------------*/
        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_Pin( periphId, (i2c_PinCode_t)i2cConfig->SclPin, (i2c_PinCode_t)I2C_SCL_PIN_UNUSED, i2cConfig->PinPull );
        }
        else
        {
            /* Error during initialization process */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_Pin( periphId, (i2c_PinCode_t)i2cConfig->SdaPin, (i2c_PinCode_t)I2C_SDA_PIN_UNUSED, i2cConfig->PinPull );
        }
        else
        {
            /* Error during initialization process */
        }

        /*-------------- Peripheral configuration (PE = 0 after reset) -------*/
        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_AnalogFilter( periphId, i2cConfig->AnalogFilter );
        }
        else
        {
            /* Error during initialization process */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_DigitalFilter( periphId, i2cConfig->DigitalFilter );
        }
        else
        {
            /* Error during initialization process */
        }

        /* Timing is calculated with the configured filters - filters have to be configured first */
        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_BusFreq( periphId, i2cConfig->BusFreq );
        }
        else
        {
            /* Error during initialization process */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_AddrMode( periphId, i2cConfig->AddrMode );
        }
        else
        {
            /* Error during initialization process */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_PeriphActive( periphId );
        }
        else
        {
            /* Error during initialization process */
        }

        /*------------------ Data handling initialization --------------------*/
        if( ( I2C_REQUEST_OK == retState              ) &&
            ( I2C_NULL_PTR   != i2cConfig->DataConfig )    )
        {
            retState = I2c_Set_DataConfig( periphId, i2cConfig->DataConfig );
        }
        else
        {
            /* Error during initialization process or data handling is not used */
        }
    }
    else
    {
        /* Configuration is invalid, nothing is modified */
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief I2C peripheral de-initialization
 *
 * Running transfer is aborted, data handling resources (DMA channels, I2C interrupts) are
 * released, the peripheral is disabled, Fast-mode Plus driver of its pins is disabled (SYSCFG
 * is not reset with the peripheral), the peripheral is reset and its clock is disabled. All
 * steps are executed, any failure is reported. GPIO pins are not changed.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Deinit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        const rcc_PeriphId_t rccId = i2c_PeriphConf[ periphId ].PeriphRcc[ I2C_CLK_SRC_PCLK ];

        /* -1- Data handling (transfer aborted, DMA channels and I2C interrupts released) */
        const i2c_RequestState_t xferState = I2c_Set_XferDeinit( periphId );

        /* -2- Peripheral and Fast-mode Plus driver of its pins */
        const i2c_RequestState_t periphState = I2c_Set_Enable( periphId, I2C_FUNCTION_INACTIVE );
        const i2c_RequestState_t fmpState    = I2c_Set_FastModePlus( periphId, I2C_FUNCTION_INACTIVE );

        /* -3- Peripheral reset and clock */
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( rccId );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( rccId );
        const rcc_RequestState_t clkState      = Rcc_Set_PeriphInactive( rccId );

        if( ( I2C_REQUEST_OK == xferState     ) &&
            ( I2C_REQUEST_OK == periphState   ) &&
            ( I2C_REQUEST_OK == fmpState      ) &&
            ( RCC_REQUEST_OK == rstActState   ) &&
            ( RCC_REQUEST_OK == rstInactState ) &&
            ( RCC_REQUEST_OK == clkState      )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Main task of module I2c
 *
 * This function shall be called in the main loop of the application or the task scheduler.
 * It moves data of running transfers in I2C_XFER_MODE_POLL.
 *
 * \note  Master clock stretching holds the bus while the task is not called - no data is lost,
 *        the transfer only takes longer.
 */
void I2c_Task( void )
{
    for( i2c_PeriphId_t periphId = (i2c_PeriphId_t)0u; I2C_PERIPH_CNT > periphId; periphId ++ )
    {
        const i2c_XferContext_t * const xferCtx = &i2c_XferContext[ periphId ];

        if( ( I2C_FUNCTION_ACTIVE == xferCtx->InitState       ) &&
            ( I2C_XFER_MODE_POLL  == xferCtx->Config.XferMode ) &&
            ( I2C_FUNCTION_ACTIVE == xferCtx->XferState       )    )
        {
            (void)I2c_Poll_Task( periphId );
        }
        else
        {
            /* No polling transfer is running on the peripheral */
        }
    }
}


/**
 * \brief Initialization of configuration structure to default values
 *
 * I2C_PERIPH_1, PCLK kernel clock, 100 kHz, analog filter enabled, digital filter off, 7-bit
 * addressing, no data handling, pins not configured, no internal pull-up.
 *
 * \param i2cConfig [out]: Pointer to configuration structure. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_DefaultConfig( i2c_Config_t * const i2cConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_NULL_PTR != i2cConfig )
    {
        i2cConfig->PeriphId      = (i2c_PeriphId_t)0u;
        i2cConfig->ClkSrc        = I2C_CLK_SRC_PCLK;
        i2cConfig->BusFreq       = I2C_DEFAULT_BUS_FREQ_HZ;
        i2cConfig->AnalogFilter  = I2C_ANALOG_FILTER_ENABLED;
        i2cConfig->DigitalFilter = I2C_DIGITAL_FILTER_OFF;
        i2cConfig->AddrMode      = I2C_ADDR_MODE_7BIT;
        i2cConfig->DataConfig    = I2C_NULL_PTR;
        i2cConfig->SclPin        = I2C_SCL_PIN_UNUSED;
        i2cConfig->SdaPin        = I2C_SDA_PIN_UNUSED;
        i2cConfig->PinPull       = I2C_PIN_PULL_NONE;

        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------ Peripheral configuration ------------------------ */
/* -------------------------------------------------------------------------- */

/**
 * \brief Enables I2C peripheral (PE = 1)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_PeriphActive( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Set_Enable( periphId, I2C_FUNCTION_ACTIVE );

    return ( retState );
}


/**
 * \brief Disables I2C peripheral (PE = 0) - communication is released, I2C state machines and
 *        status flags are reset, configuration registers are kept
 *
 * \pre   No transfer may be running (use I2c_Set_XferStop() to abort it). Otherwise
 *        \ref I2C_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_PeriphInactive( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Check_XferStopped( periphId );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Set_Enable( periphId, I2C_FUNCTION_INACTIVE );
    }
    else
    {
        /* Transfer is running, peripheral can not be disabled */
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads I2C peripheral activation state (PE)
 *
 * \param periphId     [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param periphState [out]: Pointer to store the activation state (\ref i2c_FlagState_t).
 *                           Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_PeriphState( i2c_PeriphId_t periphId, i2c_FlagState_t * const periphState )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId    ) &&
        ( I2C_NULL_PTR   != periphState )    )
    {
        const uint32_t regValue = LL_I2C_IsEnabled( i2c_PeriphConf[ periphId ].PeriphReg );

        if( 0u != regValue )
        {
            *periphState = I2C_FLAG_ACTIVE;
        }
        else
        {
            *periphState = I2C_FLAG_INACTIVE;
        }

        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures SCL frequency - TIMINGR is calculated from the kernel clock frequency, the
 *        configured noise filters and the I2C specification timings of the speed mode
 *
 * \note  Speed mode is derived from the frequency: up to 100 kHz Standard-mode, up to 400 kHz
 *        Fast-mode, up to 1 MHz Fast-mode Plus (Fast-mode Plus 20 mA drive (FMP) is enabled).
 *        The resulting frequency never exceeds the speed mode maximum and differs from the
 *        required one by I2C_BUS_FREQ_TOLERANCE_PCT at most.
 *
 * \note  Noise filters are part of the calculation - after their change the frequency has to be
 *        configured again.
 *
 * \pre   Peripheral must be disabled (PE = 0). Otherwise \ref I2C_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param busFreq  [in]: Required SCL frequency in Hz (1 - \ref I2C_BUS_FREQ_MAX_HZ)
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise (also if the frequency can not be reached
 *         with the kernel clock) returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_BusFreq( i2c_PeriphId_t periphId, i2c_FreqHz_t busFreq )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Check_PeriphDisabled( periphId );

    if( I2C_REQUEST_OK == retState )
    {
        I2C_TypeDef * const periphReg     = i2c_PeriphConf[ periphId ].PeriphReg;
        i2c_FreqHz_t        clkFreq       = 0u;
        i2c_SpeedMode_t     speedMode     = I2C_SPEED_MODE_STANDARD;
        i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_ENABLED;
        i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_OFF;
        uint32_t            timingReg     = 0u;

        retState = I2c_Get_SpeedMode( busFreq, &speedMode );

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Get_KernelClk( periphId, &clkFreq );
        }
        else
        {
            /* Frequency is out of range */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Check_KernelClk( periphId, clkFreq, speedMode );
        }
        else
        {
            /* Kernel clock frequency is not available */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Get_AnalogFilter( periphId, &analogFilter );
        }
        else
        {
            /* Kernel clock is not usable for the speed mode (device errata) */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Get_DigitalFilter( periphId, &digitalFilter );
        }
        else
        {
            /* Analog filter state is not available */
        }

        if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Get_TimingReg( clkFreq, busFreq, analogFilter, digitalFilter, &timingReg );
        }
        else
        {
            /* Digital filter state is not available */
        }

        if( I2C_REQUEST_OK == retState )
        {
            LL_I2C_SetTiming( periphReg, timingReg );

            for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                const uint32_t regValue = LL_I2C_ReadReg( periphReg, TIMINGR );

                if( timingReg == regValue )
                {
                    retState = I2C_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Timing has not yet been applied, keep return state as error */
                    retState = I2C_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Frequency can not be reached, timing is not modified */
        }

        if( ( I2C_REQUEST_OK           == retState  ) &&
            ( I2C_SPEED_MODE_FAST_PLUS == speedMode )    )
        {
            retState = I2c_Set_FastModePlus( periphId, I2C_FUNCTION_ACTIVE );
        }
        else if( I2C_REQUEST_OK == retState )
        {
            retState = I2c_Set_FastModePlus( periphId, I2C_FUNCTION_INACTIVE );
        }
        else
        {
            /* Timing configuration failed */
        }
    }
    else
    {
        /* Peripheral is enabled (or invalid ID), timing change is not allowed */
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns SCL frequency calculated from TIMINGR, noise filters and kernel clock
 *
 * \note  The value is an estimate - real SCL frequency depends on rise / fall times of the bus
 *        (typical values of the speed mode are used) and on clock stretching of the slaves.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param busFreq [out]: Pointer to store the SCL frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_BusFreq( i2c_PeriphId_t periphId, i2c_FreqHz_t * const busFreq )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId ) &&
        ( I2C_NULL_PTR   != busFreq  )    )
    {
        I2C_TypeDef * const periphReg     = i2c_PeriphConf[ periphId ].PeriphReg;
        i2c_FreqHz_t        clkFreq       = 0u;
        i2c_AnalogFilter_t  analogFilter  = I2C_ANALOG_FILTER_ENABLED;
        i2c_DigitalFilter_t digitalFilter = I2C_DIGITAL_FILTER_OFF;

        const i2c_RequestState_t clkState = I2c_Get_KernelClk( periphId, &clkFreq );
        const i2c_RequestState_t afState  = I2c_Get_AnalogFilter( periphId, &analogFilter );
        const i2c_RequestState_t dnfState = I2c_Get_DigitalFilter( periphId, &digitalFilter );

        if( ( I2C_REQUEST_OK == clkState ) &&
            ( I2C_REQUEST_OK == afState  ) &&
            ( I2C_REQUEST_OK == dnfState )    )
        {
            const i2c_TimingCnt_t presc     = LL_I2C_GetTimingPrescaler( periphReg );
            const i2c_TimingCnt_t sclLow    = LL_I2C_GetClockLowPeriod( periphReg );
            const i2c_TimingCnt_t sclHigh   = LL_I2C_GetClockHighPeriod( periphReg );
            const uint32_t        fmpState  = READ_BIT( SYSCFG->CFGR1, i2c_PeriphConf[ periphId ].PeriphFmpMask );
            const i2c_TimePs_t    clkPeriod = I2C_PS_PER_S / clkFreq;
            const i2c_TimePs_t    tPresc    = ( presc + I2C_TIMING_REG_OFFSET ) * clkPeriod;
            const i2c_TimePs_t    syncDelay = I2c_Get_SyncDelay( clkPeriod, analogFilter, digitalFilter );
            const i2c_TimingCnt_t sclCnt    = sclLow + sclHigh + ( I2C_SCL_EDGES_CNT * I2C_TIMING_REG_OFFSET );
            i2c_TimePs_t          sclPeriod = 0u;

            if( 0u != fmpState )
            {
                sclPeriod = I2c_Get_SclPeriod( syncDelay, tPresc, sclCnt, I2C_SPEED_MODE_FAST_PLUS );
            }
            else
            {
                /* Standard-mode rise / fall times are used first, Fast-mode above its maximum */
                sclPeriod = I2c_Get_SclPeriod( syncDelay, tPresc, sclCnt, I2C_SPEED_MODE_STANDARD );

                const i2c_FreqHz_t stdFreq = (i2c_FreqHz_t)( I2C_PS_PER_S / sclPeriod );

                if( i2c_SpeedCharacLut[ I2C_SPEED_MODE_STANDARD ].FreqMax < stdFreq )
                {
                    sclPeriod = I2c_Get_SclPeriod( syncDelay, tPresc, sclCnt, I2C_SPEED_MODE_FAST );
                }
                else
                {
                    /* Standard-mode frequency */
                }
            }

            *busFreq = (i2c_FreqHz_t)I2C_DIV_ROUND_NEAREST( I2C_PS_PER_S, sclPeriod );
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables / disables analog noise filter
 *
 * \pre   Peripheral must be disabled (PE = 0). Otherwise \ref I2C_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId     [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param analogFilter [in]: Required filter state, value from \ref i2c_AnalogFilter_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_AnalogFilter( i2c_PeriphId_t periphId, i2c_AnalogFilter_t analogFilter )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_ANALOG_FILTER_CNT > analogFilter )
    {
        retState = I2c_Check_PeriphDisabled( periphId );
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    if( I2C_REQUEST_OK == retState )
    {
        I2C_TypeDef * const periphReg = i2c_PeriphConf[ periphId ].PeriphReg;
        uint32_t            expected  = 0u;

        if( I2C_ANALOG_FILTER_ENABLED == analogFilter )
        {
            LL_I2C_EnableAnalogFilter( periphReg );
            expected = 1u;
        }
        else
        {
            LL_I2C_DisableAnalogFilter( periphReg );
            expected = 0u;
        }

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_I2C_IsEnabledAnalogFilter( periphReg );

            if( expected == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Filter state has not yet been applied, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Invalid parameter or peripheral is enabled, configuration change is not allowed */
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads analog noise filter state
 *
 * \param periphId      [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param analogFilter [out]: Pointer to store the filter state (\ref i2c_AnalogFilter_t).
 *                            Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_AnalogFilter( i2c_PeriphId_t periphId, i2c_AnalogFilter_t * const analogFilter )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId     ) &&
        ( I2C_NULL_PTR   != analogFilter )    )
    {
        const uint32_t regValue = LL_I2C_IsEnabledAnalogFilter( i2c_PeriphConf[ periphId ].PeriphReg );

        if( 0u != regValue )
        {
            *analogFilter = I2C_ANALOG_FILTER_ENABLED;
        }
        else
        {
            *analogFilter = I2C_ANALOG_FILTER_DISABLED;
        }

        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures digital noise filter length (DNF)
 *
 * \pre   Peripheral must be disabled (PE = 0). Otherwise \ref I2C_REQUEST_ERROR is returned
 *        and no register is modified.
 *
 * \param periphId      [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param digitalFilter [in]: Required filter length, value from \ref i2c_DigitalFilter_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_DigitalFilter( i2c_PeriphId_t periphId, i2c_DigitalFilter_t digitalFilter )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_DIGITAL_FILTER_CNT > digitalFilter )
    {
        retState = I2c_Check_PeriphDisabled( periphId );
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    if( I2C_REQUEST_OK == retState )
    {
        I2C_TypeDef * const periphReg = i2c_PeriphConf[ periphId ].PeriphReg;

        /* Enumeration value equals the DNF field value */
        LL_I2C_SetDigitalFilter( periphReg, (uint32_t)digitalFilter );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_I2C_GetDigitalFilter( periphReg );

            if( (uint32_t)digitalFilter == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Filter length has not yet been applied, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Invalid parameter or peripheral is enabled, configuration change is not allowed */
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads digital noise filter length (DNF)
 *
 * \param periphId       [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param digitalFilter [out]: Pointer to store the filter length (\ref i2c_DigitalFilter_t).
 *                             Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_DigitalFilter( i2c_PeriphId_t periphId, i2c_DigitalFilter_t * const digitalFilter )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId      ) &&
        ( I2C_NULL_PTR   != digitalFilter )    )
    {
        const uint32_t regValue = LL_I2C_GetDigitalFilter( i2c_PeriphConf[ periphId ].PeriphReg );

        if( (uint32_t)I2C_DIGITAL_FILTER_CNT > regValue )
        {
            *digitalFilter = (i2c_DigitalFilter_t)regValue;
            retState       = I2C_REQUEST_OK;
        }
        else
        {
            /* DNF field has 4 bits, value can not be out of range */
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures master addressing mode (7-bit / 10-bit slave address)
 *
 * \pre   No transfer may be running and START may not be pending. Otherwise
 *        \ref I2C_REQUEST_ERROR is returned and no register is modified.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param addrMode [in]: Required addressing mode, value from \ref i2c_AddrMode_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_AddrMode( i2c_PeriphId_t periphId, i2c_AddrMode_t addrMode )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_ADDR_MODE_CNT > addrMode )
    {
        retState = I2c_Check_XferStopped( periphId );
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    if( I2C_REQUEST_OK == retState )
    {
        I2C_TypeDef * const periphReg = i2c_PeriphConf[ periphId ].PeriphReg;

        LL_I2C_SetMasterAddressingMode( periphReg, i2c_AddrModeLut[ addrMode ] );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_I2C_GetMasterAddressingMode( periphReg );

            if( i2c_AddrModeLut[ addrMode ] == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Addressing mode has not yet been applied, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        /* Invalid parameter or transfer is running, configuration change is not allowed */
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads master addressing mode
 *
 * \param periphId  [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param addrMode [out]: Pointer to store the addressing mode (\ref i2c_AddrMode_t).
 *                        Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_AddrMode( i2c_PeriphId_t periphId, i2c_AddrMode_t * const addrMode )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId ) &&
        ( I2C_NULL_PTR   != addrMode )    )
    {
        const uint32_t regValue = LL_I2C_GetMasterAddressingMode( i2c_PeriphConf[ periphId ].PeriphReg );

        for( i2c_AddrMode_t modeIdx = I2C_ADDR_MODE_7BIT; I2C_ADDR_MODE_CNT > modeIdx; modeIdx ++ )
        {
            if( i2c_AddrModeLut[ modeIdx ] == regValue )
            {
                *addrMode = modeIdx;
                retState  = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Addressing mode does not match, keep searching */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads bus busy state (BUSY flag - START detected, cleared after STOP)
 *
 * \note  Bus held busy without running transfer indicates another master or a slave holding
 *        SDA low (bus recovery is not handled by the module).
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param busBusy [out]: Pointer to store the bus state (\ref i2c_FlagState_t). Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_BusState( i2c_PeriphId_t periphId, i2c_FlagState_t * const busBusy )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId ) &&
        ( I2C_NULL_PTR   != busBusy  )    )
    {
        const uint32_t regValue = LL_I2C_IsActiveFlag_BUSY( i2c_PeriphConf[ periphId ].PeriphReg );

        if( 0u != regValue )
        {
            *busBusy = I2C_FLAG_ACTIVE;
        }
        else
        {
            *busBusy = I2C_FLAG_INACTIVE;
        }

        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------------ Data handling ----------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Changes data handling configuration - resources of the previous mode are released and
 *        the new mode is initialized (DMA channels, I2C interrupts in NVIC)
 *
 * \pre   No transfer may be running. Otherwise \ref I2C_REQUEST_ERROR is returned and nothing
 *        is modified.
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration (copied). Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_DataConfig( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Check_DataConfig( periphId, dataConfig );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Check_XferStopped( periphId );
    }
    else
    {
        /* Configuration is invalid */
    }

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Set_XferDeinit( periphId );
    }
    else
    {
        /* Configuration is invalid or transfer is running, nothing is modified */
    }

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Set_XferInit( periphId, dataConfig );
    }
    else
    {
        /* Previous data handling could not be released */
    }

    return ( retState );
}


/**
 * \brief Returns data handling configuration
 *
 * \param periphId    [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [out]: Pointer to store the configuration. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise (also if data handling is not initialized)
 *         returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_DataConfig( i2c_PeriphId_t periphId, i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT       > periphId                              ) &&
        ( I2C_NULL_PTR        != dataConfig                            ) &&
        ( I2C_FUNCTION_ACTIVE == i2c_XferContext[ periphId ].InitState )    )
    {
        *dataConfig = i2c_XferContext[ periphId ].Config;
        retState    = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts master transfer (write / read / write + repeated START + read / address only)
 *
 * The end of the transfer is reported by XferCompleteCallback or ErrorCallback and can be read
 * by I2c_Get_XferState() / I2c_Get_XferError().
 *
 * \pre   Peripheral is enabled, data handling is initialized with a transfer mode (not NONE),
 *        no transfer is running and the bus is not busy. Otherwise \ref I2C_REQUEST_ERROR is
 *        returned and no transfer is started.
 *
 * \param periphId    [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param xferRequest [in]: Pointer to transfer request (copied, buffers are not). Must not be
 *                          NULL, slave address must fit the addressing mode.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if the transfer was started.
 *         Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_XferStart( i2c_PeriphId_t periphId, const i2c_XferRequest_t * const xferRequest )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Check_XferRequest( periphId, xferRequest );

    if( I2C_REQUEST_OK == retState )
    {
        retState = I2c_Check_XferStopped( periphId );
    }
    else
    {
        /* Request is invalid */
    }

    if( I2C_REQUEST_OK == retState )
    {
        I2C_TypeDef * const       periphReg  = i2c_PeriphConf[ periphId ].PeriphReg;
        i2c_XferContext_t * const xferCtx    = &i2c_XferContext[ periphId ];
        const uint32_t            periphEn   = LL_I2C_IsEnabled( periphReg );
        const uint32_t            busBusy    = LL_I2C_IsActiveFlag_BUSY( periphReg );

        if( ( I2C_FUNCTION_ACTIVE == xferCtx->InitState       ) &&
            ( I2C_XFER_MODE_NONE  != xferCtx->Config.XferMode ) &&
            ( 0u                  != periphEn                 ) &&
            ( 0u                  == busBusy                  )    )
        {
            const i2c_XferModeIf_t * const modeIf = &i2c_XferModeLut[ xferCtx->Config.XferMode ];

            xferCtx->Request   = *xferRequest;
            xferCtx->TxIdx     = 0u;
            xferCtx->RxIdx     = 0u;
            xferCtx->XferError = I2C_XFER_ERROR_NONE;

            if( ( 0u  < xferRequest->TxSize ) ||
                ( 0u == xferRequest->RxSize )    )
            {
                /* Write phase first (also address only transfer) */
                xferCtx->Phase          = I2C_XFER_PHASE_WRITE;
                xferCtx->PhaseRemaining = xferRequest->TxSize;
            }
            else
            {
                xferCtx->Phase          = I2C_XFER_PHASE_READ;
                xferCtx->PhaseRemaining = xferRequest->RxSize;
            }

            /* Flags of a previous transfer are cleared (write 1 to clear - not verified by read-back) */
            LL_I2C_WriteReg( periphReg, ICR, I2C_ICR_XFER_FLAGS );

            retState = I2c_Set_TxFlush( periphId );

            if( I2C_REQUEST_OK == retState )
            {
                xferCtx->XferState = I2C_FUNCTION_ACTIVE;

                retState = modeIf->Start( periphId );
            }
            else
            {
                /* Transmit data register could not be flushed */
            }

            if( I2C_REQUEST_OK == retState )
            {
                retState = I2c_Set_XferChunk( periphId, I2C_FUNCTION_ACTIVE );
            }
            else
            {
                /* Transfer mode resources could not be started */
            }

            if( I2C_REQUEST_OK != retState )
            {
                /* Transfer was not started - mode resources are released */
                (void)modeIf->Stop( periphId );
                (void)I2c_Set_Cr2Reset( periphId );
                xferCtx->XferState = I2C_FUNCTION_INACTIVE;
            }
            else
            {
                /* Transfer is running, it is finished from the transfer mode context */
            }
        }
        else
        {
            /* Data handling is not initialized, peripheral is disabled or bus is busy */
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Aborts running transfer - transfer mode resources are stopped and the peripheral is
 *        reset by PE toggling (bus lines are released, no STOP condition is sent). No callback
 *        is called.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems (also if no transfer was running). Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_XferStop( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        if( I2C_FUNCTION_ACTIVE == i2c_XferContext[ periphId ].XferState )
        {
            retState = I2c_Set_XferAbort( periphId );

            i2c_XferContext[ periphId ].XferError = I2C_XFER_ERROR_NONE;
        }
        else
        {
            /* No transfer is running */
            retState = I2C_REQUEST_OK;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads transfer state
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param xferState [out]: Pointer to store the state - \ref I2C_FUNCTION_ACTIVE while the transfer
 *                         is running. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_XferState( i2c_PeriphId_t periphId, i2c_FunctionState_t * const xferState )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId  ) &&
        ( I2C_NULL_PTR   != xferState )    )
    {
        *xferState = i2c_XferContext[ periphId ].XferState;
        retState   = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads error of the last finished transfer
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param xferError [out]: Pointer to store the error - \ref I2C_XFER_ERROR_NONE if the last
 *                         transfer finished successfully. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_XferError( i2c_PeriphId_t periphId, i2c_XferErrorId_t * const xferError )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId  ) &&
        ( I2C_NULL_PTR   != xferError )    )
    {
        *xferError = i2c_XferContext[ periphId ].XferError;
        retState   = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------------------------- Interrupts ------------------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Configures priority of I2C event and error interrupt
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param irqPrio  [in]: Interrupt priority
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_IrqPriority( i2c_PeriphId_t periphId, i2c_IrqPrio_t irqPrio )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        const nvic_RequestState_t evState = Nvic_Set_PeriphIrq_Prio( i2c_PeriphConf[ periphId ].PeriphEvNvic, irqPrio );
        const nvic_RequestState_t erState = Nvic_Set_PeriphIrq_Prio( i2c_PeriphConf[ periphId ].PeriphErNvic, irqPrio );

        if( ( NVIC_REQUEST_OK == evState ) &&
            ( NVIC_REQUEST_OK == erState )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads priority of I2C event interrupt (error interrupt uses the same priority)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param irqPrio [out]: Pointer to store the interrupt priority. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_IrqPriority( i2c_PeriphId_t periphId, i2c_IrqPrio_t * const irqPrio )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId ) &&
        ( I2C_NULL_PTR   != irqPrio  )    )
    {
        nvic_IrqPrio_t            nvicPrio  = 0u;
        const nvic_RequestState_t nvicState = Nvic_Get_PeriphIrq_Prio( i2c_PeriphConf[ periphId ].PeriphEvNvic, &nvicPrio );

        if( NVIC_REQUEST_OK == nvicState )
        {
            *irqPrio = (i2c_IrqPrio_t)nvicPrio;
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/* -------------------------------------------------------------------------- */
/* ------------- Common services for data transfer handlers ----------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Returns registers of I2C peripheral
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param periphReg [out]: Pointer to store the register pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_PeriphReg( i2c_PeriphId_t periphId, I2C_TypeDef ** const periphReg )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId  ) &&
        ( I2C_NULL_PTR   != periphReg )    )
    {
        *periphReg = i2c_PeriphConf[ periphId ].PeriphReg;
        retState   = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns DMAMUX request identifications of I2C peripheral
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param txRequest [out]: Pointer to store transmit request. Must not be NULL.
 * \param rxRequest [out]: Pointer to store receive request. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_PeriphDmaReq( i2c_PeriphId_t periphId, dma_PeriphReqId_t * const txRequest, dma_PeriphReqId_t * const rxRequest )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId  ) &&
        ( I2C_NULL_PTR   != txRequest ) &&
        ( I2C_NULL_PTR   != rxRequest )    )
    {
        *txRequest = i2c_PeriphConf[ periphId ].PeriphDmaTxReq;
        *rxRequest = i2c_PeriphConf[ periphId ].PeriphDmaRxReq;
        retState   = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns data handling context of I2C peripheral
 *
 * \param periphId     [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param xferContext [out]: Pointer to store the context pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Get_XferContext( i2c_PeriphId_t periphId, i2c_XferContext_t ** const xferContext )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId    ) &&
        ( I2C_NULL_PTR   != xferContext )    )
    {
        *xferContext = &i2c_XferContext[ periphId ];
        retState     = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Moves one data byte of the running transfer (ISR / POLL mode): received byte is stored
 *        (RXNE), next byte is written to TXDR (TXIS)
 *
 * \note  Bytes out of the request range are not expected (NBYTES equals the request sizes) -
 *        such received byte is discarded.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param isrFlags [in]: Snapshot of I2C ISR register
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_XferDataStep( i2c_PeriphId_t periphId, i2c_IsrFlags_t isrFlags )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        I2C_TypeDef * const       periphReg = i2c_PeriphConf[ periphId ].PeriphReg;
        i2c_XferContext_t * const xferCtx   = &i2c_XferContext[ periphId ];

        if( I2C_FUNCTION_ACTIVE == xferCtx->XferState )
        {
            /* Received byte (reading of RXDR clears RXNE) */
            if( 0u != ( LL_I2C_ISR_RXNE & isrFlags ) )
            {
                const i2c_Data_t    rxData = LL_I2C_ReceiveData8( periphReg );
                const i2c_DataCnt_t rxIdx  = xferCtx->RxIdx;

                if( xferCtx->Request.RxSize > rxIdx )
                {
                    xferCtx->Request.RxData[ rxIdx ] = rxData;
                    xferCtx->RxIdx                   = rxIdx + 1u;
                }
                else
                {
                    /* Unexpected byte is discarded */
                }
            }
            else
            {
                /* No received byte */
            }

            /* Next byte to be transmitted (writing of TXDR clears TXIS) */
            if( 0u != ( LL_I2C_ISR_TXIS & isrFlags ) )
            {
                const i2c_DataCnt_t txIdx = xferCtx->TxIdx;

                if( xferCtx->Request.TxSize > txIdx )
                {
                    LL_I2C_TransmitData8( periphReg, xferCtx->Request.TxData[ txIdx ] );
                    xferCtx->TxIdx = txIdx + 1u;
                }
                else
                {
                    /* TXIS is not set after the last byte of the request */
                }
            }
            else
            {
                /* Transmit data register is not empty */
            }
        }
        else
        {
            /* No transfer is running */
        }

        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Processes transfer sequencing events of the running transfer (all modes)
 *
 * - BERR: flag is cleared, the transfer continues (spurious in master mode, device errata
 *         ES0430 2.15.2 / ES0431 2.11.2 / ES0523 2.11.2)
 * - ARLO: transfer ends immediately with error
 * - NACKF: error is stored, the transfer ends by STOPF (STOP condition is sent by HW)
 * - STOPF: transfer ends with the stored result
 * - TCR: next chunk of the phase is programmed
 * - TC:  write phase finished, read phase is started by repeated START
 *
 * \note  If the next chunk can not be programmed, the transfer is aborted and
 *        \ref I2C_XFER_ERROR_BUS is reported.
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param isrFlags [in]: Snapshot of I2C ISR register
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_XferEvents( i2c_PeriphId_t periphId, i2c_IsrFlags_t isrFlags )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        I2C_TypeDef * const       periphReg = i2c_PeriphConf[ periphId ].PeriphReg;
        i2c_XferContext_t * const xferCtx   = &i2c_XferContext[ periphId ];

        retState = I2C_REQUEST_OK;

        if( 0u != ( LL_I2C_ISR_BERR & isrFlags ) )
        {
            /* Bus error may be detected spuriously in master mode, the transfer continues normally
             * (device errata ES0430 2.15.2 / ES0431 2.11.2 / ES0523 2.11.2) - flag is cleared only */
            LL_I2C_ClearFlag_BERR( periphReg );
        }
        else
        {
            /* No bus error */
        }

        if( I2C_FUNCTION_ACTIVE != xferCtx->XferState )
        {
            /* No transfer is running */
        }
        else if( 0u != ( LL_I2C_ISR_ARLO & isrFlags ) )
        {
            /* Arbitration lost - peripheral switched to slave mode, bus is released */
            LL_I2C_ClearFlag_ARLO( periphReg );

            retState = I2c_Set_XferEnd( periphId, I2C_XFER_ERROR_ARBITRATION_LOST );
        }
        else
        {
            i2c_RequestState_t chunkState = I2C_REQUEST_OK;

            if( 0u != ( LL_I2C_ISR_NACKF & isrFlags ) )
            {
                /* STOP condition is sent by HW after NACK, transfer ends on STOPF */
                LL_I2C_ClearFlag_NACK( periphReg );
                xferCtx->XferError = I2C_XFER_ERROR_NACK;
            }
            else
            {
                /* Byte / address was acknowledged */
            }

            if( 0u != ( LL_I2C_ISR_STOPF & isrFlags ) )
            {
                LL_I2C_ClearFlag_STOP( periphReg );

                retState = I2c_Set_XferEnd( periphId, xferCtx->XferError );
            }
            else if( I2C_XFER_ERROR_NONE != xferCtx->XferError )
            {
                /* NACK received, waiting for STOP condition */
            }
            else if( 0u != ( LL_I2C_ISR_TCR & isrFlags ) )
            {
                /* Next chunk of the running phase (TCR is cleared by NBYTES write) */
                chunkState = I2c_Set_XferChunk( periphId, I2C_FUNCTION_INACTIVE );
            }
            else if( ( 0u                   != ( LL_I2C_ISR_TC & isrFlags ) ) &&
                     ( I2C_XFER_PHASE_WRITE == xferCtx->Phase               ) &&
                     ( 0u                    < xferCtx->Request.RxSize      )    )
            {
                /* Write phase finished - read phase with repeated START (TC is cleared by START) */
                xferCtx->Phase          = I2C_XFER_PHASE_READ;
                xferCtx->PhaseRemaining = xferCtx->Request.RxSize;

                chunkState = I2c_Set_XferChunk( periphId, I2C_FUNCTION_ACTIVE );
            }
            else
            {
                /* No sequencing event */
            }

            if( I2C_REQUEST_OK != chunkState )
            {
                /* Transfer can not be continued - peripheral is reset and error is reported */
                (void)I2c_Set_XferError( periphId, I2C_XFER_ERROR_BUS );
                retState = I2C_REQUEST_ERROR;
            }
            else
            {
                /* Event processed */
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Terminates the running transfer by an error which is not a bus event (DMA error, failed
 *        transfer sequencing) - the transfer is aborted (peripheral reset by PE toggling releases
 *        the stretched bus), the error is stored and ErrorCallback is called
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param errorId  [in]: Error identification, value from \ref i2c_XferErrorId_t (not NONE)
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
i2c_RequestState_t I2c_Set_XferError( i2c_PeriphId_t periphId, i2c_XferErrorId_t errorId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT       > periphId ) &&
        ( I2C_XFER_ERROR_CNT   > errorId  ) &&
        ( I2C_XFER_ERROR_NONE != errorId  )    )
    {
        i2c_XferContext_t * const xferCtx = &i2c_XferContext[ periphId ];

        if( I2C_FUNCTION_ACTIVE == xferCtx->XferState )
        {
            retState = I2c_Set_XferAbort( periphId );
        }
        else
        {
            /* Transfer already ended - error is only reported */
            retState = I2C_REQUEST_OK;
        }

        xferCtx->XferError = errorId;

        if( I2C_NULL_PTR != xferCtx->Config.ErrorCallback )
        {
            xferCtx->Config.ErrorCallback( errorId );
        }
        else
        {
            /* Error callback is not used */
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Checks peripheral configuration structure (values in range, pins complete)
 *
 * \param i2cConfig [in]: Pointer to configuration structure
 *
 * \return Returns \ref I2C_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Check_Config( const i2c_Config_t * const i2cConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_NULL_PTR           != i2cConfig                ) &&
        ( I2C_PERIPH_CNT          > i2cConfig->PeriphId      ) &&
        ( I2C_CLK_SRC_CNT         > i2cConfig->ClkSrc        ) &&
        ( 0u                      < i2cConfig->BusFreq       ) &&
        ( I2C_BUS_FREQ_MAX_HZ    >= i2cConfig->BusFreq       ) &&
        ( I2C_ANALOG_FILTER_CNT   > i2cConfig->AnalogFilter  ) &&
        ( I2C_DIGITAL_FILTER_CNT  > i2cConfig->DigitalFilter ) &&
        ( I2C_ADDR_MODE_CNT       > i2cConfig->AddrMode      ) &&
        ( I2C_PIN_PULL_CNT        > i2cConfig->PinPull       )    )
    {
        const i2c_RequestState_t sclState = I2c_Check_Pin( i2cConfig->PeriphId, (i2c_PinCode_t)i2cConfig->SclPin, (i2c_PinCode_t)I2C_SCL_PIN_UNUSED );
        const i2c_RequestState_t sdaState = I2c_Check_Pin( i2cConfig->PeriphId, (i2c_PinCode_t)i2cConfig->SdaPin, (i2c_PinCode_t)I2C_SDA_PIN_UNUSED );

        if( ( I2C_REQUEST_OK == sclState ) &&
            ( I2C_REQUEST_OK == sdaState )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            /* Pin configuration is incomplete */
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks encoded SCL / SDA pin - unused pin or pin of the configured peripheral with
 *        valid port, pin and alternate function
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param pinCode    [in]: Encoded pin, value from \ref i2c_SclPin_t or \ref i2c_SdaPin_t
 * \param unusedCode [in]: Encoded value of unused pin (I2C_SCL_PIN_UNUSED / I2C_SDA_PIN_UNUSED)
 *
 * \return Returns \ref I2C_REQUEST_OK if the pin is unused or valid for the peripheral.
 *         Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Check_Pin( i2c_PeriphId_t periphId, i2c_PinCode_t pinCode, i2c_PinCode_t unusedCode )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    const i2c_PinCode_t pinPeriph = I2C_BIT_MASK_DECODE_PERIPH( pinCode );
    const i2c_PinCode_t pinPort   = I2C_BIT_MASK_DECODE_PORT( pinCode );
    const i2c_PinCode_t pinId     = I2C_BIT_MASK_DECODE_PIN( pinCode );
    const i2c_PinCode_t pinAf     = I2C_BIT_MASK_DECODE_AF( pinCode );

    if( unusedCode == pinCode )
    {
        /* Pin is not configured by the module */
        retState = I2C_REQUEST_OK;
    }
    else if( ( (i2c_PinCode_t)periphId          == pinPeriph ) &&
             ( (i2c_PinCode_t)GPIO_PORT_CNT      > pinPort   ) &&
             ( (i2c_PinCode_t)GPIO_PIN_ID_CNT    > pinId     ) &&
             ( (i2c_PinCode_t)GPIO_ALT_FUNC_CNT  > pinAf     )    )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that the peripheral is disabled (PE = 0) - required for TIMINGR, ANFOFF and DNF
 *        change
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Returns \ref I2C_REQUEST_OK if the peripheral is disabled. Otherwise (or for an
 *         invalid periphId) returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Check_PeriphDisabled( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        const uint32_t periphEnabled = LL_I2C_IsEnabled( i2c_PeriphConf[ periphId ].PeriphReg );

        if( 0u == periphEnabled )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            /* Peripheral is enabled */
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that no transfer is running and no START condition is pending
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Returns \ref I2C_REQUEST_OK if no transfer is running. Otherwise (or for an invalid
 *         periphId) returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Check_XferStopped( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        const uint32_t startPending = READ_BIT( i2c_PeriphConf[ periphId ].PeriphReg->CR2, I2C_CR2_START );

        if( ( I2C_FUNCTION_INACTIVE == i2c_XferContext[ periphId ].XferState ) &&
            ( 0u                    == startPending                          )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            /* Transfer is running */
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks transfer request - buffers of used directions and slave address range of the
 *        configured addressing mode
 *
 * \param periphId    [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param xferRequest [in]: Pointer to transfer request
 *
 * \return Returns \ref I2C_REQUEST_OK if the request is valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Check_XferRequest( i2c_PeriphId_t periphId, const i2c_XferRequest_t * const xferRequest )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;
    i2c_AddrMode_t     addrMode = I2C_ADDR_MODE_7BIT;

    if( I2C_NULL_PTR != xferRequest )
    {
        retState = I2c_Get_AddrMode( periphId, &addrMode );
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    if( I2C_REQUEST_OK == retState )
    {
        if( i2c_SlaveAddrMaxLut[ addrMode ] < xferRequest->SlaveAddr )
        {
            /* Slave address out of range */
            retState = I2C_REQUEST_ERROR;
        }
        else if( ( 0u           != xferRequest->TxSize ) &&
                 ( I2C_NULL_PTR == xferRequest->TxData )    )
        {
            /* Write phase without buffer */
            retState = I2C_REQUEST_ERROR;
        }
        else if( ( 0u           != xferRequest->RxSize ) &&
                 ( I2C_NULL_PTR == xferRequest->RxData )    )
        {
            /* Read phase without buffer */
            retState = I2C_REQUEST_ERROR;
        }
        else
        {
            retState = I2C_REQUEST_OK;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks data handling configuration (common part and transfer mode specific part)
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration
 *
 * \return Returns \ref I2C_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Check_DataConfig( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT     > periphId             ) &&
        ( I2C_NULL_PTR      != dataConfig           ) &&
        ( I2C_XFER_MODE_CNT  > dataConfig->XferMode )    )
    {
        retState = i2c_XferModeLut[ dataConfig->XferMode ].CheckConfig( periphId, dataConfig );
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables / disables the peripheral (PE) with read-back verification
 *
 * \note  PE = 0 releases the bus lines, resets I2C state machines and clears status flags
 *        (software reset). The read-back keeps PE low for more than 3 APB clock cycles.
 *
 * \param periphId    [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param enableState [in]: Required state, value from \ref i2c_FunctionState_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_Enable( i2c_PeriphId_t periphId, i2c_FunctionState_t enableState )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        I2C_TypeDef * const periphReg = i2c_PeriphConf[ periphId ].PeriphReg;
        uint32_t            expected  = 0u;

        if( I2C_FUNCTION_ACTIVE == enableState )
        {
            LL_I2C_Enable( periphReg );
            expected = 1u;
        }
        else
        {
            LL_I2C_Disable( periphReg );
            expected = 0u;
        }

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_I2C_IsEnabled( periphReg );

            if( expected == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Peripheral state has not yet been applied, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures SCL / SDA pin as open-drain alternate function (GPIO port clock is enabled by
 *        Gpio_Init())
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param pinCode    [in]: Encoded pin, value from \ref i2c_SclPin_t or \ref i2c_SdaPin_t.
 *                         Unused pin is skipped.
 * \param unusedCode [in]: Encoded value of unused pin (I2C_SCL_PIN_UNUSED / I2C_SDA_PIN_UNUSED)
 * \param pinPull    [in]: Pull resistor configuration, value from \ref i2c_PinPull_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_Pin( i2c_PeriphId_t periphId, i2c_PinCode_t pinCode, i2c_PinCode_t unusedCode, i2c_PinPull_t pinPull )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    retState = I2c_Check_Pin( periphId, pinCode, unusedCode );

    if( ( I2C_REQUEST_OK   == retState ) &&
        ( I2C_PIN_PULL_CNT  > pinPull  )    )
    {
        if( unusedCode != pinCode )
        {
            gpio_Config_t gpioConfig;

            gpioConfig.PortId         = (gpio_PortId_t)I2C_BIT_MASK_DECODE_PORT( pinCode );
            gpioConfig.PinId          = (gpio_PinId_t)I2C_BIT_MASK_DECODE_PIN( pinCode );
            gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
            gpioConfig.PinPull        = i2c_PinPullLut[ pinPull ];
            gpioConfig.PinSpeed       = GPIO_PIN_SPEED_LOW;
            gpioConfig.PinOutType     = GPIO_PIN_OUTPUT_OPENDRAIN;
            gpioConfig.PinAltFunction = (gpio_AltFunction_t)I2C_BIT_MASK_DECODE_AF( pinCode );
            gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

            /* Port clock activation, pin configuration and read-back verification is done by Gpio_Init() */
            const gpio_RequestState_t gpioState = Gpio_Init( &gpioConfig );

            if( GPIO_REQUEST_OK == gpioState )
            {
                retState = I2C_REQUEST_OK;
            }
            else
            {
                retState = I2C_REQUEST_ERROR;
            }
        }
        else
        {
            /* Pin is not configured by the module */
            retState = I2C_REQUEST_OK;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables / disables Fast-mode Plus 20 mA drive of the I2C pins (SYSCFG_CFGR1 I2Cx_FMP)
 *
 * \note  SYSCFG clock is enabled only if the bit has to be changed (the bit reads 0 with
 *        disabled SYSCFG clock).
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param fmpState [in]: Required state, value from \ref i2c_FunctionState_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_FastModePlus( i2c_PeriphId_t periphId, i2c_FunctionState_t fmpState )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        const uint32_t fmpMask  = i2c_PeriphConf[ periphId ].PeriphFmpMask;
        uint32_t       expected = 0u;

        if( I2C_FUNCTION_ACTIVE == fmpState )
        {
            expected = fmpMask;
        }
        else
        {
            expected = 0u;
        }

        if( expected == READ_BIT( SYSCFG->CFGR1, fmpMask ) )
        {
            /* Required state is already applied */
            retState = I2C_REQUEST_OK;
        }
        else
        {
            const rcc_RequestState_t rccState = Rcc_Set_PeriphActive( RCC_PERIPH_SYSCFG );

            if( RCC_REQUEST_OK == rccState )
            {
                if( I2C_FUNCTION_ACTIVE == fmpState )
                {
                    LL_SYSCFG_EnableFastModePlus( fmpMask );
                }
                else
                {
                    LL_SYSCFG_DisableFastModePlus( fmpMask );
                }

                for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    const uint32_t regValue = READ_BIT( SYSCFG->CFGR1, fmpMask );

                    if( expected == regValue )
                    {
                        retState = I2C_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* FMP state has not yet been applied, keep return state as error */
                        retState = I2C_REQUEST_ERROR;
                    }
                }
            }
            else
            {
                /* SYSCFG clock could not be enabled */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks that the I2C kernel clock is usable for the speed mode (device errata)
 *
 * - ES0430 2.15.1 / ES0431 2.11.1 / ES0523 2.11.1: SDA is sampled wrongly when tSU;DAT is shorter
 *   than one I2CCLK period - I2CCLK >= 4 / 10 / 20 MHz for Standard / Fast / Fast-mode Plus.
 * - ES0430 2.15.5 / ES0431 2.11.5 / ES0523 2.11.4: transmission stalls after the first byte when
 *   the APB1 / I2CCLK ratio is between 1.5 and 3 - the ratio is refused.
 *
 * \param periphId  [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param clkFreq   [in]: I2C kernel clock frequency in Hz
 * \param speedMode [in]: Speed mode of the required SCL frequency
 *
 * \return Returns \ref I2C_REQUEST_OK if the kernel clock is usable. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Check_KernelClk( i2c_PeriphId_t periphId, i2c_FreqHz_t clkFreq, i2c_SpeedMode_t speedMode )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT     > periphId  ) &&
        ( I2C_SPEED_MODE_CNT > speedMode )    )
    {
        rcc_FreqHz_t             apbFreq  = 0u;
        const rcc_RequestState_t rccState = Rcc_Get_PeriphClk( i2c_PeriphConf[ periphId ].PeriphRcc[ I2C_CLK_SRC_PCLK ], &apbFreq );

        if( RCC_REQUEST_OK != rccState )
        {
            /* APB clock frequency is not available */
            retState = I2C_REQUEST_ERROR;
        }
        else if( i2c_SpeedCharacLut[ speedMode ].KernelClkMin > clkFreq )
        {
            /* I2CCLK period longer than the minimum data setup time of the speed mode */
            retState = I2C_REQUEST_ERROR;
        }
        else
        {
            /* Ratio in halves: 1.5 < APB1 / I2CCLK < 3  <=>  3 < 2 x APB1 / I2CCLK < 6 */
            const uint64_t apbHalves = (uint64_t)apbFreq * I2C_ERRATA_RATIO_HALF;
            const uint64_t ratioMin  = (uint64_t)clkFreq * I2C_ERRATA_RATIO_MIN_HALVES;
            const uint64_t ratioMax  = (uint64_t)clkFreq * I2C_ERRATA_RATIO_MAX_HALVES;

            if( ratioMin >= apbHalves )
            {
                /* APB1 / I2CCLK <= 1.5 */
                retState = I2C_REQUEST_OK;
            }
            else if( ratioMax <= apbHalves )
            {
                /* APB1 / I2CCLK >= 3 */
                retState = I2C_REQUEST_OK;
            }
            else
            {
                /* APB1 / I2CCLK ratio causes stalled transmission */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns frequency of the selected I2C kernel clock (I2CCLK)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param clkFreq [out]: Pointer to store the frequency in Hz. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems (non-zero frequency). Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Get_KernelClk( i2c_PeriphId_t periphId, i2c_FreqHz_t * const clkFreq )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId ) &&
        ( I2C_NULL_PTR   != clkFreq  )    )
    {
        rcc_PeriphId_t     clkSrcId = RCC_PERIPH_ID_CNT;
        rcc_FreqHz_t       rccFreq  = 0u;
        rcc_RequestState_t rccState = Rcc_Get_PeriphClkSrc( i2c_PeriphConf[ periphId ].PeriphRcc[ I2C_CLK_SRC_PCLK ], &clkSrcId );

        if( RCC_REQUEST_OK == rccState )
        {
            rccState = Rcc_Get_PeriphClk( clkSrcId, &rccFreq );
        }
        else
        {
            /* Selected clock source is not known */
        }

        if( ( RCC_REQUEST_OK == rccState ) &&
            ( 0u              < rccFreq  )    )
        {
            *clkFreq = (i2c_FreqHz_t)rccFreq;
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns speed mode of the SCL frequency
 *
 * \param busFreq    [in]: SCL frequency in Hz
 * \param speedMode [out]: Pointer to store the speed mode. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if the frequency is in range
 *         (1 - \ref I2C_BUS_FREQ_MAX_HZ). Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Get_SpeedMode( i2c_FreqHz_t busFreq, i2c_SpeedMode_t * const speedMode )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_NULL_PTR != speedMode ) &&
        ( 0u            < busFreq   )    )
    {
        for( i2c_SpeedMode_t modeIdx = I2C_SPEED_MODE_STANDARD; I2C_SPEED_MODE_CNT > modeIdx; modeIdx ++ )
        {
            if( i2c_SpeedCharacLut[ modeIdx ].FreqMax >= busFreq )
            {
                *speedMode = modeIdx;
                retState   = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Frequency is above the speed mode maximum, keep searching */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns synchronization delay of one SCL edge: analog filter (min) + digital filter +
 *        I2C_SYNC_CLK_CYCLES x tI2CCLK
 *
 * \param clkPeriod     [in]: I2CCLK period in ps
 * \param analogFilter  [in]: Analog filter state, value from \ref i2c_AnalogFilter_t
 * \param digitalFilter [in]: Digital filter length, value from \ref i2c_DigitalFilter_t
 *
 * \return Synchronization delay in ps
 */
static i2c_TimePs_t I2c_Get_SyncDelay( i2c_TimePs_t clkPeriod, i2c_AnalogFilter_t analogFilter, i2c_DigitalFilter_t digitalFilter )
{
    i2c_TimePs_t syncDelay = ( (i2c_TimePs_t)digitalFilter + I2C_SYNC_CLK_CYCLES ) * clkPeriod;

    if( I2C_ANALOG_FILTER_ENABLED == analogFilter )
    {
        syncDelay += I2C_AF_DELAY_MIN_NS * I2C_PS_PER_NS;
    }
    else
    {
        /* Analog filter does not delay the input */
    }

    return ( syncDelay );
}


/**
 * \brief Returns SCL period: synchronization delay of both edges + SCL low and high periods +
 *        rise and fall time of the speed mode
 *
 * \param syncDelay   [in]: Synchronization delay of one edge in ps (I2c_Get_SyncDelay())
 * \param prescPeriod [in]: Prescaled clock period (tPRESC) in ps
 * \param sclCnt      [in]: Count of tPRESC periods of SCL low and high phase together
 * \param speedMode   [in]: Speed mode (rise / fall time)
 *
 * \return SCL period in ps
 */
static i2c_TimePs_t I2c_Get_SclPeriod( i2c_TimePs_t syncDelay, i2c_TimePs_t prescPeriod, i2c_TimingCnt_t sclCnt, i2c_SpeedMode_t speedMode )
{
    i2c_TimePs_t sclPeriod = ( I2C_SCL_EDGES_CNT * syncDelay ) + ( (i2c_TimePs_t)sclCnt * prescPeriod );

    if( I2C_SPEED_MODE_CNT > speedMode )
    {
        const i2c_SpeedCharac_t * const charac = &i2c_SpeedCharacLut[ speedMode ];

        sclPeriod += ( (i2c_TimePs_t)charac->RiseTime + (i2c_TimePs_t)charac->FallTime ) * I2C_PS_PER_NS;
    }
    else
    {
        /* Invalid speed mode, rise and fall times are not added */
    }

    return ( sclPeriod );
}


/**
 * \brief Calculates TIMINGR value (RM "I2C timings"): for every prescaler SCLDEL / SDADEL are
 *        derived from data setup / hold constraints and SCLL / SCLH from minimum SCL low / high
 *        periods, remaining periods are split between low and high phase. The prescaler with the
 *        smallest SCL period error is used.
 *
 * Constraints:
 * - ( SCLDEL + 1 ) x tPRESC >= tr + tSU;DAT(min)
 * - tf + tHD;DAT(min) - tAF(min) - ( DNF + 3 ) x tI2CCLK <= SDADEL x tPRESC
 *   <= tVD;DAT(max) - tr - tAF(max) - ( DNF + 4 ) x tI2CCLK
 * - tSCLL > tLOW(min), tI2CCLK < ( tSCLL - tAF(min) - tDNF ) / 4, tSCLH >= tHIGH(min)
 * - resulting frequency <= speed mode maximum and within I2C_BUS_FREQ_TOLERANCE_PCT
 *
 * \param clkFreq       [in]: I2C kernel clock frequency in Hz (non-zero)
 * \param busFreq       [in]: Required SCL frequency in Hz (1 - \ref I2C_BUS_FREQ_MAX_HZ)
 * \param analogFilter  [in]: Analog filter state, value from \ref i2c_AnalogFilter_t
 * \param digitalFilter [in]: Digital filter length, value from \ref i2c_DigitalFilter_t
 * \param timingReg    [out]: Pointer to store TIMINGR value. Must not be NULL.
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if valid timing was found.
 *         Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Get_TimingReg( i2c_FreqHz_t clkFreq,
                                             i2c_FreqHz_t busFreq,
                                             i2c_AnalogFilter_t analogFilter,
                                             i2c_DigitalFilter_t digitalFilter,
                                             uint32_t * const timingReg )
{
    i2c_RequestState_t retState  = I2C_REQUEST_ERROR;
    i2c_SpeedMode_t    speedMode = I2C_SPEED_MODE_STANDARD;

    if( ( I2C_NULL_PTR           != timingReg     ) &&
        ( 0u                      < clkFreq       ) &&
        ( I2C_ANALOG_FILTER_CNT   > analogFilter  ) &&
        ( I2C_DIGITAL_FILTER_CNT  > digitalFilter )    )
    {
        retState = I2c_Get_SpeedMode( busFreq, &speedMode );
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    if( I2C_REQUEST_OK == retState )
    {
        const i2c_SpeedCharac_t * const charac     = &i2c_SpeedCharacLut[ speedMode ];
        const i2c_TimePs_t              clkPeriod  = I2C_PS_PER_S / clkFreq;
        const i2c_TimePs_t              busPeriod  = I2C_PS_PER_S / busFreq;
        const i2c_TimePs_t              busPerMin  = I2C_PS_PER_S / charac->FreqMax;
        const i2c_TimePs_t              riseTime   = charac->RiseTime     * I2C_PS_PER_NS;
        const i2c_TimePs_t              fallTime   = charac->FallTime     * I2C_PS_PER_NS;
        const i2c_TimePs_t              lowMin     = charac->SclLowMin    * I2C_PS_PER_NS;
        const i2c_TimePs_t              highMin    = charac->SclHighMin   * I2C_PS_PER_NS;
        const i2c_TimePs_t              setupMin   = charac->DataSetupMin * I2C_PS_PER_NS;
        const i2c_TimePs_t              holdMin    = charac->DataHoldMin  * I2C_PS_PER_NS;
        const i2c_TimePs_t              validMax   = charac->DataValidMax * I2C_PS_PER_NS;
        const i2c_TimePs_t              dnfDelay   = (i2c_TimePs_t)digitalFilter * clkPeriod;
        const i2c_TimePs_t              syncDelay  = I2c_Get_SyncDelay( clkPeriod, analogFilter, digitalFilter );
        const i2c_TimePs_t              fixedDelay = I2c_Get_SclPeriod( syncDelay, 0u, 0u, speedMode );
        i2c_TimePs_t                    afMin      = 0u;
        i2c_TimePs_t                    afMax      = 0u;
        i2c_TimePs_t                    bestError  = busPeriod;
        uint32_t                        bestTiming = 0u;
        i2c_FunctionState_t             found      = I2C_FUNCTION_INACTIVE;

        if( I2C_ANALOG_FILTER_ENABLED == analogFilter )
        {
            afMin = I2C_AF_DELAY_MIN_NS * I2C_PS_PER_NS;
            afMax = I2C_AF_DELAY_MAX_NS * I2C_PS_PER_NS;
        }
        else
        {
            /* Analog filter does not delay the input */
        }

        /* Data hold constraints independent of the prescaler */
        const i2c_TimePs_t sdaMinPos = fallTime + holdMin;
        const i2c_TimePs_t sdaMinNeg = afMin + dnfDelay + ( I2C_SDADEL_MIN_CLK_CYCLES * clkPeriod );
        const i2c_TimePs_t sdaMaxNeg = riseTime + afMax + dnfDelay + ( I2C_SDADEL_MAX_CLK_CYCLES * clkPeriod );

        for( i2c_TimingCnt_t presc = 0u; I2C_TIMING_PRESC_MAX >= presc; presc ++ )
        {
            const i2c_TimePs_t prescPeriod = ( (i2c_TimePs_t)presc + I2C_TIMING_REG_OFFSET ) * clkPeriod;

            /* SCLDEL: ( SCLDEL + 1 ) x tPRESC >= tr + tSU;DAT(min) */
            i2c_TimingCnt_t sclDelCnt = (i2c_TimingCnt_t)I2C_DIV_ROUND_UP( riseTime + setupMin, prescPeriod );

            if( I2C_TIMING_REG_OFFSET > sclDelCnt )
            {
                sclDelCnt = I2C_TIMING_REG_OFFSET;
            }
            else
            {
                /* Setup time requires at least one prescaled period */
            }

            /* SDADEL range */
            i2c_TimingCnt_t     sdaDelMin   = 0u;
            i2c_TimingCnt_t     sdaDelMax   = 0u;
            i2c_FunctionState_t sdaMaxValid = I2C_FUNCTION_INACTIVE;

            if( sdaMinPos > sdaMinNeg )
            {
                sdaDelMin = (i2c_TimingCnt_t)I2C_DIV_ROUND_UP( sdaMinPos - sdaMinNeg, prescPeriod );
            }
            else
            {
                /* Hold time is covered by the input delays */
            }

            if( validMax >= sdaMaxNeg )
            {
                sdaDelMax   = (i2c_TimingCnt_t)( ( validMax - sdaMaxNeg ) / prescPeriod );
                sdaMaxValid = I2C_FUNCTION_ACTIVE;
            }
            else
            {
                /* Data valid time can not be met with this kernel clock */
            }

            /* SCL low: tSYNC + n x tPRESC > tLOW(min) and n x tPRESC > 2 x tI2CCLK */
            i2c_TimingCnt_t lowCnt   = 1u;
            i2c_TimingCnt_t highCnt  = 1u;
            const i2c_TimingCnt_t lowClkCnt = (i2c_TimingCnt_t)( ( ( I2C_SCLL_MIN_CLK_CYCLES - I2C_SYNC_CLK_CYCLES ) * clkPeriod ) / prescPeriod ) + 1u;

            if( lowMin >= syncDelay )
            {
                lowCnt = (i2c_TimingCnt_t)( ( lowMin - syncDelay ) / prescPeriod ) + 1u;
            }
            else
            {
                /* Synchronization delay covers the minimum low period */
            }

            if( lowClkCnt > lowCnt )
            {
                lowCnt = lowClkCnt;
            }
            else
            {
                /* Low period is long enough for the kernel clock */
            }

            /* SCL high: tSYNC + n x tPRESC >= tHIGH(min) */
            if( highMin > syncDelay )
            {
                highCnt = (i2c_TimingCnt_t)I2C_DIV_ROUND_UP( highMin - syncDelay, prescPeriod );
            }
            else
            {
                /* Synchronization delay covers the minimum high period */
            }

            /* Periods available for the required frequency, surplus split between low and high */
            i2c_TimingCnt_t totalCnt = lowCnt + highCnt;

            if( busPeriod > fixedDelay )
            {
                const i2c_TimingCnt_t reqCnt = (i2c_TimingCnt_t)I2C_DIV_ROUND_NEAREST( busPeriod - fixedDelay, prescPeriod );

                if( reqCnt > totalCnt )
                {
                    const i2c_TimingCnt_t extraCnt = reqCnt - totalCnt;

                    lowCnt  += extraCnt - ( extraCnt / I2C_SCL_EDGES_CNT );
                    highCnt += extraCnt / I2C_SCL_EDGES_CNT;
                    totalCnt = reqCnt;
                }
                else
                {
                    /* Minimum periods are longer than required - frequency will be lower */
                }
            }
            else
            {
                /* Required period is shorter than the fixed delays */
            }

            const i2c_TimePs_t sclPeriod = I2c_Get_SclPeriod( syncDelay, prescPeriod, totalCnt, speedMode );
            i2c_TimePs_t       sclError  = 0u;

            if( sclPeriod > busPeriod )
            {
                sclError = sclPeriod - busPeriod;
            }
            else
            {
                sclError = busPeriod - sclPeriod;
            }

            if( ( I2C_TIMING_DEL_MAX     >= ( sclDelCnt - I2C_TIMING_REG_OFFSET ) ) &&
                ( I2C_TIMING_DEL_MAX     >= sdaDelMin                             ) &&
                ( I2C_FUNCTION_ACTIVE    == sdaMaxValid                           ) &&
                ( sdaDelMax              >= sdaDelMin                             ) &&
                ( I2C_TIMING_SCL_CNT_MAX >= lowCnt                                ) &&
                ( I2C_TIMING_SCL_CNT_MAX >= highCnt                               ) &&
                ( busPerMin              <= sclPeriod                             ) &&
                ( bestError               > sclError                              )    )
            {
                bestError  = sclError;
                bestTiming = __LL_I2C_CONVERT_TIMINGS( presc,
                                                       sclDelCnt - I2C_TIMING_REG_OFFSET,
                                                       sdaDelMin,
                                                       highCnt   - I2C_TIMING_REG_OFFSET,
                                                       lowCnt    - I2C_TIMING_REG_OFFSET );
                found      = I2C_FUNCTION_ACTIVE;
            }
            else
            {
                /* Prescaler does not give valid or better timing */
            }
        }

        if( ( I2C_FUNCTION_ACTIVE         == found                                      ) &&
            ( ( bestError * I2C_PERCENT ) <= ( busPeriod * I2C_BUS_FREQ_TOLERANCE_PCT ) )    )
        {
            *timingReg = bestTiming;
            retState   = I2C_REQUEST_OK;
        }
        else
        {
            /* Frequency can not be reached with the kernel clock */
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes data handling - configuration is copied, transfer mode resources are
 *        initialized and I2C interrupts are enabled in NVIC (DMA / ISR mode)
 *
 * \note  On failure the partially initialized data handling is released.
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to checked data handling configuration
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_XferInit( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT     > periphId             ) &&
        ( I2C_NULL_PTR      != dataConfig           ) &&
        ( I2C_XFER_MODE_CNT  > dataConfig->XferMode )    )
    {
        i2c_XferContext_t * const xferCtx = &i2c_XferContext[ periphId ];

        xferCtx->Config    = *dataConfig;
        xferCtx->XferState = I2C_FUNCTION_INACTIVE;
        xferCtx->XferError = I2C_XFER_ERROR_NONE;
        xferCtx->InitState = I2C_FUNCTION_ACTIVE;

        retState = i2c_XferModeLut[ dataConfig->XferMode ].Init( periphId );

        const i2c_FunctionState_t irqUsed = I2c_Get_IrqUsed( dataConfig );

        if( ( I2C_REQUEST_OK      == retState ) &&
            ( I2C_FUNCTION_ACTIVE == irqUsed  )    )
        {
            retState = I2c_Set_IrqInit( periphId, dataConfig->IrqPriority );
        }
        else
        {
            /* Mode initialization failed or interrupts are not used */
        }

        if( I2C_REQUEST_OK != retState )
        {
            (void)I2c_Set_XferDeinit( periphId );
        }
        else
        {
            /* Data handling is initialized */
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes data handling - running transfer is aborted, transfer mode resources are
 *        released and I2C interrupts are disabled in NVIC
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems (also if data handling was not initialized).
 *         Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_XferDeinit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        i2c_XferContext_t * const xferCtx = &i2c_XferContext[ periphId ];

        if( I2C_FUNCTION_ACTIVE == xferCtx->InitState )
        {
            i2c_RequestState_t abortState = I2C_REQUEST_OK;

            if( I2C_FUNCTION_ACTIVE == xferCtx->XferState )
            {
                abortState = I2c_Set_XferAbort( periphId );
            }
            else
            {
                /* No transfer is running */
            }

            const i2c_RequestState_t deinitState = i2c_XferModeLut[ xferCtx->Config.XferMode ].Deinit( periphId );
            const i2c_RequestState_t irqState    = I2c_Set_IrqDeinit( periphId );

            xferCtx->InitState = I2C_FUNCTION_INACTIVE;

            if( ( I2C_REQUEST_OK == abortState  ) &&
                ( I2C_REQUEST_OK == deinitState ) &&
                ( I2C_REQUEST_OK == irqState    )    )
            {
                retState = I2C_REQUEST_OK;
            }
            else
            {
                retState = I2C_REQUEST_ERROR;
            }
        }
        else
        {
            /* Data handling is not initialized */
            retState = I2C_REQUEST_OK;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns if the data handling configuration uses I2C interrupts (DMA or ISR mode)
 *
 * \param dataConfig [in]: Pointer to data handling configuration
 *
 * \return \ref I2C_FUNCTION_ACTIVE if I2C interrupts are used, otherwise \ref I2C_FUNCTION_INACTIVE
 */
static i2c_FunctionState_t I2c_Get_IrqUsed( const i2c_DataConfig_t * const dataConfig )
{
    i2c_FunctionState_t irqUsed = I2C_FUNCTION_INACTIVE;

    if( ( I2C_NULL_PTR != dataConfig ) &&
        ( ( I2C_XFER_MODE_DMA == dataConfig->XferMode ) ||
          ( I2C_XFER_MODE_ISR == dataConfig->XferMode )    )    )
    {
        irqUsed = I2C_FUNCTION_ACTIVE;
    }
    else
    {
        irqUsed = I2C_FUNCTION_INACTIVE;
    }

    return ( irqUsed );
}


/**
 * \brief Registers the interrupt handler, configures priority and enables I2C event and error
 *        interrupt in NVIC
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param irqPrio  [in]: Interrupt priority
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_IrqInit( i2c_PeriphId_t periphId, i2c_IrqPrio_t irqPrio )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        const i2c_PeriphConfigStruct_t * const periphConf = &i2c_PeriphConf[ periphId ];

        const nvic_RequestState_t evHandlerState = Nvic_Set_PeriphIrq_Handler( periphConf->PeriphEvNvic, periphConf->PeriphIsr );
        const nvic_RequestState_t erHandlerState = Nvic_Set_PeriphIrq_Handler( periphConf->PeriphErNvic, periphConf->PeriphIsr );
        const i2c_RequestState_t  prioState      = I2c_Set_IrqPriority( periphId, irqPrio );
        const nvic_RequestState_t evActState     = Nvic_Set_PeriphIrq_Active( periphConf->PeriphEvNvic );
        const nvic_RequestState_t erActState     = Nvic_Set_PeriphIrq_Active( periphConf->PeriphErNvic );

        if( ( NVIC_REQUEST_OK == evHandlerState ) &&
            ( NVIC_REQUEST_OK == erHandlerState ) &&
            ( I2C_REQUEST_OK  == prioState      ) &&
            ( NVIC_REQUEST_OK == evActState     ) &&
            ( NVIC_REQUEST_OK == erActState     )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables I2C event and error interrupt in NVIC
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_IrqDeinit( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        const nvic_RequestState_t evState = Nvic_Set_PeriphIrq_Inactive( i2c_PeriphConf[ periphId ].PeriphEvNvic );
        const nvic_RequestState_t erState = Nvic_Set_PeriphIrq_Inactive( i2c_PeriphConf[ periphId ].PeriphErNvic );

        if( ( NVIC_REQUEST_OK == evState ) &&
            ( NVIC_REQUEST_OK == erState )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Programs the next chunk of the running phase into CR2 (NBYTES, RELOAD / AUTOEND / soft
 *        end) and optionally generates START condition
 *
 * - more than 255 bytes remaining in the phase: 255 bytes with RELOAD
 * - last chunk of the write phase followed by the read phase: soft end (TC -> repeated START)
 * - last chunk of the last phase: AUTOEND (STOP condition after the last byte)
 *
 * \note  START bit is cleared by HW after the address is sent - it is not verified by read-back.
 *
 * \param periphId  [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param startCond [in]: \ref I2C_FUNCTION_ACTIVE - START condition (phase start),
 *                        \ref I2C_FUNCTION_INACTIVE - reload of the running phase
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_XferChunk( i2c_PeriphId_t periphId, i2c_FunctionState_t startCond )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        I2C_TypeDef * const       periphReg = i2c_PeriphConf[ periphId ].PeriphReg;
        i2c_XferContext_t * const xferCtx   = &i2c_XferContext[ periphId ];
        const uint32_t            addrSize  = LL_I2C_GetMasterAddressingMode( periphReg );
        i2c_DataCnt_t             chunkSize = xferCtx->PhaseRemaining;
        uint32_t                  endMode   = LL_I2C_MODE_AUTOEND;
        uint32_t                  slaveAddr = (uint32_t)xferCtx->Request.SlaveAddr;
        uint32_t                  request   = LL_I2C_GENERATE_NOSTARTSTOP;
        uint32_t                  readBit   = LL_I2C_REQUEST_WRITE;

        if( I2C_NBYTES_MAX < chunkSize )
        {
            chunkSize = I2C_NBYTES_MAX;
            endMode   = LL_I2C_MODE_RELOAD;
        }
        else if( ( I2C_XFER_PHASE_WRITE == xferCtx->Phase          ) &&
                 ( 0u                    < xferCtx->Request.RxSize )    )
        {
            /* Read phase follows with repeated START */
            endMode = LL_I2C_MODE_SOFTEND;
        }
        else
        {
            /* Last chunk of the transfer - STOP condition after the last byte */
            endMode = LL_I2C_MODE_AUTOEND;
        }

        xferCtx->PhaseRemaining = xferCtx->PhaseRemaining - chunkSize;

        if( LL_I2C_ADDRESSING_MODE_7BIT == addrSize )
        {
            slaveAddr = slaveAddr << I2C_SLAVE_ADDR_7BIT_SHIFT;
        }
        else
        {
            /* 10-bit address is placed in SADD[9:0] */
        }

        if( I2C_XFER_PHASE_READ == xferCtx->Phase )
        {
            readBit = LL_I2C_REQUEST_READ;
            request = LL_I2C_GENERATE_START_READ;
        }
        else
        {
            readBit = LL_I2C_REQUEST_WRITE;
            request = LL_I2C_GENERATE_START_WRITE;
        }

        if( I2C_FUNCTION_ACTIVE != startCond )
        {
            /* Reload - RD_WRN of the running phase is kept */
            request = LL_I2C_GENERATE_NOSTARTSTOP;
        }
        else
        {
            /* Phase start */
        }

        const uint32_t expected = ( slaveAddr & I2C_CR2_SADD ) | addrSize | endMode | readBit |
                                  ( ( (uint32_t)chunkSize << I2C_CR2_NBYTES_Pos ) & I2C_CR2_NBYTES );

        LL_I2C_HandleTransfer( periphReg, slaveAddr, addrSize, (uint32_t)chunkSize, endMode, request );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = READ_BIT( periphReg->CR2, I2C_CR2_XFER_MASK );

            if( expected == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Transfer configuration has not yet been applied, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Ends the running transfer - transfer mode resources are stopped, TXDR is flushed, CR2 is
 *        cleared and XferCompleteCallback / ErrorCallback is called
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param errorId  [in]: Result of the transfer, value from \ref i2c_XferErrorId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_XferEnd( i2c_PeriphId_t periphId, i2c_XferErrorId_t errorId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT     > periphId ) &&
        ( I2C_XFER_ERROR_CNT > errorId  )    )
    {
        i2c_XferContext_t * const      xferCtx  = &i2c_XferContext[ periphId ];
        const i2c_XferModeIf_t * const modeIf   = &i2c_XferModeLut[ xferCtx->Config.XferMode ];
        i2c_XferErrorId_t              xferErr  = errorId;

        const i2c_RequestState_t stopState  = modeIf->Stop( periphId );
        const i2c_RequestState_t doneState  = modeIf->CheckDone( periphId );
        const i2c_RequestState_t flushState = I2c_Set_TxFlush( periphId );
        const i2c_RequestState_t cr2State   = I2c_Set_Cr2Reset( periphId );

        if( ( I2C_XFER_ERROR_NONE == xferErr   ) &&
            ( I2C_REQUEST_OK      != doneState )    )
        {
            /* STOP was detected, but DMA did not move all bytes */
            xferErr = I2C_XFER_ERROR_DMA_TRANSFER;
        }
        else
        {
            /* Result is kept */
        }

        xferCtx->XferError = xferErr;
        xferCtx->XferState = I2C_FUNCTION_INACTIVE;

        if( I2C_XFER_ERROR_NONE == xferErr )
        {
            if( I2C_NULL_PTR != xferCtx->Config.XferCompleteCallback )
            {
                xferCtx->Config.XferCompleteCallback( );
            }
            else
            {
                /* Transfer complete callback is not used */
            }
        }
        else
        {
            if( I2C_NULL_PTR != xferCtx->Config.ErrorCallback )
            {
                xferCtx->Config.ErrorCallback( xferErr );
            }
            else
            {
                /* Error callback is not used */
            }
        }

        if( ( I2C_REQUEST_OK == stopState  ) &&
            ( I2C_REQUEST_OK == flushState ) &&
            ( I2C_REQUEST_OK == cr2State   )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Aborts the running transfer without callback - transfer mode resources are stopped,
 *        the peripheral is reset by PE toggling (if it was enabled) and CR2 is cleared
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_XferAbort( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        i2c_XferContext_t * const xferCtx     = &i2c_XferContext[ periphId ];
        const uint32_t            periphEn    = LL_I2C_IsEnabled( i2c_PeriphConf[ periphId ].PeriphReg );
        const i2c_RequestState_t  stopState   = i2c_XferModeLut[ xferCtx->Config.XferMode ].Stop( periphId );
        i2c_RequestState_t        resetState  = I2C_REQUEST_OK;

        xferCtx->XferState = I2C_FUNCTION_INACTIVE;

        if( 0u != periphEn )
        {
            /* Software reset - bus lines are released, state machines and flags are reset */
            resetState = I2c_Set_Enable( periphId, I2C_FUNCTION_INACTIVE );

            if( I2C_REQUEST_OK == resetState )
            {
                resetState = I2c_Set_Enable( periphId, I2C_FUNCTION_ACTIVE );
            }
            else
            {
                /* Peripheral could not be disabled */
            }
        }
        else
        {
            /* Peripheral is disabled, communication is already released */
        }

        const i2c_RequestState_t cr2State = I2c_Set_Cr2Reset( periphId );

        if( ( I2C_REQUEST_OK == stopState  ) &&
            ( I2C_REQUEST_OK == resetState ) &&
            ( I2C_REQUEST_OK == cr2State   )    )
        {
            retState = I2C_REQUEST_OK;
        }
        else
        {
            retState = I2C_REQUEST_ERROR;
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears transfer fields of CR2 (slave address, NBYTES, RELOAD, AUTOEND, RD_WRN, HEAD10R),
 *        addressing mode (ADD10) is kept
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_Cr2Reset( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        I2C_TypeDef * const periphReg = i2c_PeriphConf[ periphId ].PeriphReg;

        CLEAR_BIT( periphReg->CR2, I2C_CR2_RESET_MASK );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = READ_BIT( periphReg->CR2, I2C_CR2_RESET_MASK );

            if( 0u == regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* CR2 has not yet been cleared, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Flushes transmit data register (TXE is set by SW - byte written to TXDR by a terminated
 *        transfer is discarded)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Function processing state. Returns \ref I2C_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_Set_TxFlush( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        I2C_TypeDef * const periphReg = i2c_PeriphConf[ periphId ].PeriphReg;

        LL_I2C_ClearFlag_TXE( periphReg );

        for( i2c_TimeoutCnt_t iterationCnt = 0u; I2C_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            const uint32_t regValue = LL_I2C_IsActiveFlag_TXE( periphReg );

            if( 0u != regValue )
            {
                retState = I2C_REQUEST_OK;
                break;
            }
            else
            {
                /* Transmit data register has not yet been flushed, keep return state as error */
                retState = I2C_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks configuration of I2C_XFER_MODE_NONE (no mode specific resources)
 *
 * \param periphId   [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref I2C_REQUEST_OK if the parameters are valid. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_None_Check_Config( i2c_PeriphId_t periphId, const i2c_DataConfig_t * const dataConfig )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( ( I2C_PERIPH_CNT  > periphId   ) &&
        ( I2C_NULL_PTR   != dataConfig )    )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Transfer mode handler without resources (NONE mode, CheckDone of ISR / POLL mode)
 *
 * \param periphId [in]: I2C peripheral identification, value from \ref i2c_PeriphId_t
 *
 * \return Returns \ref I2C_REQUEST_OK for a valid periphId. Otherwise returns
 *         \ref I2C_REQUEST_ERROR.
 */
static i2c_RequestState_t I2c_None_Xfer( i2c_PeriphId_t periphId )
{
    i2c_RequestState_t retState = I2C_REQUEST_ERROR;

    if( I2C_PERIPH_CNT > periphId )
    {
        retState = I2C_REQUEST_OK;
    }
    else
    {
        retState = I2C_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

#ifdef I2C1
/**
 * \brief I2C1 event and error interrupt service routine
 */
static void I2c_I2c1_IsrHandler( void )
{
    (void)I2c_Isr_Handler( I2C_PERIPH_1 );
    __DSB();    /* Cortex-M4 erratum 838869 (ES0430 / ES0431 / ES0523 2.1.3): stores completed before the exception return */
}
#endif /* I2C1 */

#ifdef I2C2
/**
 * \brief I2C2 event and error interrupt service routine
 */
static void I2c_I2c2_IsrHandler( void )
{
    (void)I2c_Isr_Handler( I2C_PERIPH_2 );
    __DSB();    /* Cortex-M4 erratum 838869 (ES0430 / ES0431 / ES0523 2.1.3): stores completed before the exception return */
}
#endif /* I2C2 */

#ifdef I2C3
/**
 * \brief I2C3 event and error interrupt service routine
 */
static void I2c_I2c3_IsrHandler( void )
{
    (void)I2c_Isr_Handler( I2C_PERIPH_3 );
    __DSB();    /* Cortex-M4 erratum 838869 (ES0430 / ES0431 / ES0523 2.1.3): stores completed before the exception return */
}
#endif /* I2C3 */

#ifdef I2C4
/**
 * \brief I2C4 event and error interrupt service routine
 */
static void I2c_I2c4_IsrHandler( void )
{
    (void)I2c_Isr_Handler( I2C_PERIPH_4 );
    __DSB();    /* Cortex-M4 erratum 838869 (ES0430 / ES0431 / ES0523 2.1.3): stores completed before the exception return */
}
#endif /* I2C4 */

/* ================================ TASKS =================================== */
