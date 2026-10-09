/**
 * \defgroup I2c I2c
 * \brief I2c module
 */

/**
 * \author Mr.Nobody
 * \file I2c_Types.h
 * \ingroup I2c
 * \brief Inter-Integrated Circuit (I2C) MCAL module global types definition
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 * \note STM32G4 family: configuration structures and type names are common for
 *       all families, enumerations with hardware specific values (peripherals,
 *       clock sources, pins, DMA) follow STM32G4 I2C.
 *
 */

#ifndef I2C_I2C_TYPES_H
#define I2C_I2C_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Dma_Types.h"                      /* DMA types definitions          */
#include "Stm32_i2c.h"                      /* I2C RAL functionality          */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define I2C_NULL_PTR                        ( ( void* ) 0u )

/** Maximum bus frequency in Hz (Fast-mode Plus) */
#define I2C_BUS_FREQ_MAX_HZ                 ( 1000000u )

/** Maximum 7-bit slave address */
#define I2C_SLAVE_ADDR_7BIT_MAX             ( 0x7Fu )

/** Maximum 10-bit slave address */
#define I2C_SLAVE_ADDR_10BIT_MAX            ( 0x3FFu )

/** Peripheral identification bit offset in encoded pin value */
#define I2C_BIT_MASK_PERIPH_BIT_OFFSET      ( 15u )

/** Port identification bit offset in encoded pin value */
#define I2C_BIT_MASK_PORT_BIT_OFFSET        ( 10u )

/** Pin identification bit offset in encoded pin value */
#define I2C_BIT_MASK_PIN_BIT_OFFSET         ( 5u )

/** Alternate function identification bit offset in encoded pin value */
#define I2C_BIT_MASK_AF_BIT_OFFSET          ( 0u )

/** Mask of one field (5 bits) in encoded pin value */
#define I2C_BIT_MASK_FIELD                  ( 0x1Fu )


/* ========================== EXPORTED MACROS =============================== */

/** Encode pin configuration (peripheral, port, pin, alternate function) into single bit-mask */
#define I2C_PIN_BIT_MASK_ENCODE( PERIPH_ID, PORT_ID, PIN_ID, AF_ID )    ( ( (PERIPH_ID) << I2C_BIT_MASK_PERIPH_BIT_OFFSET ) | \
                                                                          ( (PORT_ID)   << I2C_BIT_MASK_PORT_BIT_OFFSET   ) | \
                                                                          ( (PIN_ID)    << I2C_BIT_MASK_PIN_BIT_OFFSET    ) | \
                                                                          ( (AF_ID)     << I2C_BIT_MASK_AF_BIT_OFFSET     )   )

/** Extract peripheral ID from encoded pin value */
#define I2C_BIT_MASK_DECODE_PERIPH( CODED_VAL )     ( ( (CODED_VAL) >> I2C_BIT_MASK_PERIPH_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

/** Extract port ID from encoded pin value */
#define I2C_BIT_MASK_DECODE_PORT( CODED_VAL )       ( ( (CODED_VAL) >> I2C_BIT_MASK_PORT_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

/** Extract pin ID from encoded pin value */
#define I2C_BIT_MASK_DECODE_PIN( CODED_VAL )        ( ( (CODED_VAL) >> I2C_BIT_MASK_PIN_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

/** Extract alternate function ID from encoded pin value */
#define I2C_BIT_MASK_DECODE_AF( CODED_VAL )         ( ( (CODED_VAL) >> I2C_BIT_MASK_AF_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t i2c_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t i2c_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t i2c_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    i2c_MajorVersion_t Major; /**< Major version */
    i2c_MinorVersion_t Minor; /**< Minor version */
    i2c_PatchVersion_t Patch; /**< Patch version */
}   i2c_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    I2C_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    I2C_FUNCTION_ACTIVE         /**< Function status is active   */
}   i2c_FunctionState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    I2C_REQUEST_ERROR = 0u, /**< Processing request failed  */
    I2C_REQUEST_OK          /**< Processing request succeed */
}   i2c_RequestState_t;


/** Flag states enumeration */
typedef enum
{
    I2C_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    I2C_FLAG_ACTIVE         /**< Active flag state   */
}   i2c_FlagState_t;


/** \brief Frequency value represented in Hz */
typedef uint32_t i2c_FreqHz_t;

/** \brief Slave address (7-bit: 0 - 0x7F, 10-bit: 0 - 0x3FF, not shifted, without R/W bit) */
typedef uint16_t i2c_SlaveAddr_t;

/** \brief Type representing transferred data byte */
typedef uint8_t i2c_Data_t;

/** \brief Type representing count of transferred data bytes */
typedef uint16_t i2c_DataCnt_t;

/** \brief Interrupt priority type definition */
typedef uint32_t i2c_IrqPrio_t;


/** \brief I2C peripheral identification */
typedef enum
{
#ifdef I2C1
    I2C_PERIPH_1 = 0u, /**< I2C peripheral 1 ID */
#endif
#ifdef I2C2
    I2C_PERIPH_2,      /**< I2C peripheral 2 ID */
#endif
#ifdef I2C3
    I2C_PERIPH_3,      /**< I2C peripheral 3 ID */
#endif
#ifdef I2C4
    I2C_PERIPH_4,      /**< I2C peripheral 4 ID */
#endif
    I2C_PERIPH_CNT     /**< Count of I2C peripherals */
}   i2c_PeriphId_t;


/** \brief I2C kernel clock (I2CCLK) source */
typedef enum
{
    I2C_CLK_SRC_PCLK = 0u, /**< APB1 clock (PCLK1)                                               */
    I2C_CLK_SRC_SYSCLK,    /**< System clock (SYSCLK)                                            */
    I2C_CLK_SRC_HSI,       /**< 16 MHz High Speed Internal oscillator (HSI16) - up to Fast-mode
                                (Fast-mode Plus needs I2CCLK >= 20 MHz, device errata)          */
    I2C_CLK_SRC_CNT        /**< Count of clock sources                                            */
}   i2c_ClkSrc_t;


/** \brief Analog noise filter state (filter suppresses spikes shorter than 50 ns) */
typedef enum
{
    I2C_ANALOG_FILTER_ENABLED = 0u, /**< Analog noise filter enabled  */
    I2C_ANALOG_FILTER_DISABLED,     /**< Analog noise filter disabled */
    I2C_ANALOG_FILTER_CNT           /**< Count of options             */
}   i2c_AnalogFilter_t;


/** \brief Digital noise filter - spikes shorter than the given count of I2CCLK periods are suppressed */
typedef enum
{
    I2C_DIGITAL_FILTER_OFF = 0u, /**< Digital noise filter disabled        */
    I2C_DIGITAL_FILTER_1,        /**< Filter capability up to 1 tI2CCLK    */
    I2C_DIGITAL_FILTER_2,        /**< Filter capability up to 2 tI2CCLK    */
    I2C_DIGITAL_FILTER_3,        /**< Filter capability up to 3 tI2CCLK    */
    I2C_DIGITAL_FILTER_4,        /**< Filter capability up to 4 tI2CCLK    */
    I2C_DIGITAL_FILTER_5,        /**< Filter capability up to 5 tI2CCLK    */
    I2C_DIGITAL_FILTER_6,        /**< Filter capability up to 6 tI2CCLK    */
    I2C_DIGITAL_FILTER_7,        /**< Filter capability up to 7 tI2CCLK    */
    I2C_DIGITAL_FILTER_8,        /**< Filter capability up to 8 tI2CCLK    */
    I2C_DIGITAL_FILTER_9,        /**< Filter capability up to 9 tI2CCLK    */
    I2C_DIGITAL_FILTER_10,       /**< Filter capability up to 10 tI2CCLK   */
    I2C_DIGITAL_FILTER_11,       /**< Filter capability up to 11 tI2CCLK   */
    I2C_DIGITAL_FILTER_12,       /**< Filter capability up to 12 tI2CCLK   */
    I2C_DIGITAL_FILTER_13,       /**< Filter capability up to 13 tI2CCLK   */
    I2C_DIGITAL_FILTER_14,       /**< Filter capability up to 14 tI2CCLK   */
    I2C_DIGITAL_FILTER_15,       /**< Filter capability up to 15 tI2CCLK   */
    I2C_DIGITAL_FILTER_CNT       /**< Count of options                     */
}   i2c_DigitalFilter_t;


/** \brief Master addressing mode (applied to every transfer) */
typedef enum
{
    I2C_ADDR_MODE_7BIT = 0u, /**< 7-bit slave address  */
    I2C_ADDR_MODE_10BIT,     /**< 10-bit slave address */
    I2C_ADDR_MODE_CNT        /**< Count of options     */
}   i2c_AddrMode_t;


/** \brief Pull resistor of SCL / SDA pins (external pull-up resistors are recommended) */
typedef enum
{
    I2C_PIN_PULL_NONE = 0u, /**< No internal pull resistor (external pull-up used)               */
    I2C_PIN_PULL_UP,        /**< Internal pull-up (~40 kOhm - sufficient only for short / slow bus) */
    I2C_PIN_PULL_CNT        /**< Count of options                                                */
}   i2c_PinPull_t;


/** \brief List of SCL pins available for I2C peripherals (generated from the STM32CubeMX database) */
typedef enum
{
    I2C_SCL_PIN_I2C1_PA13      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_A   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_4  ), /**< I2C1 SCL pin connected to PA13  */
    I2C_SCL_PIN_I2C1_PA15      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_4  ), /**< I2C1 SCL pin connected to PA15  */
    I2C_SCL_PIN_I2C1_PB8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_B   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_4  ), /**< I2C1 SCL pin connected to PB8   */
    I2C_SCL_PIN_I2C2_PA9       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_A   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_4  ), /**< I2C2 SCL pin connected to PA9   */
    I2C_SCL_PIN_I2C2_PC4       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_C   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_4  ), /**< I2C2 SCL pin connected to PC4   */
#if defined(STM32G471xx) || \
    defined(STM32G473xx) || \
    defined(STM32G474xx) || \
    defined(STM32G483xx) || \
    defined(STM32G484xx)
    I2C_SCL_PIN_I2C2_PF6       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_F   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_4  ), /**< I2C2 SCL pin connected to PF6   */
#endif
#if defined(I2C3)
    I2C_SCL_PIN_I2C3_PA8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_2  ), /**< I2C3 SCL pin connected to PA8   */
    I2C_SCL_PIN_I2C3_PC8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_C   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_8  ), /**< I2C3 SCL pin connected to PC8   */
#if defined(STM32G471xx) || \
    defined(STM32G473xx) || \
    defined(STM32G474xx) || \
    defined(STM32G483xx) || \
    defined(STM32G484xx)
    I2C_SCL_PIN_I2C3_PF3       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_F   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_4  ), /**< I2C3 SCL pin connected to PF3   */
#endif
#if defined(STM32G471xx) || \
    defined(STM32G473xx) || \
    defined(STM32G474xx) || \
    defined(STM32G483xx) || \
    defined(STM32G484xx)
    I2C_SCL_PIN_I2C3_PG7       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_G   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_4  ), /**< I2C3 SCL pin connected to PG7   */
#endif
#endif /* I2C3 */
#if defined(I2C4)
    I2C_SCL_PIN_I2C4_PA13      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_A   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_3  ), /**< I2C4 SCL pin connected to PA13  */
    I2C_SCL_PIN_I2C4_PC6       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_C   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_8  ), /**< I2C4 SCL pin connected to PC6   */
    I2C_SCL_PIN_I2C4_PF14      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_F   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_4  ), /**< I2C4 SCL pin connected to PF14  */
    I2C_SCL_PIN_I2C4_PG3       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_G   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_4  ), /**< I2C4 SCL pin connected to PG3   */
#endif /* I2C4 */
    I2C_SCL_PIN_UNUSED         = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )  /**< Pin is not configured by the module */
}   i2c_SclPin_t;


/** \brief List of SDA pins available for I2C peripherals (generated from the STM32CubeMX database) */
typedef enum
{
    I2C_SDA_PIN_I2C1_PA14      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_A   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_4  ), /**< I2C1 SDA pin connected to PA14  */
    I2C_SDA_PIN_I2C1_PB7       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_B   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_4  ), /**< I2C1 SDA pin connected to PB7   */
    I2C_SDA_PIN_I2C1_PB9       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_B   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_4  ), /**< I2C1 SDA pin connected to PB9   */
    I2C_SDA_PIN_I2C2_PA8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_4  ), /**< I2C2 SDA pin connected to PA8   */
    I2C_SDA_PIN_I2C2_PF0       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_F   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_4  ), /**< I2C2 SDA pin connected to PF0   */
#if defined(I2C3)
    I2C_SDA_PIN_I2C3_PB5       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_8  ), /**< I2C3 SDA pin connected to PB5   */
    I2C_SDA_PIN_I2C3_PC9       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_C   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_8  ), /**< I2C3 SDA pin connected to PC9   */
    I2C_SDA_PIN_I2C3_PC11      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_C   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_8  ), /**< I2C3 SDA pin connected to PC11  */
#if defined(STM32G471xx) || \
    defined(STM32G473xx) || \
    defined(STM32G474xx) || \
    defined(STM32G483xx) || \
    defined(STM32G484xx)
    I2C_SDA_PIN_I2C3_PF4       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_F   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_4  ), /**< I2C3 SDA pin connected to PF4   */
#endif
#if defined(STM32G471xx) || \
    defined(STM32G473xx) || \
    defined(STM32G474xx) || \
    defined(STM32G483xx) || \
    defined(STM32G484xx)
    I2C_SDA_PIN_I2C3_PG8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_G   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_4  ), /**< I2C3 SDA pin connected to PG8   */
#endif
#endif /* I2C3 */
#if defined(I2C4)
    I2C_SDA_PIN_I2C4_PB7       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_B   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_3  ), /**< I2C4 SDA pin connected to PB7   */
    I2C_SDA_PIN_I2C4_PC7       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_C   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_8  ), /**< I2C4 SDA pin connected to PC7   */
    I2C_SDA_PIN_I2C4_PF15      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_F   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_4  ), /**< I2C4 SDA pin connected to PF15  */
    I2C_SDA_PIN_I2C4_PG4       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_G   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_4  ), /**< I2C4 SDA pin connected to PG4   */
#endif /* I2C4 */
    I2C_SDA_PIN_UNUSED         = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )  /**< Pin is not configured by the module */
}   i2c_SdaPin_t;


/** DMA peripherals enumeration list (I2C requests are routed to any channel by DMAMUX1) */
typedef enum
{
    I2C_DMA_PERIPH_1 = DMA_PERIPH_1, /**< DMA peripheral 1 identification */
#if defined(DMA2)
    I2C_DMA_PERIPH_2 = DMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    I2C_DMA_PERIPH_CNT               /**< Count of DMA peripherals        */
}   i2c_DmaPeriphId_t;


/** Enumeration of available channels of DMA peripheral */
typedef enum
{
    I2C_DMA_CHANNEL_1 = DMA_CHANNEL_1, /**< DMA channel 1 */
    I2C_DMA_CHANNEL_2 = DMA_CHANNEL_2, /**< DMA channel 2 */
    I2C_DMA_CHANNEL_3 = DMA_CHANNEL_3, /**< DMA channel 3 */
    I2C_DMA_CHANNEL_4 = DMA_CHANNEL_4, /**< DMA channel 4 */
    I2C_DMA_CHANNEL_5 = DMA_CHANNEL_5, /**< DMA channel 5 */
    I2C_DMA_CHANNEL_6 = DMA_CHANNEL_6, /**< DMA channel 6 */
#if defined(DMA1_Channel7)
    I2C_DMA_CHANNEL_7 = DMA_CHANNEL_7, /**< DMA channel 7 */
#endif
#if defined(DMA1_Channel8)
    I2C_DMA_CHANNEL_8 = DMA_CHANNEL_8, /**< DMA channel 8 */
#endif
    I2C_DMA_CHANNEL_CNT                /**< Count of DMA channels */
}   i2c_DmaChannelId_t;


/** Channel priority options enumeration */
typedef enum
{
    I2C_DMA_PRIORITY_LOW      = DMA_PRIORITY_LOW     , /**< Priority level : Low       */
    I2C_DMA_PRIORITY_MEDIUM   = DMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    I2C_DMA_PRIORITY_HIGH     = DMA_PRIORITY_HIGH    , /**< Priority level : High      */
    I2C_DMA_PRIORITY_VERYHIGH = DMA_PRIORITY_VERYHIGH, /**< Priority level : Very_High */
}   i2c_DmaPriority_t;


/* -------------------------------------------------------------------------- */
/* ---------------------- Data handling configuration ----------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief List of data transfer modes
 *
 * All modes use the same transfer request and report the same events through the same
 * callbacks - they differ only in the context moving the data:
 * - DMA:  DMA channels, requests routed by DMAMUX (bus events and errors from I2C interrupts)
 * - ISR:  I2C interrupt service routines (event + error interrupt)
 * - POLL: I2c_Task() polling I2C flags (callbacks from I2c_Task() context)
 */
typedef enum
{
    I2C_XFER_MODE_NONE = 0u, /**< Data transfers are not used                                 */
    I2C_XFER_MODE_DMA,       /**< Data are transferred by DMA                                 */
    I2C_XFER_MODE_ISR,       /**< Data are transferred by I2C interrupt service routines      */
    I2C_XFER_MODE_POLL,      /**< Data are transferred by I2c_Task() (polling of I2C flags)   */
    I2C_XFER_MODE_CNT        /**< Count of data transfer modes                                */
}   i2c_XferMode_t;


/** \brief List of data transfer errors reported through \ref i2c_XferErrCallback_t */
typedef enum
{
    I2C_XFER_ERROR_NONE = 0u,           /**< No error (last transfer finished successfully)         */
    I2C_XFER_ERROR_NACK,                /**< Slave did not acknowledge address or data byte (NACKF) */
    I2C_XFER_ERROR_BUS,                 /**< Transfer sequencing failed (BERR is not reported on STM32G4 - spurious in master mode, device errata) */
    I2C_XFER_ERROR_ARBITRATION_LOST,    /**< Arbitration lost to another master (ARLO)              */
    I2C_XFER_ERROR_DMA_TRANSFER,        /**< DMA transfer error (bus error during transfer)         */
    I2C_XFER_ERROR_DMA_CONFIG,          /**< DMA configuration error - not reported by STM32G4      */
    I2C_XFER_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration update error - not reported by STM32G4 */
    I2C_XFER_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun - not reported by STM32G4          */
    I2C_XFER_ERROR_CNT                  /**< Count of data transfer errors                          */
}   i2c_XferErrorId_t;


/** \brief Transfer complete callback (STOP condition sent after the last byte) */
typedef void ( i2c_XferCallback_t )( void );

/** \brief Data transfer error callback, error identification is given as parameter */
typedef void ( i2c_XferErrCallback_t )( i2c_XferErrorId_t errorId );


/**
 * \brief Data handling configuration (common for DMA, ISR and POLL mode)
 *
 * Callback events (equal in all modes):
 * - XferCompleteCallback: all bytes of the request were transferred and STOP condition was sent
 * - ErrorCallback:        transfer was terminated by an error (\ref i2c_XferErrorId_t)
 *
 * Unused callback shall be set to I2C_NULL_PTR. DMA identifications / priorities are used only
 * in I2C_XFER_MODE_DMA (transmit and receive channel must differ), IrqPriority is used in DMA
 * and ISR mode (event and error interrupt).
 */
typedef struct
{
    i2c_XferMode_t          XferMode;             /**< Data transfer mode (NONE / DMA / ISR / POLL)     */
    i2c_DmaPeriphId_t       TxDmaPeriphId;        /**< DMA peripheral (transmission in DMA mode)        */
    i2c_DmaChannelId_t      TxDmaChannelId;       /**< DMA channel (transmission in DMA mode)           */
    i2c_DmaPriority_t       TxDmaPriority;        /**< DMA channel priority (transmission in DMA mode)  */
    i2c_DmaPeriphId_t       RxDmaPeriphId;        /**< DMA peripheral (reception in DMA mode)           */
    i2c_DmaChannelId_t      RxDmaChannelId;       /**< DMA channel (reception in DMA mode)              */
    i2c_DmaPriority_t       RxDmaPriority;        /**< DMA channel priority (reception in DMA mode)     */
    i2c_IrqPrio_t           IrqPriority;          /**< I2C interrupts priority (DMA / ISR mode)         */
    i2c_XferCallback_t     *XferCompleteCallback; /**< Transfer complete. I2C_NULL_PTR if not used.     */
    i2c_XferErrCallback_t  *ErrorCallback;        /**< Transfer error. I2C_NULL_PTR if not used.        */
}   i2c_DataConfig_t;


/**
 * \brief Master transfer request
 *
 * - TxSize > 0, RxSize = 0: write TxSize bytes
 * - TxSize = 0, RxSize > 0: read RxSize bytes
 * - TxSize > 0, RxSize > 0: write TxSize bytes, repeated START, read RxSize bytes
 *                           (e.g. register address followed by register data)
 * - TxSize = 0, RxSize = 0: address only (slave presence check, NACK error if not present)
 *
 * \note  Buffers are not copied - they must stay valid until the end of the transfer.
 */
typedef struct
{
    i2c_SlaveAddr_t   SlaveAddr; /**< Slave address (not shifted, see \ref i2c_AddrMode_t)     */
    const i2c_Data_t *TxData;    /**< Data to be written. May be I2C_NULL_PTR if TxSize = 0    */
    i2c_DataCnt_t     TxSize;    /**< Count of bytes to be written                             */
    i2c_Data_t       *RxData;    /**< Buffer for read data. May be I2C_NULL_PTR if RxSize = 0  */
    i2c_DataCnt_t     RxSize;    /**< Count of bytes to be read                                */
}   i2c_XferRequest_t;


/** \brief I2C peripheral configuration structure */
typedef struct
{
    i2c_PeriphId_t           PeriphId;      /**< I2C peripheral identification                                  */
    i2c_ClkSrc_t             ClkSrc;        /**< I2C kernel clock source                                        */
    i2c_FreqHz_t             BusFreq;       /**< Required SCL frequency in Hz (1 - 1 MHz). Mode is derived:
                                                 up to 100 kHz Standard, 400 kHz Fast, 1 MHz Fast-mode Plus    */
    i2c_AnalogFilter_t       AnalogFilter;  /**< Analog noise filter state                                      */
    i2c_DigitalFilter_t      DigitalFilter; /**< Digital noise filter length                                    */
    i2c_AddrMode_t           AddrMode;      /**< Master addressing mode                                         */

    const i2c_DataConfig_t  *DataConfig;    /**< Data handling configuration (copied). I2C_NULL_PTR - data
                                                 handling is not initialized                                   */

    i2c_SclPin_t             SclPin;        /**< SCL pin of PeriphId (I2C_SCL_PIN_UNUSED - not configured)      */
    i2c_SdaPin_t             SdaPin;        /**< SDA pin of PeriphId (I2C_SDA_PIN_UNUSED - not configured)      */
    i2c_PinPull_t            PinPull;       /**< Pull resistor of SCL and SDA pins                              */
}   i2c_Config_t;

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* I2C_I2C_TYPES_H */
