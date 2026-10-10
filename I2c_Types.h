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
 * \note STM32F7 family: configuration structures and type names are common for
 *       all families, enumerations with hardware specific values (peripherals,
 *       clock sources, pins, DMA streams) follow STM32F7 I2C.
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

/** Mask of one field (5 bits) in encoded pin / DMA stream value */
#define I2C_BIT_MASK_FIELD                  ( 0x1Fu )

/** I2C peripheral identification bit offset in encoded DMA stream value */
#define I2C_DMA_BIT_MASK_PERIPH_BIT_OFFSET  ( 15u )

/** DMA peripheral identification bit offset in encoded DMA stream value */
#define I2C_DMA_BIT_MASK_DMA_BIT_OFFSET     ( 10u )

/** Stream identification bit offset in encoded DMA stream value */
#define I2C_DMA_BIT_MASK_STREAM_BIT_OFFSET  ( 5u )

/** Channel selection (CHSEL) bit offset in encoded DMA stream value */
#define I2C_DMA_BIT_MASK_CHSEL_BIT_OFFSET   ( 0u )


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

/**
 * \brief Encodes DMA stream (I2C peripheral, DMA peripheral, stream, channel selection) into single
 *        value of \ref i2c_DmaCode_t
 *
 * The macro defines the values of the DMA stream lists \ref i2c_TxDma_t and \ref i2c_RxDma_t, e.g. the
 * I2C1 transmit request on DMA1 stream 6 (channel selection 1) is \ref I2C_TX_DMA_I2C1_DMA1_STREAM6:
 * I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_6, 1u )
 */
#define I2C_DMA_ENCODE( PERIPH_ID, DMA_ID, STREAM_ID, CHSEL )   ( (i2c_DmaCode_t)( ( (uint32_t)(PERIPH_ID) << I2C_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) | \
                                                                                   ( (uint32_t)(DMA_ID)    << I2C_DMA_BIT_MASK_DMA_BIT_OFFSET    ) | \
                                                                                   ( (uint32_t)(STREAM_ID) << I2C_DMA_BIT_MASK_STREAM_BIT_OFFSET ) | \
                                                                                   ( (uint32_t)(CHSEL)     << I2C_DMA_BIT_MASK_CHSEL_BIT_OFFSET  )   ) )

/** Stream is not configured by the module (value of the *_DMA_UNUSED items of the DMA stream lists) */
#define I2C_DMA_CODE_UNUSED                 I2C_DMA_ENCODE( I2C_PERIPH_CNT, I2C_DMA_PERIPH_CNT, I2C_DMA_CHANNEL_CNT, 0u )

/** Extract I2C peripheral ID from encoded DMA stream value */
#define I2C_DMA_BIT_MASK_DECODE_PERIPH( CODED_VAL ) ( ( (CODED_VAL) >> I2C_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

/** Extract DMA peripheral ID from encoded DMA stream value */
#define I2C_DMA_BIT_MASK_DECODE_DMA( CODED_VAL )    ( ( (CODED_VAL) >> I2C_DMA_BIT_MASK_DMA_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

/** Extract stream ID from encoded DMA stream value */
#define I2C_DMA_BIT_MASK_DECODE_STREAM( CODED_VAL ) ( ( (CODED_VAL) >> I2C_DMA_BIT_MASK_STREAM_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

/** Extract channel selection (CHSEL) from encoded DMA stream value */
#define I2C_DMA_BIT_MASK_DECODE_CHSEL( CODED_VAL )  ( ( (CODED_VAL) >> I2C_DMA_BIT_MASK_CHSEL_BIT_OFFSET ) & I2C_BIT_MASK_FIELD )

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

/** \brief Encoded DMA stream (value of \ref i2c_TxDma_t or \ref i2c_RxDma_t) */
typedef uint32_t i2c_DmaCode_t;


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
    I2C_CLK_SRC_HSI,       /**< 16 MHz High Speed Internal oscillator (HSI) - up to Fast-mode
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


/** \brief List of SCL pins available for I2C peripherals (generated from the STM32CubeMX database, pins of some STM32F7 lines guarded by the CMSIS device line) */
typedef enum
{
    I2C_SCL_PIN_I2C1_PB6       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_B   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_4  ), /**< I2C1 SCL pin connected to PB6   */
    I2C_SCL_PIN_I2C1_PB8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_B   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_4  ), /**< I2C1 SCL pin connected to PB8   */
    I2C_SCL_PIN_I2C2_PB10      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_B   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_4  ), /**< I2C2 SCL pin connected to PB10  */
    I2C_SCL_PIN_I2C2_PF1       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_F   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_4  ), /**< I2C2 SCL pin connected to PF1   */
    I2C_SCL_PIN_I2C2_PH4       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_H   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_4  ), /**< I2C2 SCL pin connected to PH4   */
    I2C_SCL_PIN_I2C3_PA8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_4  ), /**< I2C3 SCL pin connected to PA8   */
    I2C_SCL_PIN_I2C3_PH7       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_H   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_4  ), /**< I2C3 SCL pin connected to PH7   */
#if defined(I2C4)
#if defined(STM32F765xx) || \
    defined(STM32F767xx) || \
    defined(STM32F769xx) || \
    defined(STM32F777xx) || \
    defined(STM32F779xx)
    I2C_SCL_PIN_I2C4_PB6       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_B   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_11 ), /**< I2C4 SCL pin connected to PB6   */
#endif
#if defined(STM32F765xx) || \
    defined(STM32F767xx) || \
    defined(STM32F769xx) || \
    defined(STM32F777xx) || \
    defined(STM32F779xx)
    I2C_SCL_PIN_I2C4_PB8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_B   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_1  ), /**< I2C4 SCL pin connected to PB8   */
#endif
    I2C_SCL_PIN_I2C4_PD12      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_D   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_4  ), /**< I2C4 SCL pin connected to PD12  */
    I2C_SCL_PIN_I2C4_PF14      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_F   , GPIO_PIN_ID_14 , GPIO_ALT_FUNC_4  ), /**< I2C4 SCL pin connected to PF14  */
    I2C_SCL_PIN_I2C4_PH11      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_H   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_4  ), /**< I2C4 SCL pin connected to PH11  */
#endif /* I2C4 */
    I2C_SCL_PIN_UNUSED         = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )  /**< Pin is not configured by the module */
}   i2c_SclPin_t;


/** \brief List of SDA pins available for I2C peripherals (generated from the STM32CubeMX database) */
typedef enum
{
    I2C_SDA_PIN_I2C1_PB7       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_B   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_4  ), /**< I2C1 SDA pin connected to PB7   */
    I2C_SDA_PIN_I2C1_PB9       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_1   , GPIO_PORT_B   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_4  ), /**< I2C1 SDA pin connected to PB9   */
    I2C_SDA_PIN_I2C2_PB11      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_B   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_4  ), /**< I2C2 SDA pin connected to PB11  */
    I2C_SDA_PIN_I2C2_PF0       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_F   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_4  ), /**< I2C2 SDA pin connected to PF0   */
    I2C_SDA_PIN_I2C2_PH5       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_2   , GPIO_PORT_H   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_4  ), /**< I2C2 SDA pin connected to PH5   */
    I2C_SDA_PIN_I2C3_PC9       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_C   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_4  ), /**< I2C3 SDA pin connected to PC9   */
    I2C_SDA_PIN_I2C3_PH8       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_3   , GPIO_PORT_H   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_4  ), /**< I2C3 SDA pin connected to PH8   */
#if defined(I2C4)
#if defined(STM32F765xx) || \
    defined(STM32F767xx) || \
    defined(STM32F769xx) || \
    defined(STM32F777xx) || \
    defined(STM32F779xx)
    I2C_SDA_PIN_I2C4_PB7       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_B   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_11 ), /**< I2C4 SDA pin connected to PB7   */
#endif
#if defined(STM32F765xx) || \
    defined(STM32F767xx) || \
    defined(STM32F769xx) || \
    defined(STM32F777xx) || \
    defined(STM32F779xx)
    I2C_SDA_PIN_I2C4_PB9       = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_B   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_1  ), /**< I2C4 SDA pin connected to PB9   */
#endif
    I2C_SDA_PIN_I2C4_PD13      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_D   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_4  ), /**< I2C4 SDA pin connected to PD13  */
    I2C_SDA_PIN_I2C4_PF15      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_F   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_4  ), /**< I2C4 SDA pin connected to PF15  */
    I2C_SDA_PIN_I2C4_PH12      = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_4   , GPIO_PORT_H   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_4  ), /**< I2C4 SDA pin connected to PH12  */
#endif /* I2C4 */
    I2C_SDA_PIN_UNUSED         = I2C_PIN_BIT_MASK_ENCODE( I2C_PERIPH_CNT, GPIO_PORT_CNT, GPIO_PIN_ID_CNT, GPIO_ALT_FUNC_CNT )  /**< Pin is not configured by the module */
}   i2c_SdaPin_t;


/** DMA peripherals enumeration list (I2C requests are connected to DMA1 only) */
typedef enum
{
    I2C_DMA_PERIPH_1 = DMA_PERIPH_1, /**< DMA peripheral 1 identification */
    I2C_DMA_PERIPH_CNT
}   i2c_DmaPeriphId_t;


/**
 * \brief Enumeration of DMA streams (channel of the interface = stream of STM32F7 DMA)
 *
 * The stream must be connected to the I2C request of the transfer direction (RM, DMA1 request
 * mapping), e.g. I2C1 RX streams 0 / 5, I2C1 TX streams 6 / 7. The streams usable by the I2C
 * peripherals are given by the lists \ref i2c_TxDma_t and \ref i2c_RxDma_t.
 */
typedef enum
{
    I2C_DMA_CHANNEL_0  = DMA_STREAM_0,  /**< DMA stream 0                   */
    I2C_DMA_CHANNEL_1  = DMA_STREAM_1,  /**< DMA stream 1                   */
    I2C_DMA_CHANNEL_2  = DMA_STREAM_2,  /**< DMA stream 2                   */
    I2C_DMA_CHANNEL_3  = DMA_STREAM_3,  /**< DMA stream 3                   */
    I2C_DMA_CHANNEL_4  = DMA_STREAM_4,  /**< DMA stream 4                   */
    I2C_DMA_CHANNEL_5  = DMA_STREAM_5,  /**< DMA stream 5                   */
    I2C_DMA_CHANNEL_6  = DMA_STREAM_6,  /**< DMA stream 6                   */
    I2C_DMA_CHANNEL_7  = DMA_STREAM_7,  /**< DMA stream 7                   */
    I2C_DMA_CHANNEL_CNT                 /**< Count of available DMA streams */
}   i2c_DmaChannelId_t;


/**
 * \brief List of DMA streams able to serve the I2C TX request of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the channel selection of the stream is part of the value, streams
 *        existing only on some STM32F7 lines are guarded by the CMSIS device line)
 */
typedef enum
{
    I2C_TX_DMA_I2C1_DMA1_STREAM6       = I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_6, 1u ), /**< I2C1 TX request on DMA1 stream 6 (channel selection 1) */
    I2C_TX_DMA_I2C1_DMA1_STREAM7       = I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_7, 1u ), /**< I2C1 TX request on DMA1 stream 7 (channel selection 1) */
#if defined(STM32F765xx) || \
    defined(STM32F767xx) || \
    defined(STM32F769xx) || \
    defined(STM32F777xx) || \
    defined(STM32F779xx)
    I2C_TX_DMA_I2C2_DMA1_STREAM4       = I2C_DMA_ENCODE( I2C_PERIPH_2, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_4, 8u ), /**< I2C2 TX request on DMA1 stream 4 (channel selection 8) */
#endif
    I2C_TX_DMA_I2C2_DMA1_STREAM7       = I2C_DMA_ENCODE( I2C_PERIPH_2, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_7, 7u ), /**< I2C2 TX request on DMA1 stream 7 (channel selection 7) */
#if defined(STM32F765xx) || \
    defined(STM32F767xx) || \
    defined(STM32F769xx) || \
    defined(STM32F777xx) || \
    defined(STM32F779xx)
    I2C_TX_DMA_I2C3_DMA1_STREAM0       = I2C_DMA_ENCODE( I2C_PERIPH_3, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_0, 8u ), /**< I2C3 TX request on DMA1 stream 0 (channel selection 8) */
#endif
    I2C_TX_DMA_I2C3_DMA1_STREAM4       = I2C_DMA_ENCODE( I2C_PERIPH_3, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_4, 3u ), /**< I2C3 TX request on DMA1 stream 4 (channel selection 3) */
#if defined(I2C4)
    I2C_TX_DMA_I2C4_DMA1_STREAM5       = I2C_DMA_ENCODE( I2C_PERIPH_4, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_5, 2u ), /**< I2C4 TX request on DMA1 stream 5 (channel selection 2) */
#endif
#if defined(I2C4) && \
    !defined(STM32F745xx) && \
    !defined(STM32F746xx) && \
    !defined(STM32F750xx) && \
    !defined(STM32F756xx)
    I2C_TX_DMA_I2C4_DMA1_STREAM6       = I2C_DMA_ENCODE( I2C_PERIPH_4, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_6, 8u ), /**< I2C4 TX request on DMA1 stream 6 (channel selection 8) */
#endif
    I2C_TX_DMA_UNUSED                  = I2C_DMA_CODE_UNUSED  /**< DMA stream is not selected */
}   i2c_TxDma_t;


/**
 * \brief List of DMA streams able to serve the I2C RX request of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the channel selection of the stream is part of the value, streams
 *        existing only on some STM32F7 lines are guarded by the CMSIS device line)
 */
typedef enum
{
    I2C_RX_DMA_I2C1_DMA1_STREAM0       = I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_0, 1u ), /**< I2C1 RX request on DMA1 stream 0 (channel selection 1) */
    I2C_RX_DMA_I2C1_DMA1_STREAM5       = I2C_DMA_ENCODE( I2C_PERIPH_1, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_5, 1u ), /**< I2C1 RX request on DMA1 stream 5 (channel selection 1) */
    I2C_RX_DMA_I2C2_DMA1_STREAM2       = I2C_DMA_ENCODE( I2C_PERIPH_2, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_2, 7u ), /**< I2C2 RX request on DMA1 stream 2 (channel selection 7) */
    I2C_RX_DMA_I2C2_DMA1_STREAM3       = I2C_DMA_ENCODE( I2C_PERIPH_2, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_3, 7u ), /**< I2C2 RX request on DMA1 stream 3 (channel selection 7) */
    I2C_RX_DMA_I2C3_DMA1_STREAM1       = I2C_DMA_ENCODE( I2C_PERIPH_3, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_1, 1u ), /**< I2C3 RX request on DMA1 stream 1 (channel selection 1) */
    I2C_RX_DMA_I2C3_DMA1_STREAM2       = I2C_DMA_ENCODE( I2C_PERIPH_3, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_2, 3u ), /**< I2C3 RX request on DMA1 stream 2 (channel selection 3) */
#if defined(I2C4) && \
    !defined(STM32F745xx) && \
    !defined(STM32F746xx) && \
    !defined(STM32F750xx) && \
    !defined(STM32F756xx)
    I2C_RX_DMA_I2C4_DMA1_STREAM1       = I2C_DMA_ENCODE( I2C_PERIPH_4, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_1, 8u ), /**< I2C4 RX request on DMA1 stream 1 (channel selection 8) */
#endif
#if defined(I2C4)
    I2C_RX_DMA_I2C4_DMA1_STREAM2       = I2C_DMA_ENCODE( I2C_PERIPH_4, I2C_DMA_PERIPH_1, I2C_DMA_CHANNEL_2, 2u ), /**< I2C4 RX request on DMA1 stream 2 (channel selection 2) */
#endif
    I2C_RX_DMA_UNUSED                  = I2C_DMA_CODE_UNUSED  /**< DMA stream is not selected */
}   i2c_RxDma_t;


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
 * - DMA:  DMA1 streams of the I2C requests (bus events and errors from I2C interrupts)
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
    I2C_XFER_ERROR_BUS,                 /**< Transfer sequencing failed (BERR is not reported on STM32F7 - spurious in master mode, device errata) */
    I2C_XFER_ERROR_ARBITRATION_LOST,    /**< Arbitration lost to another master (ARLO)              */
    I2C_XFER_ERROR_DMA_TRANSFER,        /**< DMA transfer error (bus error during transfer)         */
    I2C_XFER_ERROR_DMA_CONFIG,          /**< DMA configuration error - not reported by STM32F7      */
    I2C_XFER_ERROR_DMA_CONFIG_UPDATE,   /**< DMA configuration update error - not reported by STM32F7 */
    I2C_XFER_ERROR_DMA_TRIGGER_OVERRUN, /**< DMA trigger overrun - not reported by STM32F7          */
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
 * Unused callback shall be set to I2C_NULL_PTR. DMA streams / priorities are used only in
 * I2C_XFER_MODE_DMA (TxDma / RxDma are items of the lists \ref i2c_TxDma_t / \ref i2c_RxDma_t of the
 * configured I2C peripheral, I2C_TX_DMA_UNUSED / I2C_RX_DMA_UNUSED in other modes), IrqPriority is
 * used in DMA and ISR mode (event and error interrupt).
 */
typedef struct
{
    i2c_XferMode_t          XferMode;             /**< Data transfer mode (NONE / DMA / ISR / POLL)     */
    i2c_TxDma_t             TxDma;                /**< DMA stream (transmission in DMA mode)            */
    i2c_DmaPriority_t       TxDmaPriority;        /**< DMA stream priority (transmission in DMA mode)   */
    i2c_RxDma_t             RxDma;                /**< DMA stream (reception in DMA mode)               */
    i2c_DmaPriority_t       RxDmaPriority;        /**< DMA stream priority (reception in DMA mode)      */
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
