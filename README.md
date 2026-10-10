# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32F7 MCUs in **master mode**.  
It supports initialization, SCL frequency configuration with automatic TIMINGR calculation, noise filters, and data transfers in DMA, interrupt or polling mode.

---

## Features

- Kernel clock source selection (PCLK1 / SYSCLK / HSI)
- SCL frequency in Hz - TIMINGR is calculated from I2C specification timings (Standard-mode, Fast-mode, Fast-mode Plus incl. FMP drive)
- Analog and digital noise filter
- 7-bit and 10-bit addressing
- Transfers: write, read, write + repeated START + read (register access), address only (device presence check)
- Transfers longer than 255 bytes (NBYTES reload)
- Data transfer modes: DMA (DMA1 streams), ISR, POLL (`I2c_Task()`) - same request, same callbacks
- SCL / SDA pins configured as open-drain alternate function

Not supported: slave mode, SMBus, bus recovery (slave holding SDA low), transfer timeout.

---

## STM32F7 specifics

The public interface is the same as on STM32H5. Differences of the STM32F7 implementation:

| Feature                   | STM32F7 behavior                                                                                             |
|---------------------------|--------------------------------------------------------------------------------------------------------------|
| I2C peripherals           | I2C1 - I2C3 (all lines), I2C4 (STM32F74x / F75x / F76x / F77x)                                               |
| Kernel clock source       | `I2C_CLK_SRC_PCLK` (APB1), `I2C_CLK_SRC_SYSCLK`, `I2C_CLK_SRC_HSI` (16 MHz) selected in `RCC_DCKCFGR2`; re-initialization with another source releases the active one in RCC first (STM32H5 bug AB#1098) |
| Fast-mode Plus drive      | `SYSCFG_PMC.I2Cx_FMP` (SYSCFG clock is enabled only when the bit has to change); the bit is cleared by `I2c_Deinit()` - SYSCFG is not reset with the I2C |
| Kernel clock limits       | I2CCLK >= 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus, APB1 / I2CCLK ratio between 1.5 and 3 is refused (device errata, see below) |
| DMA                       | DMA1 stream selected by `TxDma` / `RxDma` from the lists `i2c_TxDma_t` / `i2c_RxDma_t` - one item per I2C peripheral and stream, named `I2C_TX_DMA_I2Cx_DMA1_STREAMz` / `I2C_RX_DMA_I2Cx_DMA1_STREAMz` (e.g. `I2C_TX_DMA_I2C1_DMA1_STREAM6`), the channel selection of the stream is part of the item (see below); items of another I2C peripheral, items of the other direction and `I2C_TX_DMA_UNUSED` / `I2C_RX_DMA_UNUSED` are refused in the DMA mode, items of streams existing only on some device lines are guarded by the CMSIS device line; only the transfer error interrupt of the streams is used |
| Interrupt lines           | Event (`I2Cx_EV`) and error (`I2Cx_ER`) line per peripheral, one handler                                     |
| Errors                    | NACK, arbitration lost, DMA transfer error (also STOP before DMA moved all bytes); bus error (BERR) is not reported - see errata |

### DMA1 request mapping

| Request  | Stream / channel (all STM32F7)  | STM32F74x / F75x / F76x / F77x | STM32F76x / F77x only (channel 8) |
|----------|---------------------------------|--------------------------------|-----------------------------------|
| I2C1_RX  | stream 0 / ch 1, stream 5 / ch 1 |                               |                                   |
| I2C1_TX  | stream 6 / ch 1, stream 7 / ch 1 |                               |                                   |
| I2C2_RX  | stream 2 / ch 7, stream 3 / ch 7 |                               |                                   |
| I2C2_TX  | stream 7 / ch 7                 |                                | stream 4 / ch 8                   |
| I2C3_RX  | stream 1 / ch 1, stream 2 / ch 3 |                               |                                   |
| I2C3_TX  | stream 4 / ch 3                 |                                | stream 0 / ch 8                   |
| I2C4_RX  |                                 | stream 2 / ch 2                | stream 1 / ch 8                   |
| I2C4_TX  |                                 | stream 5 / ch 2                | stream 6 / ch 8                   |

Source: STM32CubeMX database (DMA IP of STM32F722 / STM32F777). Channel 8 requires the 4-bit CHSEL of STM32F76x / F77x (`DMA_SxCR_CHSEL_3`).

### Device errata (ES0334)

ES0334 Rev 9 (STM32F76x / F77x), the same I2C IP is used on all STM32F7 lines.

| Erratum | Handling |
|---------|----------|
| Wrong data sampling when data setup time (tSU;DAT) is shorter than one I2C kernel clock period | `I2c_Set_BusFreq()` / `I2c_Init()` refuse a kernel clock below 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus |
| Spurious bus error detection in master mode | BERR flag is only cleared, the transfer continues (`I2C_XFER_ERROR_BUS` is reported only when the transfer sequencing fails) |
| Transmission stalled after first byte transfer | `I2c_Set_BusFreq()` / `I2c_Init()` refuse an APB1 / I2CCLK ratio between 1.5 and 3 |
| 10-bit master mode: new transfer cannot be launched if first part of the address is not acknowledged by the slave | On STOPF of a transfer ended by NACK in 10-bit addressing mode with `CR2.START` still set, the peripheral is disabled until START is released and enabled again (bug AB#1147) |
| Last-received byte loss in reload mode | Not affected - NBYTES of the next chunk is reloaded on TCR without waiting for the last byte of the chunk, the byte is moved to RXDR after the reload and read by the data handler |

Not applicable (master mode only, no low-power handling): spurious master transfer upon own slave address match, OVR flag not set in underrun condition (slave), I2C disabled before Stop mode (description inaccuracy).

---

## Public API

### Module Management
- `i2c_ModuleVersion_t I2c_Get_ModuleVersion(void);`
- `i2c_RequestState_t  I2c_Init(const i2c_Config_t * const i2cConfig);`
- `i2c_RequestState_t  I2c_Deinit(i2c_PeriphId_t periphId);`
- `void                I2c_Task(void);`
- `i2c_RequestState_t  I2c_Get_DefaultConfig(i2c_Config_t * const i2cConfig);`

### Peripheral Configuration
- `I2c_Set_PeriphActive` / `I2c_Set_PeriphInactive` / `I2c_Get_PeriphState`
- `I2c_Set_BusFreq` / `I2c_Get_BusFreq` (PE = 0 required for set)
- `I2c_Set_AnalogFilter` / `I2c_Get_AnalogFilter` (PE = 0 required for set)
- `I2c_Set_DigitalFilter` / `I2c_Get_DigitalFilter` (PE = 0 required for set)
- `I2c_Set_AddrMode` / `I2c_Get_AddrMode`
- `I2c_Get_BusState`

### Data Transfers
- `I2c_Set_DataConfig` / `I2c_Get_DataConfig`
- `I2c_Set_XferStart` / `I2c_Set_XferStop`
- `I2c_Get_XferState` / `I2c_Get_XferError`

### Interrupts
- `I2c_Set_IrqPriority` / `I2c_Get_IrqPriority`

---

## Usage

```c
static const i2c_DataConfig_t i2cData =
{
    .XferMode             = I2C_XFER_MODE_ISR,
    .IrqPriority          = 5u,
    .XferCompleteCallback = App_I2cDone,
    .ErrorCallback        = App_I2cError,
};

i2c_Config_t i2cConfig;

(void)I2c_Get_DefaultConfig( &i2cConfig );

i2cConfig.PeriphId   = I2C_PERIPH_1;
i2cConfig.BusFreq    = 400000u;
i2cConfig.DataConfig = &i2cData;
i2cConfig.SclPin     = I2C_SCL_PIN_I2C1_PB8;
i2cConfig.SdaPin     = I2C_SDA_PIN_I2C1_PB9;

(void)I2c_Init( &i2cConfig );

/* Read 2 bytes from register 0x10 of slave 0x48 */
static const i2c_Data_t regAddr = 0x10u;
static i2c_Data_t       regData[ 2u ];

const i2c_XferRequest_t request =
{
    .SlaveAddr = 0x48u,
    .TxData    = &regAddr, .TxSize = 1u,
    .RxData    = regData,  .RxSize = 2u,
};

(void)I2c_Set_XferStart( I2C_PERIPH_1, &request );
```

SCL / SDA pins are selected from `i2c_SclPin_t` / `i2c_SdaPin_t` - only pins available on the selected device line are defined (e.g. I2C4 on PB6 - PB9 only on STM32F76x / F77x). The pin must belong to `PeriphId`, otherwise `I2c_Init()` returns error. Pin tables were generated from the STM32CubeMX database.

---

## CMake Integration

1. Link `I2c_Lib` to your CMake library.
2. Include `I2c_Port.h` in your project.

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)
