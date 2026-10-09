# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32U5 MCUs in **master mode**.  
It supports initialization, SCL frequency configuration with automatic TIMINGR calculation, noise filters, and data transfers in DMA, interrupt or polling mode.

---

## Features

- Kernel clock source selection (PCLK / SYSCLK / HSI16 / MSIK), I2C1 - I2C6 (I2C5 / I2C6 on STM32U59x / U5Ax / U5Fx / U5Gx)
- SCL frequency in Hz - TIMINGR is calculated from I2C specification timings (Standard-mode, Fast-mode, Fast-mode Plus incl. FMP drive)
- Analog and digital noise filter
- 7-bit and 10-bit addressing
- Transfers: write, read, write + repeated START + read (register access), address only (device presence check)
- Transfers longer than 255 bytes (NBYTES reload)
- Data transfer modes: DMA (GPDMA), ISR, POLL (`I2c_Task()`) - same request, same callbacks
- SCL / SDA pin lists per device family (open-drain alternate function)

Not supported: slave mode, SMBus, bus recovery (slave holding SDA low), transfer timeout.

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
i2cConfig.SclPin     = I2C_SCL_PIN_I2C1_PB6;
i2cConfig.SdaPin     = I2C_SDA_PIN_I2C1_PB7;

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

SCL / SDA pins are selected from `i2c_SclPin_t` / `i2c_SdaPin_t` - only pins available on the selected device line are defined (preprocessor conditions per STM32U5 line). The pin must belong to `PeriphId`, otherwise `I2c_Init()` returns error. Pin tables were generated from the STM32CubeMX pin database.

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
