# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32L4 / STM32L4+ MCUs in **master mode**.  
It supports initialization, SCL frequency configuration with automatic TIMINGR calculation, noise filters, and data transfers in DMA, interrupt or polling mode.

---

## Features

- Kernel clock source selection (PCLK1 / SYSCLK / HSI16)
- SCL frequency in Hz - TIMINGR is calculated from I2C specification timings (Standard-mode, Fast-mode, Fast-mode Plus incl. FMP drive)
- Analog and digital noise filter
- 7-bit and 10-bit addressing
- Transfers: write, read, write + repeated START + read (register access), address only (device presence check)
- Transfers longer than 255 bytes (NBYTES reload)
- Data transfer modes: DMA (DMA1 / DMA2), ISR, POLL (`I2c_Task()`) - same request, same callbacks
- SCL / SDA pins configured as open-drain alternate function

Not supported: slave mode, SMBus, bus recovery (slave holding SDA low), transfer timeout.

---

## STM32L4 / STM32L4+ specifics

The public interface is the same as on STM32H5 (`I2c_Port.h` identical, STM32G4 implementation as base). Differences of the STM32L4 implementation:

| Feature                   | STM32L4 / STM32L4+ behavior                                                                                  |
|---------------------------|--------------------------------------------------------------------------------------------------------------|
| I2C peripherals           | I2C1 / I2C3 (all lines), I2C2 (not STM32L432 / L442), I2C4 (STM32L45x / L46x, L49x / L4Ax, STM32L4+)          |
| Kernel clock source       | `I2C_CLK_SRC_PCLK` (APB1), `I2C_CLK_SRC_SYSCLK`, `I2C_CLK_SRC_HSI` (HSI16); re-initialization with another source releases the active one in RCC first (STM32H5 bug AB#1098) |
| Fast-mode Plus drive      | `SYSCFG_CFGR1.I2Cx_FMP` (SYSCFG clock is enabled only when the bit has to change); the bit is cleared by `I2c_Deinit()` - SYSCFG is not reset with the I2C |
| Kernel clock limits       | I2CCLK >= 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus, APB1 / I2CCLK ratio between 1.5 and 3 is refused (errata workarounds of the STM32G4 module, see below) |
| Pins                      | Generated from the STM32CubeMX GPIO modes database - every pin item is active on exactly the CMSIS device lines whose package or die has the pin (guards by the device line) |
| DMA                       | `TxDma` / `RxDma` select the DMA channel from the lists `i2c_TxDma_t` / `i2c_RxDma_t` - one item per I2C peripheral, DMA peripheral and channel, named `I2C_TX_DMA_I2Cx_DMAy_CHANNELz` / `I2C_RX_DMA_I2Cx_DMAy_CHANNELz` (e.g. `I2C_TX_DMA_I2C1_DMA1_CHANNEL6`); transmit and receive channel must differ. STM32L4: fixed request mapping (DMA_CSELR, the request selection is part of the item) - I2C1 TX / RX DMA1 channel 6 / 7 or DMA2 channel 7 / 6, I2C2 TX / RX DMA1 channel 4 / 5, I2C3 TX / RX DMA1 channel 2 / 3, I2C4 TX / RX DMA2 channel 2 / 1; the lists contain these channels only. STM32L4+: any DMA1 / DMA2 channel through DMAMUX1, the lists contain all of them. Items of another I2C peripheral and `I2C_TX_DMA_UNUSED` / `I2C_RX_DMA_UNUSED` are refused in the DMA mode. Only the transfer error interrupt of the channels is used |
| Interrupt lines           | Event (`I2Cx_EV`) and error (`I2Cx_ER`) line per peripheral, one handler (I2C4 lines in STM32L4+ order on STM32L4+) |
| Errors                    | NACK, arbitration lost, DMA transfer error (also STOP before DMA moved all bytes); bus error (BERR) is not reported - see errata |

### Device errata

The workarounds of the STM32G4 module (same I2C IP, STM32G4 errata sheets ES0430 / ES0431 / ES0523) are kept - STM32L4 / STM32L4+ errata sheets are not reviewed yet:

| STM32G4 erratum | Handling |
|-----------------|----------|
| ES0430 2.15.1 - wrong data sampling when data setup time (tSU;DAT) is shorter than one I2C kernel clock period | `I2c_Set_BusFreq()` / `I2c_Init()` refuse a kernel clock below 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus |
| ES0430 2.15.2 - spurious bus error detection in master mode | BERR flag is only cleared, the transfer continues (`I2C_XFER_ERROR_BUS` is reported only when the transfer sequencing fails) |
| ES0430 2.15.5 - transmission stalled after first byte transfer | `I2c_Set_BusFreq()` / `I2c_Init()` refuse an APB1 / I2CCLK ratio between 1.5 and 3 |
| Cortex-M4 r0p1 erratum 838869 | every I2C interrupt handler ends with DSB |

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

SCL / SDA pins are selected from `i2c_SclPin_t` / `i2c_SdaPin_t` - only pins available on the selected device line are defined. The pin must belong to `PeriphId`, otherwise `I2c_Init()` returns error.

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_I2c.c` - I2C and SYSCFG registers emulated, RCC / NVIC / GPIO / DMA mocked; run on several presets (I2C2 / I2C4 availability differs).
- Integration tests (target, boards named by the MCU): `Tests/IntegrationTests/ItTest_I2c.c` - master I2C1 (module under test), slave I2C3 emulating a register device by LL in the test set. Wiring (without it the slave tests are ignored):
  - Nucleo-64 / Nucleo-144: PB8 (SCL, D15) - PC0, PB9 (SDA, D14) - PC1
  - Nucleo-32 (NUCLEO-L412KB / L432KC): PA9 (D1) - PA7 (A6), PA10 (D0) - PB4 (D12)
  - Nucleo-64-P sharing the identification with a Nucleo-32 board (NUCLEO-L412RB-P / L433RC-P, PA7 = SMPS switch): PA9 - PC0, PA10 - PC1 (slave pins selected by the package at run time)

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
