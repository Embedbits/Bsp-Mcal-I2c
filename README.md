# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32H7 MCUs in **master mode**.  
It supports initialization, SCL frequency configuration with automatic TIMINGR calculation, noise filters, and data transfers in DMA, interrupt or polling mode.

---

## Features

- Kernel clock source selection (PCLK / PLL3R / HSI / CSI)
- SCL frequency in Hz - TIMINGR is calculated from I2C specification timings (Standard-mode, Fast-mode, Fast-mode Plus incl. FMP drive)
- Analog and digital noise filter
- 7-bit and 10-bit addressing
- Transfers: write, read, write + repeated START + read (register access), address only (device presence check)
- Transfers longer than 255 bytes (NBYTES reload)
- Data transfer modes: DMA (DMA1 / DMA2 streams with DMAMUX1 requests), ISR, POLL (`I2c_Task()`) - same request, same callbacks
- SCL / SDA pins configured as open-drain alternate function

Not supported: slave mode, SMBus, bus recovery (slave holding SDA low), transfer timeout.

---

## STM32H7 specifics

The public interface is the same as on STM32H5. The STM32H7 implementation is based on the STM32G4 module (same I2C v2 IP, DMAMUX1 request routing) with the 10-bit address NACK workaround of the STM32F7 module (AB#1147). Differences:

| Feature                   | STM32H7 behavior                                                                                             |
|---------------------------|--------------------------------------------------------------------------------------------------------------|
| I2C peripherals           | I2C1 - I2C3 (all lines), I2C4 (D3 / SmartRun domain, all lines), I2C5 (STM32H72x / H73x)                     |
| Kernel clock source       | `I2C_CLK_SRC_PCLK` (PCLK1, PCLK4 for I2C4), `I2C_CLK_SRC_PLLR` (PLL3R), `I2C_CLK_SRC_HSI` (HSI / HSIDIV), `I2C_CLK_SRC_CSI` (4 MHz - Standard-mode only); re-initialization with another source releases the active one in RCC first (STM32H5 bug AB#1098) |
| Fast-mode Plus drive      | `SYSCFG_PMCR.I2Cx_FMP` (SYSCFG clock is enabled only when the bit has to change); the bit is cleared by `I2c_Deinit()` - SYSCFG is not reset with the I2C |
| Kernel clock limits       | I2CCLK >= 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus, APB / I2CCLK ratio between 1.5 and 3 is refused (device errata, see below) |
| DMA                       | DMA1 / DMA2 stream 0 - 7 selected by `TxDmaPeriphId` / `TxDmaChannelId` and `RxDmaPeriphId` / `RxDmaChannelId`, request routed by DMAMUX1 (`DMA_REQ_I2Cx_TX` / `DMA_REQ_I2Cx_RX`); only the transfer error interrupt of the streams is used. I2C4 requests are routed by DMAMUX2 to BDMA (not supported by the Dma module) - DMA mode of I2C4 is refused. Buffers shall be accessible by DMA1 / DMA2 (not DTCM / ITCM) |
| Interrupt lines           | Event (`I2Cx_EV`) and error (`I2Cx_ER`) line per peripheral, one handler (no DSB - Cortex-M7)               |
| Errors                    | NACK, arbitration lost, DMA transfer error (also STOP before DMA moved all bytes); bus error (BERR) is not reported - see errata |

**STM32H7R3 / H7R7 / H7S3 / H7S7** (Ral family STM32H7RS, macro `STM32H7RS`): I2C1 - I2C3, no DMA1 / DMA2 streams -
DMA mode uses GPDMA1 / HPDMA1 channels (`Gpdma_Lib`, `I2c_Dma.c` of the STM32H5 module compiled under `STM32H7RS`,
GPDMA1 requests `GPDMA_REQ_I2Cx_RX / _TX`), Fast-mode Plus 20 mA drive by `I2C_CR1.FMP` (no SYSCFG_PMCR bits),
kernel clock sources as on the classic lines (I2C1 shares its mux with I3C1), SCL / SDA pin enumerations from the
STM32H7RS open pin data. Unit tests: STM32H7 tests on STM32H7R / H7S (DMA stream tests and the SYSCFG Fast-mode Plus
test ignored there, Fast-mode Plus tests check `I2C_CR1.FMP`) - the STM32H5 module has no unit tests of the GPDMA data
handling, the DMA mode is verified by the integration tests on NUCLEO-H7S3L8 with the IT phase.

### Device errata

STM32H7 errata sheets (ES0392 / ES0445 / ES0478) are not available offline - the I2C errata of the same I2C v2 IP (STM32F7 ES0334, STM32G4 ES0430) are handled and will be confirmed by the errata audit AB#844.

| Erratum | Handling |
|---------|----------|
| Wrong data sampling when data setup time (tSU;DAT) is shorter than one I2C kernel clock period | `I2c_Set_BusFreq()` / `I2c_Init()` refuse a kernel clock below 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus |
| Spurious bus error detection in master mode | BERR flag is only cleared, the transfer continues (`I2C_XFER_ERROR_BUS` is reported only when the transfer sequencing fails) |
| Transmission stalled after first byte transfer | `I2c_Set_BusFreq()` / `I2c_Init()` refuse an APB / I2CCLK ratio between 1.5 and 3 |
| 10-bit master mode: new transfer cannot be launched if first part of the address is not acknowledged by the slave | After STOPF of a NACKed 10-bit transfer the peripheral is disabled until CR2.START is released and enabled again |

Not applicable (master mode only): spurious master transfer upon own slave address match, OVR flag not set in underrun condition (slave), SDA held low upon SMBus timeout expiry (SMBus slave).

Not yet tested on hardware (integration tests prepared for STM32H7 Nucleo boards).

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

i2cConfig.PeriphId   = I2C_PERIPH_3;
i2cConfig.BusFreq    = 400000u;
i2cConfig.DataConfig = &i2cData;
i2cConfig.SclPin     = I2C_SCL_PIN_I2C3_PC8;
i2cConfig.SdaPin     = I2C_SDA_PIN_I2C3_PC9;

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

(void)I2c_Set_XferStart( I2C_PERIPH_3, &request );
```

SCL / SDA pins are selected from `i2c_SclPin_t` / `i2c_SdaPin_t` - only pins available on the selected device line (STM32H72x / H73x, H74x / H75x, H7A3 / H7B0 / H7B3) are defined. The pin must belong to `PeriphId`, otherwise `I2c_Init()` returns error. Pin tables were generated from ST open pin data (STM32_open_pin_data).


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
