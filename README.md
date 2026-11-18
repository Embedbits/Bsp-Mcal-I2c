# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32H7 MCUs in **master or slave mode**.  
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
- Slave mode (`Role = I2C_ROLE_SLAVE`): own address 1 (7-bit or 10-bit), own address 2 with mask,
  general call, clock stretching enabled / disabled, fill byte for a master read beyond the buffer;
  listen with buffers of the application, address match callback (direction, matched address - the
  buffers can be replaced there, eg. register device: register address written, repeated START,
  register data read), transfer complete callback with the result (received / transmitted byte
  count, bytes beyond the buffers); DMA / ISR / POLL as for the master

Not supported: SMBus, wake-up from Stop mode by address match, bus recovery (slave holding SDA low),
transfer timeout.

---

## STM32H7 specifics

The public interface is the same as on STM32H5. The STM32H7 implementation is based on the STM32G4 module (same I2C v2 IP, DMAMUX1 request routing) with the 10-bit address NACK workaround of the STM32F7 module (AB#1147). Differences:

| Feature                   | STM32H7 behavior                                                                                             |
|---------------------------|--------------------------------------------------------------------------------------------------------------|
| I2C peripherals           | I2C1 - I2C3 (all lines), I2C4 (D3 / SmartRun domain, all lines), I2C5 (STM32H72x / H73x)                     |
| Kernel clock source       | `I2C_CLK_SRC_PCLK` (PCLK1, PCLK4 for I2C4), `I2C_CLK_SRC_PLLR` (PLL3R), `I2C_CLK_SRC_HSI` (HSI / HSIDIV), `I2C_CLK_SRC_CSI` (4 MHz - Standard-mode only); re-initialization with another source releases the active one in RCC first (STM32H5 bug AB#1098) |
| Fast-mode Plus drive      | `SYSCFG_PMCR.I2Cx_FMP` (SYSCFG clock is enabled only when the bit has to change); the bit is cleared by `I2c_Deinit()` - SYSCFG is not reset with the I2C |
| Kernel clock limits       | I2CCLK >= 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus, APB / I2CCLK ratio between 1.5 and 3 is refused (device errata, see below) |
| DMA                       | DMA1 / DMA2 stream 0 - 7 selected by `TxDmaPeriphId` / `TxDmaChannelId` and `RxDmaPeriphId` / `RxDmaChannelId`, request routed by DMAMUX1 (`DMA_REQ_I2Cx_TX` / `DMA_REQ_I2Cx_RX`); the master uses only the transfer error interrupt of the streams, the slave also the transfer complete interrupt (a full buffer switches the phase to the I2C interrupt). I2C4 requests are routed by DMAMUX2 to BDMA (not supported by the Dma module) - DMA mode of I2C4 is refused. Buffers shall be accessible by DMA1 / DMA2 (not DTCM / ITCM) |
| Slave (listen)            | Same sequencing as on STM32H5 (`I2c_Set_SlaveEvents()`), own addresses written by `I2c_Init()` and switched on by `I2c_Set_ListenStart()`. DMA mode: the stream of the direction of the address match is armed with the buffer of the slave; the DMA streams work in direct mode (no FIFO), so the transmitted count is the block size minus the remaining data count of the stream. I2C4 (no DMA mode) listens in ISR / POLL mode. Clock stretching enabled is recommended (see errata below) |
| Interrupt lines           | Event (`I2Cx_EV`) and error (`I2Cx_ER`) line per peripheral, one handler (no DSB - Cortex-M7)               |
| Errors                    | NACK, arbitration lost, DMA transfer error (also STOP before DMA moved all bytes); bus error (BERR) is not reported - see errata |

**STM32H7R3 / H7R7 / H7S3 / H7S7** (Ral family STM32H7RS, macro `STM32H7RS`): I2C1 - I2C3, no DMA1 / DMA2 streams -
DMA mode uses GPDMA1 / HPDMA1 channels (`Gpdma_Lib`, `I2c_Dma.c` of the STM32H5 module compiled under `STM32H7RS`,
GPDMA1 requests `GPDMA_REQ_I2Cx_RX / _TX`), Fast-mode Plus 20 mA drive by `I2C_CR1.FMP` (no SYSCFG_PMCR bits),
kernel clock sources as on the classic lines (I2C1 shares its mux with I3C1), SCL / SDA pin enumerations from the
STM32H7RS open pin data. Unit tests: STM32H7 tests on STM32H7R / H7S (DMA stream tests and the SYSCFG Fast-mode Plus
test ignored there, Fast-mode Plus tests check `I2C_CR1.FMP`) - the STM32H5 module has no unit tests of the GPDMA data
handling, the DMA mode is verified by the integration tests on NUCLEO-H7S3L8 with the IT phase. The slave in DMA mode
counts the transmitted bytes as block size - remaining block size - GPDMA FIFO level (`Gpdma_Get_FifoLevel`).

### Device errata

STM32H7 errata sheets (ES0392 / ES0445 / ES0478) are not available offline - the I2C errata of the same I2C v2 IP (STM32F7 ES0334, STM32G4 ES0430) are handled and will be confirmed by the errata audit AB#844.

| Erratum | Handling |
|---------|----------|
| Wrong data sampling when data setup time (tSU;DAT) is shorter than one I2C kernel clock period | `I2c_Set_BusFreq()` / `I2c_Init()` refuse a kernel clock below 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus |
| Spurious bus error detection in master mode | BERR flag is only cleared, the transfer continues (`I2C_XFER_ERROR_BUS` is reported only when the transfer sequencing fails) |
| Transmission stalled after first byte transfer | `I2c_Set_BusFreq()` / `I2c_Init()` refuse an APB / I2CCLK ratio between 1.5 and 3 |
| 10-bit master mode: new transfer cannot be launched if first part of the address is not acknowledged by the slave | After STOPF of a NACKed 10-bit transfer the peripheral is disabled until CR2.START is released and enabled again |

Slave mode (errata of the same I2C IP, STM32H7 sheets not available offline): the spurious master transfer upon own slave address match is not applicable (the peripheral is either master or slave, `Role`), the SMBus errata are not applicable (SMBus is not supported). The OVR flag is not set in the underrun condition of a slave transmitter with clock stretching disabled - such an underrun is not reported as `I2C_XFER_ERROR_OVERRUN`, clock stretching enabled (default) avoids it.

Hardware status: the master integration tests are prepared for the STM32H7 Nucleo boards, the slave mode is tested by
unit tests on every device line and by master + slave integration tests on the I2C_LOOP wiring of the boards (master I2C1
PB8 / PB9 -> slave I2C2 PB10 / PB11 on NUCLEO-H7A3ZI-Q and NUCLEO-H7S3L8, I2C4 PB6 / PB7 on NUCLEO-H723ZG and
NUCLEO-H755ZI-Q; the slave DMA tests are ignored on I2C4). The STM32H7R / STM32H7S integration test firmware needs
`-DINTEGRATION_TEST_OPTIMIZATION=-Os` (64 KB flash).

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

### Slave
- `I2c_Set_ListenStart` / `I2c_Set_ListenStop` / `I2c_Get_ListenState`
- `I2c_Set_SlaveBuffer` (from the address callback: buffers of the starting phase)
- `I2c_Get_SlaveResult` (last transfer ended by STOP)

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

### Slave

```c
static i2c_Data_t regs[ 16u ];
static i2c_Data_t rxBuf[ 4u ];

static void App_SlaveAddr( i2c_SlaveDir_t slaveDir, i2c_SlaveAddr_t matchedAddr )
{
    (void)matchedAddr;

    if( ( I2C_SLAVE_DIR_READ == slaveDir ) &&
        ( 16u             > rxBuf[ 0u ] )    )
    {
        /* Register read: the register address was written before the repeated START */
        const i2c_SlaveBuffer_t buffer = { rxBuf, 4u, &regs[ rxBuf[ 0u ] ], 16u - rxBuf[ 0u ] };

        (void)I2c_Set_SlaveBuffer( I2C_PERIPH_2, &buffer );
    }
    else
    {
        /* Master writes - the register address and data are received into rxBuf */
    }
}

static const i2c_DataConfig_t slaveData =
{
    .XferMode                  = I2C_XFER_MODE_ISR,
    .IrqPriority               = 5u,
    .ErrorCallback             = App_I2cError,
    .SlaveAddrCallback         = App_SlaveAddr,
    .SlaveXferCompleteCallback = App_SlaveDone,
};

(void)I2c_Get_DefaultConfig( &i2cConfig );

i2cConfig.PeriphId            = I2C_PERIPH_2;
i2cConfig.Role                = I2C_ROLE_SLAVE;
i2cConfig.SlaveConfig.OwnAddr = 0x2Cu;
i2cConfig.DataConfig          = &slaveData;
i2cConfig.SclPin              = I2C_SCL_PIN_I2C2_PB10;
i2cConfig.SdaPin              = I2C_SDA_PIN_I2C2_PB11;

(void)I2c_Init( &i2cConfig );

const i2c_SlaveBuffer_t buffer = { rxBuf, 4u, regs, 16u };

(void)I2c_Set_ListenStart( I2C_PERIPH_2, &buffer );
```

The address match stretches SCL until the address callback returned and the phase is prepared. A byte
read by the master beyond `TxSize` is `FillByte`, a byte written beyond `RxSize` is acknowledged and
discarded (both reported in the result). In DMA mode the transfer complete of the DMA stream of a full buffer
switches the phase to the I2C interrupt for the bytes beyond the buffer. The DMA streams of the
classic STM32H7 lines work in direct mode (no FIFO) - the transmitted count is the data count moved by
the stream; on STM32H7R / STM32H7S the transmitted count takes the bytes read ahead into the GPDMA
FIFO into account (`Gpdma_Get_FifoLevel`). I2C4 has no DMA mode (BDMA / DMAMUX2), as for the master.
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
