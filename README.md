# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32L4 / STM32L4+ MCUs in **master or slave mode**.
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
- Slave mode (`Role = I2C_ROLE_SLAVE`): own address 1 (7-bit or 10-bit), own address 2 with mask,
  general call, clock stretching enabled / disabled, fill byte for a master read beyond the buffer;
  listen with buffers of the application, address match callback (direction, matched address - the
  buffers can be replaced there, eg. register device: register address written, repeated START,
  register data read), transfer complete callback with the result (received / transmitted byte
  count, bytes beyond the buffers); DMA / ISR / POLL as for the master

Not supported: SMBus, wake-up from Stop mode by address match, bus recovery (slave holding SDA low),
transfer timeout.

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
| DMA                       | `TxDma` / `RxDma` select the DMA channel from the lists `i2c_TxDma_t` / `i2c_RxDma_t` - one item per I2C peripheral, DMA peripheral and channel, named `I2C_TX_DMA_I2Cx_DMAy_CHANNELz` / `I2C_RX_DMA_I2Cx_DMAy_CHANNELz` (e.g. `I2C_TX_DMA_I2C1_DMA1_CHANNEL6`); transmit and receive channel must differ. STM32L4: fixed request mapping (DMA_CSELR, the request selection is part of the item) - I2C1 TX / RX DMA1 channel 6 / 7 or DMA2 channel 7 / 6, I2C2 TX / RX DMA1 channel 4 / 5, I2C3 TX / RX DMA1 channel 2 / 3, I2C4 TX / RX DMA2 channel 2 / 1; the lists contain these channels only. STM32L4+: any DMA1 / DMA2 channel through DMAMUX1, the lists contain all of them. Items of another I2C peripheral and `I2C_TX_DMA_UNUSED` / `I2C_RX_DMA_UNUSED` are refused in the DMA mode. The transfer error interrupt of the channels is used, the transfer complete interrupt only by the slave phase (DMA without FIFO) |
| Interrupt lines           | Event (`I2Cx_EV`) and error (`I2Cx_ER`) line per peripheral, one handler (I2C4 lines in STM32L4+ order on STM32L4+) |
| Errors                    | NACK, arbitration lost, DMA transfer error (also STOP before DMA moved all bytes); bus error (BERR) is not reported in master mode - see errata (the slave reports it); slave: overrun / underrun (OVR, clock stretching disabled) |

### Device errata

The workarounds of the STM32G4 module (same I2C IP, STM32G4 errata sheets ES0430 / ES0431 / ES0523) are kept - STM32L4 / STM32L4+ errata sheets are not reviewed yet:

| STM32G4 erratum | Handling |
|-----------------|----------|
| ES0430 2.15.1 - wrong data sampling when data setup time (tSU;DAT) is shorter than one I2C kernel clock period | `I2c_Set_BusFreq()` / `I2c_Init()` refuse a kernel clock below 4 / 10 / 20 MHz for Standard-mode / Fast-mode / Fast-mode Plus |
| ES0430 2.15.2 - spurious bus error detection in master mode | master: BERR flag is only cleared, the transfer continues (`I2C_XFER_ERROR_BUS` is reported only when the transfer sequencing fails); the slave reports BERR as `I2C_XFER_ERROR_BUS` |
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
discarded (both reported in the result). In DMA mode the DMA transfer complete of a full buffer
switches the phase to the I2C interrupt for the bytes beyond the buffer; the DMA of STM32L4 has no
FIFO, the transmitted count is the count of bytes written to TXDR (the byte written in advance and not
read by the master is subtracted at the end of the phase).

SCL / SDA pins are selected from `i2c_SclPin_t` / `i2c_SdaPin_t` - only pins available on the selected device line are defined. The pin must belong to `PeriphId`, otherwise `I2c_Init()` returns error.

---

## Testing

- Unit tests (host, Unity / CMock / RegMem): `Tests/UnitTests/Test_I2c.c` - I2C and SYSCFG registers emulated, RCC / NVIC / GPIO / DMA mocked; run on several presets (I2C2 / I2C4 availability differs).
- Integration tests (target, boards named by the MCU): `Tests/IntegrationTests/ItTest_I2c.c` - configuration, kernel clock, Fast-mode Plus, NACK of a not present slave (master I2C1) and master + slave tests in which the slave is the module itself (slave mode, register device emulated in its callbacks): board fixture `I2C_LOOP` of the integration test inventory (the tests are ignored on a board with a fixture without it and on boards without a known wiring). Wiring:
  - NUCLEO-L4R5ZI: master I2C1 PB8 (SCL) / PB9 (SDA) -> slave I2C2 PB10 (SCL) / PB11 (SDA), external pull-ups 4.7 kOhm; DMA1 channels 6 / 7 (master) and 4 / 5 (slave)
  - other boards: master I2C1 PB8 / PB9 (Nucleo-64 / Nucleo-144) or PA9 / PA10 (Nucleo-32 and Nucleo-64-P of the same MCU names), no wiring of a slave known

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
