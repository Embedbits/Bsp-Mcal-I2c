# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32F4 MCUs (I2C v1 peripheral) in **master or slave mode**.  
It supports initialization, SCL frequency configuration with automatic CCR / TRISE calculation, noise filters, and data transfers in DMA, interrupt or polling mode.
The public interface is shared with the STM32H5 family.

---

## Features

- I2C clock is APB1 clock (PCLK1, 2 - 50 MHz) - `I2C_CLK_SRC_PCLK` only
- SCL frequency in Hz (up to 400 kHz) - CR2 FREQ, CCR (duty cycle 2 or 16/9) and TRISE are calculated from PCLK1 and I2C specification timings
- Analog and digital noise filter on devices with FLTR register (not STM32F405 / 407 / 415 / 417 - only filters off accepted there)
- 7-bit and 10-bit addressing
- Transfers: write, read, write + repeated START + read (register access), address only (device presence check)
- Transfers up to 65535 bytes per phase (I2C v1 has no byte counter - master sequencing in software, reception by BTF procedure of RM)
- Data transfer modes: DMA (DMA1 streams), ISR, POLL (`I2c_Task()`) - same request, same callbacks
- SCL / SDA pin lists per device family (open-drain alternate function)
- Slave mode (`Role = I2C_ROLE_SLAVE`): own address 1 (7-bit or 10-bit), own address 2, general call, clock stretching
  enabled / disabled, fill byte for a master read beyond the buffer; listen with buffers of the application, address
  match callback (direction, matched address - the buffers can be replaced there, eg. register device: register
  address written, repeated START, register data read), transfer complete callback with the result (received /
  transmitted byte count, bytes beyond the buffers); DMA / ISR / POLL as for the master

Not supported: SMBus, FMPI2C1 (STM32F410 / 412 / 413 / 423 / 446), Fast-mode Plus, bus recovery (slave holding SDA low), transfer timeout, mask of own address 2 (see the slave specifics).

### STM32F4 specifics

- Transfer end: the STOP condition is requested at the end of the transfer, the next `I2c_Set_XferStart()` waits until it is sent.
- Transfer abort (`I2c_Set_XferStop()`, DMA / sequencing errors) uses software reset (SWRST) with restored configuration - clearing PE does not stop a running communication on I2C v1.
- DMA mode: `TxDma` / `RxDma` select the DMA1 **stream** from the lists `i2c_TxDma_t` / `i2c_RxDma_t` - one item per I2C peripheral and stream, named `I2C_TX_DMA_I2Cx_DMA1_STREAMz` / `I2C_RX_DMA_I2Cx_DMA1_STREAMz` (e.g. STM32F407 I2C1: RX `I2C_RX_DMA_I2C1_DMA1_STREAM0` / `..._STREAM5`, TX `I2C_TX_DMA_I2C1_DMA1_STREAM6` / `..._STREAM7`), the channel selection of the stream is part of the item. Items of another I2C peripheral, items of the other direction and `I2C_TX_DMA_UNUSED` / `I2C_RX_DMA_UNUSED` are refused in the DMA mode, items of streams existing only on some device lines are guarded by the CMSIS device line. Single byte reception is done by I2C interrupt (DMA can not NACK a single byte).
- Device errata handling: spurious bus error (BERR) in master mode is cleared and ignored. Repeated START setup time may be violated in Standard-mode above 88 kHz - use up to 88 kHz or Fast-mode with strict slaves.
- STM32F4 LL `LL_I2C_IsEnabledAnalogFilter()` returns inverted state - the module reads FLTR ANOFF directly.

### STM32F4 slave specifics (differences to the STM32H5 slave: I2C v1 has SR1 / SR2, no ISR / ICR / ADDCODE / TXDR prefetch counter)

- Same public interface and behavior as STM32H5 (`I2c_Set_ListenStart` / `Stop`, `I2c_Get_ListenState`, `I2c_Set_SlaveBuffer`, `I2c_Get_SlaveResult`, `SlaveAddrCallback`, `SlaveXferCompleteCallback`); the sequencing of all modes is `I2c_Set_SlaveEvents()` called with a snapshot of SR1.
- Own addresses: I2C v1 has no own address enable bits - the slave answers while ACK is set (`I2c_Set_ListenStart()` sets ACK, ENDUAL for own address 2 and ENGC for the general call, `I2c_Set_ListenStop()` clears them). Own address 1 is OAR1 (bit 14 kept at 1, `ADDMODE` for 10-bit), own address 2 is OAR2.
- **Own address 2 has no mask** (OAR2 compares all 7 bits): `OwnAddr2Mask` other than `I2C_OWN_ADDR2_MASK_NONE` is refused by `I2c_Init()`. Own address 2 is recognized by I2C v1 only together with a 7-bit own address 1 (RM: dual addressing in 7-bit mode) - the combination with a 10-bit own address 1 is refused by `I2c_Init()`.
- Matched address: no address code in I2C v1 - SR2 `GENCALL` gives the general call (0x00), `DUALF` own address 2, otherwise own address 1 (7-bit or 10-bit) is reported. The direction is `TRA` of SR2. Reading of SR2 clears ADDR and releases SCL **before** `SlaveAddrCallback` is called; the clock stays stretched by the data phase (transmit: until the first byte is written, receive: after the second byte, clock stretching enabled), buffers replaced in the callback apply to the starting phase.
- **End of a transfer whose last phase is a read**: STOPF is not set after the NACK of the master (RM) - the NACK (AF) ends the read phase and the transfer (`SlaveXferCompleteCallback` is called at the NACK, not at STOP). A transfer ending with a write phase ends at STOP (STOPF). A master that does not NACK the last byte of a read phase is not supported by I2C v1.
- The byte written in advance to DR (TXE = 0 at the end of the phase) is subtracted from the transmitted count, as the TXDR prefetch of STM32H5; the unsent byte left in DR by a NACK was not found to be sent as the first byte of the next read phase (integration test `It_I2c_Slave_ShortReadsAllModes_FirstByteNotStale`).
- Slave DMA mode: the stream of the direction of the address match is armed with the buffer of the slave and `DMAEN` is set; the transfer complete of the stream switches the bytes beyond the buffer to the buffer interrupt (fill bytes / discarded bytes). The DMA streams use the direct mode (no FIFO, 8-bit sizes), the transmitted count is the block size minus the remaining count of the stream (no FIFO level as on GPDMA, `Dma` module is unchanged). The transfer complete interrupt of the transmit stream is enabled only for a slave.
- Interrupts: the event and error interrupts are enabled while the slave listens, the buffer interrupt (RXNE / TXE) only for the running phase of ISR mode and for the bytes beyond the buffer in DMA mode (TXE stays set after the end of a read phase). A DMA error of a slave aborts the transfer (software reset with restored configuration, listening continues).
- Errors (BERR, ARLO, OVR with clock stretching disabled) end the running transfer without result and are reported by `ErrorCallback`; the slave keeps listening.
- Tests: unit tests 73 per device line (RegMem emulation, SR1 / SR2 preset by the tests), integration tests 26 (master + slave of one MCU wired together: NUCLEO-F411RE and NUCLEO-F446RE I2C1 PB8 / PB9 -> I2C3 PA8 / PC9, NUCLEO-F413ZH I2C1 PB8 / PB9 -> I2C2 PB10 / PB11, fixture feature `I2C_LOOP`).

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
- `I2c_Get_SlaveResult` (last finished transfer)

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

A byte read by the master beyond `TxSize` is `FillByte`, a byte written beyond `RxSize` is acknowledged and
discarded (both reported in the result). In DMA mode the transfer complete of a full buffer switches the phase
to the I2C interrupt for the bytes beyond the buffer.

SCL / SDA pins are selected from `i2c_SclPin_t` / `i2c_SdaPin_t` - every pin item is active on exactly the CMSIS device lines whose package or die has the pin according to the STM32CubeMX GPIO modes database (guards by the device line). The pin must belong to `PeriphId`, otherwise `I2c_Init()` returns error.

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
