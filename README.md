# I2C Peripheral Driver

This module provides an abstraction layer for configuring and managing **I2C peripherals** on STM32U5 MCUs in **master or slave mode**.  
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
- Slave mode (`Role = I2C_ROLE_SLAVE`): own address 1 (7-bit or 10-bit), own address 2 with mask,
  general call, clock stretching enabled / disabled, fill byte for a master read beyond the buffer;
  listen with buffers of the application, address match callback (direction, matched address - the
  buffers can be replaced there, eg. register device: register address written, repeated START,
  register data read), transfer complete callback with the result (received / transmitted byte
  count, bytes beyond the buffers); DMA / ISR / POLL as for the master

Not supported: SMBus, wake-up from Stop mode by address match, bus recovery (slave holding SDA low),
transfer timeout.

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
discarded (both reported in the result). In DMA mode the GPDMA transfer complete of a full buffer
switches the phase to the I2C interrupt for the bytes beyond the buffer; the transmitted count takes
the bytes read ahead into the GPDMA FIFO into account (`Gpdma_Get_FifoLevel`).

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
