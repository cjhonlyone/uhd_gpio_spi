## Using USRP X310 Front GPIO as SPI Interface (Write Only)

This project demonstrates how to use the front-panel GPIO pins of the USRP X310 to emulate an SPI interface. The SPI communication leverages UHD timed commands and RFNoC blocks to drive GPIO signals at precise intervals.

### Features

- SPI Communication via GPIO: Emulates SPI over front-panel GPIOs using the UHD API.
- Configurable SPI Mode: Supports setting Clock Polarity (CPOL) and Clock Phase (CPHA).
- SPI Frequency Control: Allows adjusting SPI clock frequency up to 2 MHz. 
- Burst Write and Read: Supports bursts up to 6 bytes per transfer. 
- Full-Duplex Operation: Transmits and receives data simultaneously (with caveats on read performance).

### Example

```c++
    UhdGpioSpi *UhdGpioSpiInstance;
    UhdGpioSpiInstance = new UhdGpioSpi(radio_ctrl,GPIO_SCK_MASK, GPIO_SDO_MASK, GPIO_SDI_MASK, GPIO_SCS_MASK);
    UhdGpioSpiInstance->set_cpol(false);
    UhdGpioSpiInstance->set_cpha(false);
    UhdGpioSpiInstance->set_frequency(1e6);
    UhdGpioSpiInstance->init();
    UhdGpioSpiInstance->write_and_read(spi_write_data, spi_read_data, 4);

```

### Known Issues

Can't read by now.

The use of `radio_ctrl->get_gpio_attr("FP0", "READBACK")` introduces significant latency. I don't know why.