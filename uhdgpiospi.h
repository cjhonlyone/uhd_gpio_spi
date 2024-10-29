#ifndef UHDGPIOSPI_H
#define UHDGPIOSPI_H


#include <iostream>
#include <vector>

#include <uhd/rfnoc/radio_control.hpp>

class UhdGpioSpi {
private:
    uint32_t SCK_MASK, SCS_MASK, SDO_MASK, SDI_MASK;
    double spi_frequency;
    bool cpol, cpha;
    uhd::rfnoc::radio_control::sptr radio_ctrl;

public:
    UhdGpioSpi(uhd::rfnoc::radio_control::sptr ctrl,
               uint32_t sck, uint32_t scs, uint32_t sdo, uint32_t sdi,
               double freq = 2e6, bool cpol = false, bool cpha = false);

    void init();
    void set_cpol(bool value);
    void set_cpha(bool value);
    void set_frequency(double freq);

    int write_and_read(uint8_t* write_buffer, uint8_t* read_buffer, uint32_t length);
};

#endif // UHDGPIOSPI_H
