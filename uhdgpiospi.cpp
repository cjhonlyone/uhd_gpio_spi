#include "uhdgpiospi.h"

UhdGpioSpi::UhdGpioSpi(uhd::rfnoc::radio_control::sptr ctrl,
                       uint32_t sck, uint32_t sdo, uint32_t sdi, uint32_t scs,
                       double freq, bool cpol, bool cpha)
    : radio_ctrl(ctrl), SCK_MASK(sck), SCS_MASK(scs),
      SDO_MASK(sdo), SDI_MASK(sdi), spi_frequency(freq), cpol(cpol), cpha(cpha)
{

}

void UhdGpioSpi::init() {
    radio_ctrl->set_gpio_attr("FP0", "CTRL", 0x00000000);
    radio_ctrl->set_gpio_attr("FP0", "DDR", SCK_MASK | SCS_MASK | SDO_MASK);
    radio_ctrl->set_gpio_attr("FP0", "OUT", SCS_MASK | (cpol ? SCK_MASK : 0x00000000));
}


void UhdGpioSpi::set_cpol(bool value) { cpol = value; }

void UhdGpioSpi::set_cpha(bool value) { cpha = value; }

void UhdGpioSpi::set_frequency(double freq) { spi_frequency = freq; }

int UhdGpioSpi::write_and_read(uint8_t* write_buffer, uint8_t* read_buffer, uint32_t length) {

    double spi_period = 1.0 / spi_frequency;
    radio_ctrl->clear_command_time(0);
    uhd::time_spec_t time = radio_ctrl->get_time_now() + uhd::time_spec_t(0.05);
    time = time + uhd::time_spec_t(spi_period);
    radio_ctrl->set_command_time(time, 0);
    radio_ctrl->set_gpio_attr("FP0", "OUT", (~SDO_MASK) & (~SCS_MASK) & (cpol ? 0xffffffff : (~SCK_MASK)));

    for (uint32_t i = 0; i < length; i++) {
        for (uint32_t j = 0; j < 8; j++) {
            time = time + uhd::time_spec_t(0.5 * spi_period);
            radio_ctrl->set_command_time(time, 0);
            if ((cpha & cpol) || ((!cpha) & (!cpol))) { // SCK = 0, SCS = 0
                radio_ctrl->set_gpio_attr("FP0", "OUT", (write_buffer[i] & (1 << (7 - j))) ? (~SCS_MASK) & (~SCK_MASK) : ((~SCS_MASK) & (~SCK_MASK) & (~SDO_MASK)));
            } else { // SCK = 1, SCS = 0
                radio_ctrl->set_gpio_attr("FP0", "OUT", (write_buffer[i] & (1 << (7 - j))) ? (~SCS_MASK) : (~SCS_MASK) & (~SDO_MASK));
            }

            time = time + uhd::time_spec_t(0.5 * spi_period);
            radio_ctrl->set_command_time(time, 0);
            if ((cpha & cpol) || ((!cpha) & (!cpol))) { // SCK = 1, SCS = 0
                radio_ctrl->set_gpio_attr("FP0", "OUT", (write_buffer[i] & (1 << (7 - j))) ? (~SCS_MASK) : (~SCS_MASK) & (~SDO_MASK));
            } else { // SCK = 0, SCS = 0
                radio_ctrl->set_gpio_attr("FP0", "OUT", (write_buffer[i] & (1 << (7 - j))) ? (~SCS_MASK) & (~SCK_MASK) : ((~SCS_MASK) & (~SCK_MASK) & (~SDO_MASK)));
            }

//            radio_ctrl->set_command_time(time, 0);
//            if (radio_ctrl->get_gpio_attr("FP0", "READBACK") & SDI_MASK)
//                read_buffer[i] |= (1 << (7 - j));
//            if (j < 7) read_buffer[i] <<= 1;
        }

        // nop
        time = time + uhd::time_spec_t(0.5*spi_period);
        radio_ctrl->set_command_time(time, 0);
        radio_ctrl->set_gpio_attr("FP0", "OUT", (~SDO_MASK) & (~SCS_MASK) & (cpol ? 0xffffffff : (~SCK_MASK)));

    }

    time = time + uhd::time_spec_t(spi_period);
    radio_ctrl->set_command_time(time, 0);
    radio_ctrl->set_gpio_attr("FP0", "OUT", SCS_MASK | (cpol ? SCK_MASK : 0x00000000));

    return 0;
}
