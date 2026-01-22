#include "NavX_Utilities.h"
#include <fmt/printf.h>

// Function from Kauailab Website:
//   https://pdocs.kauailabs.com/navx-mxp/examples/mxp-io-expansion/
int GetChannelFromPin( PinType type, int io_pin_number ) {
    int roborio_channel = 0;
    if ( io_pin_number < 0 ) {
        throw std::runtime_error("Error:  navX-MXP I/O Pin #");
    }
    switch ( type ) {
    case DigitalIO:
        if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
            throw std::runtime_error("Error:  Invalid navX-MXP Digital I/O Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_DIGIO_PINS +
                          (io_pin_number > 3 ? 4 : 0);
        break;
    case PWM:
        if ( io_pin_number > MAX_NAVX_MXP_DIGIO_PIN_NUMBER ) {
            throw std::runtime_error("Error:  Invalid navX-MXP Digital I/O Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_PWM_PINS;
        break;
    case AnalogIn:
        if ( io_pin_number > MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER ) {
            throw new std::runtime_error("Error:  Invalid navX-MXP Analog Input Pin #");
        }
        roborio_channel = io_pin_number + NUM_ROBORIO_ONBOARD_ANALOGIN_PINS;
        break;
    case AnalogOut:
        if ( io_pin_number > MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER ) {
            throw new std::runtime_error("Error:  Invalid navX-MXP Analog Output Pin #");
        }
        roborio_channel = io_pin_number;
        break;
    }
    return roborio_channel;
}


