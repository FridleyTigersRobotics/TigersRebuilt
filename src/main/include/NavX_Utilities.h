#pragma once

enum PinType { DigitalIO, PWM, AnalogIn, AnalogOut };

/* GetChannelFromPin( PinType, int ) - converts from a navX-MXP */
/* Pin type and number to the corresponding RoboRIO Channel     */
/* Number, which is used by the WPI Library functions.          */
int GetChannelFromPin( PinType type, int io_pin_number );

static const int MAX_NAVX_MXP_DIGIO_PIN_NUMBER      = 9;
static const int MAX_NAVX_MXP_ANALOGIN_PIN_NUMBER   = 3;
static const int MAX_NAVX_MXP_ANALOGOUT_PIN_NUMBER  = 1;
static const int NUM_ROBORIO_ONBOARD_DIGIO_PINS     = 10;
static const int NUM_ROBORIO_ONBOARD_PWM_PINS       = 10;
static const int NUM_ROBORIO_ONBOARD_ANALOGIN_PINS  = 4;