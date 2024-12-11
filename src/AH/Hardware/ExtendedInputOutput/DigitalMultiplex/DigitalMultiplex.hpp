/* ✔ */

#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include "AH/Hardware/ExtendedInputOutput/StaticSizeExtendedIOElement.hpp"

BEGIN_AH_NAMESPACE

/**
 * @brief   Base class for I/O expanders.
 *
 * @tparam  Driver
 *          The type of the driver to use.
 *
 * @tparam  N
 *          The number of pins
 */
template <class Driver, uint16_t N>
class DigitalMultiplex : 
    public StaticSizeExtendedIOElement<N> {
  protected:
    Driver& driver;

    DigitalMultiplex(Driver& driver) : driver(driver) {}

  public:
    void pinModeBuffered(pin_t pin, PinMode_t mode) = 0;
    void digitalWriteBuffered(pin_t pin, PinStatus_t status) = 0;
    PinStatus_t digitalReadBuffered(pin_t pin) = 0;
    analog_t analogReadBuffered(pin_t pin) = 0;
    void analogWriteBuffered(pin_t, analog_t) = 0;

    void updateBufferedOutputs() = 0;
    void updateBufferedInputs() = 0;
    virtual void updateBufferedPinModes() = 0;
};

END_AH_NAMESPACE

AH_DIAGNOSTIC_POP()
