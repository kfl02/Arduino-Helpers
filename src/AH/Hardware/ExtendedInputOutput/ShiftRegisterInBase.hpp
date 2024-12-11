/* ✔ */

#pragma once

#include "ExtendedInputOutput.hpp"
#include "StaticSizeExtendedIOElement.hpp"
#include <AH/Containers/BitArray.hpp>

BEGIN_AH_NAMESPACE

/**
 * @brief   A class for parallel-in/serial-out shift registers, 
 *          like the 74HC165.
 * 
 * @tparam  N
 *          The number of bits in total. Usually, shift registers (e.g. the
 *          74HC165) have eight bits per chip, so `length = 8 * k` where `k`
 *          is the number of cascaded chips.
 * 
 * @ingroup AH_ExtIO
 */
template <uint16_t N>
class ShiftRegisterInBase : public StaticSizeExtendedIOElement<N> {
  protected:
    /**
     * @brief   Create a new ShiftRegisterInBase object with a given bit order,
     *          and a given number of outputs.
     * 
     * @param   latchPin
     *          The digital output pin connected to the latch pin (SH_LD) 
     *          of the shift register.
     * @param   bitOrder
     *          Either `MSBFIRST` (most significant bit first) or `LSBFIRST`
     *          (least significant bit first).
     */
    ShiftRegisterInBase(pin_t latchPin, BitOrder_t bitOrder);

  public:
    /**
     * @brief   The pinMode function is not implemented because the mode is
     *          `INPUT` by definition.
     */
    void pinMode(pin_t pin, PinMode_t mode) override
        __attribute__((deprecated)) {
        (void)pin;
        (void)mode;
    }

    /**
     * @copydoc pinMode
     */
    void pinModeBuffered(pin_t pin, PinMode_t mode) override
        __attribute__((deprecated)) {
        (void)pin;
        (void)mode;
    }

    /**
     * @brief   Set the state of a given output pin.
     * 
     * @param   pin
     *          The shift register pin to set.
     * @param   val
     *          The value to set the pin to.
     *          (Either `HIGH` (1) or `LOW` (0))
     */
    void digitalWrite(pin_t pin, PinStatus_t val) override;

    /**
     * @brief   Set the output of a given pin in the software buffer.
     * @copydetails digitalWrite
     */
    void digitalWriteBuffered(pin_t pin, PinStatus_t val) override;

    /**
     * @brief   Get the current state of a given output pin.
     * 
     * @param   pin
     *          The shift register pin to read from.
     * @retval  0
     *          The state of the pin is `LOW`.
     * @retval  1
     *          The state of the pin is `HIGH`.
     */
    PinStatus_t digitalRead(pin_t pin) override;

    /** 
     * @copydoc digitalRead
     */
    PinStatus_t digitalReadBuffered(pin_t pin) override {
        return digitalRead(pin);
    }

    /**
     * @brief   The analogRead function is deprecated because a shift
     *          is always digital.
     * @param   pin
     *          The shift register pin to read from.
     * @retval  0
     *          The state of the pin is `LOW`.
     * @retval  1023
     *          The state of the pin is `HIGH`.
     */
    analog_t analogRead(pin_t pin) override __attribute__((deprecated)) {
        return ((1 << ADC_BITS) - 1) * digitalRead(pin);
    }

    /**
     * @copydoc analogRead
     */
    analog_t analogReadBuffered(pin_t pin) override
        __attribute__((deprecated)) {
        return ((1 << ADC_BITS) - 1) * digitalRead(pin);
    }

    /**
     * @brief   The analogWrite function is deprecated because a shift
     *          is always digital.
     * @param   pin
     *          The shift register pin to set.
     * @param   val
     *          The value to set the pin to. A value greater or equal to 0x80
     *          will set the pin to a `HIGH` state, a value less than 0x80 will
     *          set the pin to a `LOW` state.
     */
    void analogWrite(pin_t pin, analog_t val) override
        __attribute__((deprecated)) {
        digitalWrite(pin, val >= 0x80 ? HIGH : LOW);
    }

    /**
     * @copydoc analogWrite
     */
    void analogWriteBuffered(pin_t pin, analog_t val) override
        __attribute__((deprecated)) {
        digitalWrite(pin, val >= 0x80 ? HIGH : LOW);
    }

    /**
     * @brief   PISO shift registers don't have an output buffer.
     */
    void updateBufferedOutputs() override {} // LCOV_EXCL_LINE

  protected:
    const pin_t latchPin;
    const BitOrder_t bitOrder;

    BitArray<N> buffer;
};

END_AH_NAMESPACE

#include "ShiftRegisterInBase.ipp"
