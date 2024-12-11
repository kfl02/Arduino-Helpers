/* ✔ */

#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include <limits>
#include "AH/Hardware/ExtendedInputOutput/StaticSizeExtendedIOElement.hpp"
#include "AH/Hardware/ADCHelpers.hpp"
#include "AH/Math/IncreaseBitDepth.hpp"

BEGIN_AH_NAMESPACE


template <uint16_t I, 
          uint16_t O,
          size_t R,
          class Driver>
class ADDAMultiplex : 
    public StaticSizeExtendedIOElement<I + O> {
  protected:
    Driver& driver;

    ADDAMultiplex(Driver& driver) :
        driver(driver) {}

  public:
    using analog_native_t = decltype(bitWidthType<R>());
    const uint8_t analogResolution = R;

    constexpr analog_t nativeToAnalogT(analog_native_t val) {
        if constexpr(sizeof(analog_t) * CHAR_BIT > R) {
            return val >> (R - sizeof(analog_t) * CHAR_BIT);
        } else {
            return val;
        }
    }

    constexpr analog_native_t analogToNativeT(analog_t val) {
        if constexpr(R > sizeof(analog_t) * CHAR_BIT) {
            return increaseBitDepth<R, sizeof(analog_t) * CHAR_BIT,
                                    analog_native_t, analog_t>(val);
        } else {
            return val;
        }
    }

    virtual void digitalWriteNative(pin_t pin, PinStatus_t status) = 0;
    virtual PinStatus_t digitalReadNative(pin_t pin) = 0;
    virtual void analogWriteNative(pin_t pin, analog_native_t val) = 0;
    virtual analog_native_t analogReadNative(pin_t pin) = 0;

    void digitalWrite(pin_t pin, PinStatus_t status) override {
        analog_native_t val = (status == HIGH) ? std::numeric_limits<analog_native_t>::max() : 0;
        this->analogWriteNative(pin, val);        
    }

    PinStatus_t digitalRead(pin_t pin) override {
        return this->analogReadNative(pin) > (std::numeric_limits<analog_native_t>::max() >> 1) ? HIGH : LOW;
    }

    void analogWrite(pin_t pin, analog_t val) override {
        this->analogWriteNative(pin, analogToNativeT(val));
    }

    analog_t analogRead(pin_t pin) override {
        return nativeToAnalogT(this->analogReadNative(pin));
    }

    virtual analog_t analogReadBuffered(pin_t pin) {
        return this->analogRead(pin);
    }

    virtual void analogWriteBuffered(pin_t pin, analog_t val) {
        this->analogWrite(pin, val);
    }

    virtual void digitalWriteBuffered(pin_t pin, PinStatus_t status) {
        this->digitalWrite(pin, status);
    }

    virtual PinStatus_t digitalReadBuffered(pin_t pin) {
        return this->digitalRead(pin);
    }

    void begin() = 0;

    /**
     * @brief   The pinMode function is not implemented because the mode is
     *          implied by definition.
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
     * @brief   No periodic updating of the state is necessary, all actions are
     *          carried out when the user calls analogRead or digitalRead.
     */
    void updateBufferedOutputs() override {} // LCOV_EXCL_LINE

    /**
     * @brief   No periodic updating of the state is necessary, all actions are
     *          carried out when the user calls analogRead or digitalRead.
     */
    void updateBufferedInputs() override {} // LCOV_EXCL_LINE
};

template <uint16_t I, 
          size_t R,
          class Driver>
class ADMultiplex :
    public ADDAMultiplex<I, 0, R, Driver> {
  protected:
    ADMultiplex(Driver driver) :
        ADDAMultiplex<I, 0, R, Driver>(driver) {}

  public:
    /**
     * @brief   The analogWriteNative function is not implemented because writing
     *          an output to an ADC is not useful.
     */
    void analogWriteNative(pin_t pin, ADDAMultiplex<I, 0, R, Driver>::analog_native_t val) final // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}                           // LCOV_EXCL_LINE

    /**
     * @brief   The digitalWrite function is not implemented because writing an
     *          output to an ADC is not useful.
     */
    void digitalWrite(pin_t, PinStatus_t) final // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}          // LCOV_EXCL_LINE

    /**
     * @copydoc digitalWrite
     */
    void digitalWriteBuffered(pin_t, PinStatus_t) final // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}                  // LCOV_EXCL_LINE

    /**
     * @brief   The analogWrite function is not implemented because writing an
     *          output to an ADC is not useful.
     */
    void analogWrite(pin_t, analog_t) final // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}      // LCOV_EXCL_LINE

    /**
     * @copydoc analogWrite
     */
    void analogWriteBuffered(pin_t, analog_t) final // LCOV_EXCL_LINE
        __attribute__((deprecated)) {}              // LCOV_EXCL_LINE
};

template <uint16_t O,
          size_t R,
          class Driver>
class DAMultiplex :
    public ADDAMultiplex<0, O, R, Driver> {
  protected:
    DAMultiplex(Driver driver) :
        ADDAMultiplex<0, O, R, Driver>(driver) {}

  public:
    /**
     * @brief   The analogReadNative function is not implemented because reading
     *          an input from a DAC is not useful.
     */
    ADDAMultiplex<0, O, R, Driver>::analog_native_t analogReadNative(pin_t pin) final   // LCOV_EXCL_LINE
        __attribute__((deprecated)) { return 0; }                                       // LCOV_EXCL_LINE

    /**
     * @brief   The digitalRead function is not implemented because reading an
     *          input from a DAC is not useful.
     */
    PinStatus_t digitalRead(pin_t pin) final        // LCOV_EXCL_LINE
        __attribute__((deprecated)) { return 0; }   // LCOV_EXCL_LINE

    /**
     * @copydoc digitalRead
     */
    PinStatus_t digitalReadBuffered(pin_t pin) final    // LCOV_EXCL_LINE
        __attribute__((deprecated)) { return 0; }       // LCOV_EXCL_LINE

    /**
     * @brief   The analogRead function is not implemented because reading an
     *          input from a DAC is not useful.
     */
    analog_t analogRead(pin_t pin) final            // LCOV_EXCL_LINE
        __attribute__((deprecated)) { return 0; }   // LCOV_EXCL_LINE

    /**
     * @copydoc analogRead
     */
    analog_t analogReadBuffered(pin_t pin) final    // LCOV_EXCL_LINE
        __attribute__((deprecated)) { return 0; }  // LCOV_EXCL_LINE
};

END_AH_NAMESPACE

AH_DIAGNOSTIC_POP()
