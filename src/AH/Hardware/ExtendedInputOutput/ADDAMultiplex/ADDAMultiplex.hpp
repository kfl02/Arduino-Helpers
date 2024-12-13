/* ✔ */

#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include <limits>
#include "AH/Hardware/ExtendedInputOutput/StaticSizeExtendedIOElement.hpp"
#include "AH/Hardware/ADCHelpers.hpp"
#include "AH/Math/IncreaseBitDepth.hpp"

BEGIN_AH_NAMESPACE


/**
 * @brief   An abstract base class for AD/DA converters.
 *          For converters with a bit depth greater than 16 bit (the wdith of
 *          analog_t), @ref analogReadNative and @ref analogWriteNative 
 *          functions have to be overridden in the implementation.
 *          The @ref analogRead and @ref analogWrite default implementations
 *          will take care of converting values to the appropriate range.
 *          If you need access to the full range of values, you have to call 
 *          @ref analogReadNative and @ref analogWriteNative directly, not via
 *          the ExtIO wrapper. Remember to take care of the pin numbers by
 *          subtracting @ref getStart in such cases.
 *
 * @tparam  I
 *          The number of input pins.
 * 
 * @tparam  O
 *          The number of output pins.
 * 
 * @tparam  R
 *          The native bit depth of the AD/DA converter.
 * 
 * @tparam  Driver
 *          The driver class to use (e.g. SPIClass, TwoWire).
 * 
 */
template <uint16_t I, 
          uint16_t O,
          size_t R,
          class Driver>
class ADDAMultiplex : 
    public StaticSizeExtendedIOElement<I + O> {
  protected:
    Driver& driver;

    /**
     * @brief   Constructor for the abstract AD/DA converter.
     * 
     * @param   driver
     *          The driver interface to use (e.g. SPI, Wire).
     */
    ADDAMultiplex(Driver& driver) :
        driver(driver) {}

  public:
    using native_analog_t = decltype(bitWidthType<R>());
    const uint8_t analogResolution = R;

    /**
     * @brief   Convert a native_analog_t value, which is as wide as needed for
     *          the given bit depth of the AD/DA converter, to an analog_t 
     *          (16 bit) by right shifting, if necessary.
     * 
     * @return  The converted value.
     */
    constexpr analog_t nativeToAnalogT(native_analog_t val) {
        if constexpr(R > sizeof(analog_t) * CHAR_BIT) {
            return val >> (R - sizeof(analog_t) * CHAR_BIT);
        } else {
            return val;
        }
    }

    /**
     * @brief   Convert a analog_t value (16 bit), to a native_analog_t value, 
     *          which is as wide as needed for the given bit depth of the 
     *          AD/DA converter, by using @ref increaseBitDepth, if necessary.
     * 
     * @return  The converted value.
     */
    constexpr native_analog_t analogToNativeT(analog_t val) {
        if constexpr(R > sizeof(analog_t) * CHAR_BIT) {
            return increaseBitDepth<R, sizeof(analog_t) * CHAR_BIT,
                                    native_analog_t, analog_t>(val);
        } else {
            return val;
        }
    }

    /**
     * @brief   Write an analog (or PWM) value to the given pin.
     *          Override this function if the native bit depth of the AD/DA
     *          converter is greater than the bit depth of analog_t (16).
     * 
     * @param   pin 
     *          The (zero-based) pin of this IO element.
     * @param   val 
     *          The new analog value to set the pin to.
     */
    virtual void analogWriteNative(pin_t pin, native_analog_t val) = 0;

    /**
     * @brief   Read the analog value of the given pin.
     *          Override this function if the native bit depth of the AD/DA
     *          converter is greater than the bit depth of analog_t (16).
     * 
     * @param   pin 
     *          The (zero-based) pin of this IO element.
     * @return  The new analog value of pin.
     */
    virtual native_analog_t analogReadNative(pin_t pin) = 0;

    /**
     * @brief   Write an analog (or PWM) value to the given pin.
     *          If the native bit depth of the AD/DA converter ist greater than
     *          the bit depth of analog_t, the given value's bit depth will be
     *          increased appropiately. 
     *          You can override this function directly instead of 
     *          @ref analogWriteNative if the bit depth is less or equal than 
     *          the bit depth of analog_t (16).
     * 
     * @param   pin 
     *          The (zero-based) pin of this IO element.
     * @param   val 
     *          The new analog value to set the pin to.
     */
    void analogWrite(pin_t pin, analog_t val) override {
        this->analogWriteNative(pin, analogToNativeT(val));
    }

    /**
     * @brief   Read the analog value of the given pin.
     *          If the native bit depth of the AD/DA converter ist greater than
     *          the bit depth of analog_t, the native value will be shifted
     *          right appropiately.
     *          You can override this function directly instead of 
     *          @ref analogWriteNative if the bit depth is smaller than the bit
     *          depth of analog_t (16).
     * 
     * @param   pin 
     *          The (zero-based) pin of this IO element.
     * @return  The new analog value of pin.
     */
    analog_t analogRead(pin_t pin) override {
        return nativeToAnalogT(this->analogReadNative(pin));
    }

    /** 
     * @brief   Set the output of the given pin to the given state.
     *          If the given status is high, the analog value will be set to
     *          the maximum value of the native bit depth, 0 otherwise.
     * 
     * @param   pin
     *          The (zero-based) pin of this IO element.
     * @param   state
     *          The new state to set the pin to.
     */
    void digitalWrite(pin_t pin, PinStatus_t status) override {
        if constexpr(R > sizeof(analog_t) * CHAR_BIT) {
            native_analog_t val = 
                (status == HIGH) 
                    ? std::numeric_limits<native_analog_t>::max() : 0;
            this->analogWriteNative(pin, val);
        } else {
            analog_t val = 
                (status == HIGH) 
                    ? std::numeric_limits<native_analog_t>::max() : 0;
            this->analogWrite(pin, val);
        }
    }

    /** 
     * @brief   Read the state of the given pin.
     *          If the analog value is greater than the half of the maximum 
     *          value of the native bit depth, HIGH is returned, LOW otherwise.
     * 
     * @param   pin
     *          The (zero-based) pin of this IO element.
     * @return  The state of the given pin.
     */
    PinStatus_t digitalRead(pin_t pin) override {
        if constexpr(R > sizeof(analog_t) * CHAR_BIT) {
            return this->analogRead(pin) 
                    > (std::numeric_limits<analog_t>::max() >> 1) 
                    ? HIGH : LOW;
        } else {
            return this->analogRead(pin) 
                    > (std::numeric_limits<native_analog_t>::max() >> 1) 
                    ? HIGH : LOW;
        }
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


/**
 * @brief   An abstract base class for AD converters.
 *          The write functions are disabled because writing to an ADC is not 
 *          useful.
 *
 * @tparam  I
 *          The number of input pins.
 * 
 * @tparam  R
 *          The native bit depth of the AD converter.
 * 
 * @tparam  Driver
 *          The driver class to use (e.g. SPIClass, TwoWire).
 * 
 */
template <uint16_t I, 
          size_t R,
          class Driver>
class ADMultiplex :
    public ADDAMultiplex<I, 0, R, Driver> {
  protected:
    ADMultiplex(Driver driver) :
        ADDAMultiplex<I, 0, R, Driver>(driver) {}

  public:
    using native_analog_t = ADDAMultiplex<I, 0, R, Driver>::native_analog_t;

    /**
     * @brief   The analogWriteNative function is not implemented because writing
     *          an output to an ADC is not useful.
     */
    void analogWriteNative(pin_t pin, native_analog_t val) final // LCOV_EXCL_LINE
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


/**
 * @brief   An abstract base class for DA converters.
 *          The read functions are disabled because reading from a DAC is not 
 *          useful.
 *
 * @tparam  O
 *          The number of output pins.
 * 
 * @tparam  R
 *          The native bit depth of the DA converter.
 * 
 * @tparam  Driver
 *          The driver class to use (e.g. SPIClass, TwoWire).
 * 
 */
template <uint16_t O,
          size_t R,
          class Driver>
class DAMultiplex :
    public ADDAMultiplex<0, O, R, Driver> {
  protected:
    DAMultiplex(Driver driver) :
        ADDAMultiplex<0, O, R, Driver>(driver) {}

  public:
    using native_analog_t = ADDAMultiplex<0, O, R, Driver>::native_analog_t;

    /**
     * @brief   The analogReadNative function is not implemented because reading
     *          an input from a DAC is not useful.
     */
    native_analog_t analogReadNative(pin_t pin) final   // LCOV_EXCL_LINE
        __attribute__((deprecated)) { return 0; }       // LCOV_EXCL_LINE

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
