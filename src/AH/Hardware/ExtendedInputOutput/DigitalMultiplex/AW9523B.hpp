#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include <AH/Containers/Array.hpp>
#include <AH/Containers/ArrayHelpers.hpp>
#include <AH/Containers/BitArray.hpp>
#include "AH/Hardware/ExtendedInputOutput/DigitalMultiplex/DigitalMultiplex.hpp"

AH_DIAGNOSTIC_EXTERNAL_HEADER()
#include <Wire.h>
#include <SPI.h>
AH_DIAGNOSTIC_POP()

BEGIN_AH_NAMESPACE

template <class Driver, uint16_t N = 16>
class AW9523B : 
    public virtual DigitalMultiplex<Driver, N> {
  private:
    enum AW9523BRegAddr {
        INPUT_PORT0 = 0x00,
        INPUT_PORT1,
        OUTPUT_PORT0,
        OUTPUT_PORT1,
        CONFIG_PORT0,
        CONFIG_PORT1,
        INT_PORT0,
        INT_PORT1,
        ID = 0x10,
        GCR,
        LED_MODE_SWITCH0,
        LED_MODE_SWITCH1,
        DIM0 = 0x20,
        DIM1,
        DIM2,
        DIM3,
        DIM4,
        DIM5,
        DIM6,
        DIM7,
        DIM8,
        DIM9,
        DIM10,
        DIM11,
        DIM12,
        DIM13,
        DIM14,
        DIM15,
        SW_RSTN = 0x7f
    };
    const uint8_t BASE_ADDRESS = 0b1011000;

    uint8_t address;
    pin_t interruptPin;

    bool pinModesDirty = true;
    // Output mode by default
    BitArray<N> bufferedPinModeBits;
    Array<PinMode_t, N> bufferedPinModes;
    bool pinModesDirty = true;
    BitArray<N> bufferedPullups;
    bool outputsDirty = true;
    BitArray<N> bufferedOutputs;
    BitArray<N> bufferedInputs;
    bool analogOutputsDirty = true;
    Array<uint8_t, N> bufferedAnalogOutputs;

    void writeReg(const uint8_t reg, const uint8_t value) override {
        this->driver.beginTransmission(this->address);
        this->driver.write(reg);
        this->driver.write(value);
        this->driver.endTransmission();
    }

    uint8_t readReg(const uint8_t reg) override {
        this->driver.beginTransmission(this->address);
        this->driver.write(reg);
        this->driver.endTransmission();
        this->driver.requestFrom(this->address, 1);

        uint8_t ret = 0;
        ret = this->driver.read();

        return ret;
    }

    void writeReg16(const uint8_t reg, const uint16_t value) override {
        this->driver.beginTransmission(this->address);
        this->driver.write(reg);
        this->driver.write(value >> 8);
        this->driver.write(value & 0xff);
        this->driver.endTransmission();
    }

    uint16_t readReg16(const uint8_t reg) override {
        this->driver.beginTransmission(this->address);
        this->driver.write(reg);
        this->driver.endTransmission();
        this->driver.requestFrom(this->address, 2);

        uint16_t ret = 0;
        ret = this->driver.read() << 8;
        ret |= this->driver.read();

        return ret;
    }

    void writeAnalogRegs() {
        this->driver.beginTransmission(this->address);
        this->driver.write(DIM0);

        for(size_t i = 0; i < N; ++i) {
            this->driver.write(bufferedAnalogOutputs[i]);
        }

        this->driver.endTransmission();
    }

  public:
    AW9523B(uint8_t address, pin_t interruptPin) :
            address((address & 0b11) | BASE_ADDRESS),
            interruptPin(interruptPin) {
    }

    void begin() {
        if (interruptPin != NO_PIN) {
            ExtIO::pinMode(interruptPin, INPUT_PULLUP);
            // default mode for pins is output, so disable interrupts initially
            writeReg16(INT_PORT0, 0xffff);
        }

        // Default value of pins depends on address
        uint8_t defaultState = 
                    (address & 0b01 ? 0x0f : 0x00)
                    | (address & 0b10 ? 0xf0 : 0x00);

        bufferedInputs.setByte(0, defaultState);
        bufferedInputs.setByte(1, defaultState);
        bufferedOutputs.setByte(0, defaultState);
        bufferedOutputs.setByte(1, defaultState);

        for(size_t i = 0; i < N / 2; ++i) {
            bufferedPinModes[i] = OUTPUT_OPEN_DRAIN;
        }

        for(size_t i = N / 2; i < N; ++i) {
            bufferedPinModes[i] = OUTPUT;
        }
    }

    void updateBufferedOutputs() override {
        updateBufferedPinModes();

        if (!outputsDirty || !analogOutputsDirty) {
            return;
        }

        if(outputsDirty) {
            writeReg16(OUTPUT_PORT0, 
                        (bufferedOutputs.getByte(0) << 8) 
                        | bufferedOutpus.getByte(1));

            outputsDirty = false;
        }

        if(analogOutputsDirty) {
            writeAnalogRegs();

            analogOutputsDirty = false;
        }
    }

    void updateBufferedInputs() override {
        // Only update if at least one pin is configured as input
        if (!hasInputs()) {
            return;
        }

        // Only update if a pin change interrupt happened
        if (interruptPin != NO_PIN 
            && ExtIO::digitalRead(interruptPin) == HIGH) {
            return;
        }

        uint16_t val = readReg16(INPUT_PORT0);

        bufferedInputs.setByte(0, val >> 8);
        bufferedInputs.setByte(1, val & 0xff);
    }

    void pinModeBuffered(pin_t pin, PinMode_t mode) override {
        if (mode == INPUT) {
            pinModesDirty |= bufferedPinModes[pin] != INPUT;

            bufferedPinModeBits.set(pin);
            bufferedPullups.clear(pin);
        } else if (mode == OUTPUT) {
            pinModesDirty |= bufferedPinModes[pin] != OUTPUT;

            bufferedPinModeBits.clear(pin);
        } else if (mode == OUTPUT_OPEN_DRAIN) {
            pinModesDirty |= bufferedPinModes[pin] != OUTPUT_OPEN_DRAIN;

            bufferedPinModeBits.set(pin);
            bufferedPullups.set(pin);
        } else if (mode == ANALOG) {
            pinModesDirty |= bufferedPinModes[pin] != ANALOG;

            bufferedPinModeBits.clear(pin);
        } else {
            return;
        }

        bufferedPinModes[pin] = mode;
    }

    void digitalWriteBuffered(pin_t pin, PinStatus_t status) override {
        bool boolstate = status == HIGH;

        outputsDirty |= bufferedOutputs.get(pin) != boolstate;

        bufferedOutputs.set(pin, boolstate);
    }

    PinStatus_t digitalReadBuffered(pin_t pin) override {
        return bufferedInputs.get(pin) ? HIGH : LOW;
    }

    void analogWriteBuffered(pin_t pin, analog_t val) override {
        analogOutputsDirty = bufferedAnalogOutputs[i] != val;

        bufferedAnalogOutputs[pin] = val;
    }

    analog_t analogReadBuffered(pin_t pin) override {
        return bufferedAnalogOutputs[i];
    }

    void updateBufferedPinModes() override {
        if (pinModesDirty) {
            uint16_t val = (bufferedPinModeBits.getByte(0) << 8) | bufferedPinModeBits.getByte(1);

            writeReg16(CONFIG_PORT0, val);
            writeReg16(INT_PORT0, val);

            bool gpomd = false;

            // AW9523B can only switch between open drain and push/pull for pins on port 0.
            // Port 1 pins are always in push/pull mode.
            // If any pin of port 0 is OUTPUT, we switch the whole port to push/pull mode.
            for(size_t i = 0; i < N / 2; i++) {
                if(bufferedPinModes[i] == OUTPUT) {
                    gpomd = true;
                    break;
                }
            }

            uint16_t led_mode_switch = 0x0;

            // If any pin is of mode ANALOG we turn on LED mode for the pin
            for(size_t i = 0; i < N; i++) {
                if(bufferedPinModes[i] != ANALOG) {
                    led_mode_switch |= ~(1 << i);
                }
            }

            writeReg(GCR, gpomd ? 0b10000 : 0);
            writeReg16(LED_MODE_SWITCH0, led_mode_switch);

            pinModesDirty = false;
        }
    }

    bool hasInputs() const {
        return bufferedPinModesBits.getByte(0) != 0
                || bufferedPinModesBits.getByte(1) != 0;
    }
};

END_AH_NAMESPACE