#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include <AH/Containers/BitArray.hpp>
#include "AH/Hardware/ExtendedInputOutput/DigitalMultiplex/DigitalMultiplex.hpp"

AH_DIAGNOSTIC_EXTERNAL_HEADER()
#include <Wire.h>
#include <SPI.h>
AH_DIAGNOSTIC_POP()

BEGIN_AH_NAMESPACE

/**
 * @brief   Class for MCP23xxx I²C/SPI I/O expanders.
 *
 * @tparam  Driver
 *          The type of the I²C/SPI driver to use.
 *
 * @tparam  N
 *          The number of pins
 */
template <class Driver, uint16_t N>
class MCP23xxx : 
    public virtual DigitalMultiplex<Driver, N> {
  protected:
    enum MCP230xxRegAddr {
        IODIR = 0x00,
        IODIRA = 0x00,
        IODIRB = 0x01,
        IPOL = 0x01,
        IPOLA = 0x02,
        IPOLB = 0x03,
        GPINTEN = 0x02,
        GPINTENA = 0x04,
        GPINTENB = 0x05,
        DEFVAL = 0x03,
        DEFVALA = 0x06,
        DEFVALB = 0x07,
        INTCON = 0x04,
        INTCONA = 0x08,
        INTCONB = 0x09,
        IOCON = 0x05,
        IOCONA = 0x0A,
        IOCONB = 0x0B,
        GPPU = 0x06,
        GPPUA = 0x0C,
        GPPUB = 0x0D,
        INTF = 0x07,
        INTFA = 0x0E,
        INTFB = 0x0F,
        INTCAP = 0x08,
        INTCAPA = 0x10,
        INTCAPB = 0x11,
        GPIO = 0x09,
        GPIOA = 0x12,
        GPIOB = 0x13,
        OLAT = 0x0A,
        OLATA = 0x14,
        OLATB = 0x15,
    };

    uint8_t address;
    pin_t interruptPin;

    bool pinModesDirty = true;
    BitArray<N> bufferedPinModes;
    bool pullupsDirty = true;
    BitArray<N> bufferedPullups;
    bool outputsDirty = true;
    BitArray<N> bufferedOutputs;
    BitArray<N> bufferedInputs;

    MCP23xxx(uint8_t address, pin_t interruptPin) :
             address(address),
             interruptPin(interruptPin) {}

    void init() {
        if (interruptPin != NO_PIN) {
            ExtIO::pinMode(interruptPin, INPUT_PULLUP);
        }
    }

    virtual void writeReg(const uint8_t reg, const uint8_t value) = 0;
    virtual uint8_t readReg(const uint8_t reg) = 0;
    virtual void writeReg16(const uint8_t reg, const uint16_t value) = 0;
    virtual uint16_t readReg16(const uint8_t reg) = 0;

    virtual void begin() = 0;

    void pinModeBuffered(pin_t pin, PinMode_t mode) override {
        if (mode == INPUT) {
            pinModesDirty |= bufferedPinModes.get(pin) == 0;
            pullupsDirty |= bufferedPullups.get(pin) == 1;
            bufferedPinModes.set(pin);
            bufferedPullups.clear(pin);
        } else if (mode == OUTPUT) {
            pinModesDirty |= bufferedPinModes.get(pin) == 1;
            bufferedPinModes.clear(pin);
        } else if (mode == INPUT_PULLUP) {
            pinModesDirty |= bufferedPinModes.get(pin) == 0;
            pullupsDirty |= bufferedPullups.get(pin) == 0;
            bufferedPinModes.set(pin);
            bufferedPullups.set(pin);
        }
    }

    void digitalWriteBuffered(pin_t pin, PinStatus_t status) override {
        bool boolstate = status == HIGH;
        outputsDirty |= bufferedOutputs.get(pin) != boolstate;
        bufferedOutputs.set(pin, boolstate);
    }

    PinStatus_t digitalReadBuffered(pin_t pin) override {
        return bufferedInputs.get(pin) ? HIGH : LOW;
    }

    analog_t analogReadBuffered(pin_t pin) override {
        return bufferedInputs.get(pin) ? (1 << ADC_BITS) - 1 : 0;
    }

    void analogWriteBuffered(pin_t pin, analog_t value) override {
        digitalWriteBuffered(pin, value >= (1 << (ADC_BITS - 1)) ? HIGH : LOW);
    }

    virtual bool hasInputs() const = 0;
};

/**
 * @brief   Class for MCP230xx I²C I/O expanders.
 *          Provides methods for reading/writing 8 and 16 bit registers.
 *
 * @tparam  Driver
 *          The type of the I²C driver to use.
 *
 * @tparam  N
 *          The number of pins
 */
template <class Driver, uint16_t N>
class MCP230xx : 
    public virtual MCP23xxx<Driver, N> {
  private:
      const uint8_t BASE_ADDRESS = 0x20;

  protected:
    void writeReg(const uint8_t reg, const uint8_t value) override {
        uint8_t address = this->address | BASE_ADDRESS;

        this->driver.beginTransmission(address);
        this->driver.write(reg);
        this->driver.write(value);
        this->driver.endTransmission();
    }

    uint8_t readReg(const uint8_t reg) override {
        uint8_t address = this->address | BASE_ADDRESS;

        this->driver.beginTransmission(address);
        this->driver.write(reg);
        this->driver.endTransmission();
        this->driver.requestFrom(address, 1);

        uint8_t ret = 0;
        ret = this->driver.read();

        return ret;
    }

    void writeReg16(const uint8_t reg, const uint16_t value) override {
        uint8_t address = this->address | BASE_ADDRESS;

        this->driver.beginTransmission(address);
        this->driver.write(reg);
        this->driver.write(value >> 8);
        this->driver.write(value & 0xff);
        this->driver.endTransmission();
    }

    uint16_t readReg16(const uint8_t reg) override {
        uint8_t address = this->address | BASE_ADDRESS;

        this->driver.beginTransmission(address);
        this->driver.write(reg);
        this->driver.endTransmission();
        this->driver.requestFrom(address, 2);

        uint16_t ret = 0;
        ret = this->driver.read() << 8;
        ret |= this->driver.read();

        return ret;
    }
};

/**
 * @brief   Class for MCP23Sxx SPI I/O expanders.
 *          Provides methods for reading/writing 8 and 16 bit registers.
 *
 * @tparam  Driver
 *          The type of the SPI driver to use.
 *
 * @tparam  N
 *          The number of pins
 */
template <class Driver, uint16_t N>
class MCP23Sxx : 
    public virtual MCP23xxx<Driver, N> {
  private:
    static const uint8_t WRITE_REG = 0x40;
    static const uint8_t READ_REG = 0x41;

    pin_t selectPin;
    SPISettings spiSettings{SPI_MAX_SPEED, MSBFIRST, SPI_MODE0};

  protected:
    MCP23Sxx(pin_t selectPin) : 
             selectPin(selectPin) {}

    void begin() override {
        ExtIO::pinMode(selectPin, OUTPUT);
        ExtIO::digitalWrite(selectPin, HIGH);
    }

    void writeReg(const uint8_t reg, const uint8_t value) override {
        ExtIO::digitalWrite(selectPin, LOW);
        this->driver.beginTransaction(spiSettings);
        this->driver.write(WRITE_REG | (this->address << 1));
        this->driver.write(reg);
        this->driver.write(value);
        this->driver.endTransaction();
        ExtIO::digitalWrite(selectPin, HIGH);
    }

    uint8_t readReg(const uint8_t reg) override {
        ExtIO::digitalWrite(selectPin, LOW);
        this->driver.beginTransaction(spiSettings);
        this->driver.write(READ_REG | (this->address << 1));
        this->driver.write(reg);

        uint8_t ret = 0;

        ret = this->driver.transfer(0xff);

        this->driver.endTransaction();
        ExtIO::digitalWrite(selectPin, HIGH);

        return ret;
    }

    void writeReg16(const uint8_t reg, const uint16_t value) override {
        ExtIO::digitalWrite(selectPin, LOW);
        this->driver.beginTransaction(spiSettings);
        this->driver.write(WRITE_REG | (this->address << 1));
        this->driver.write(reg);
        this->driver.write16(value
        this->driver.endTransaction();
        ExtIO::digitalWrite(selectPin, HIGH);
    }

    uint16_t readReg16(const uint8_t reg) override {
        ExtIO::digitalWrite(selectPin, LOW);
        this->driver.beginTransaction(spiSettings);
        this->driver.write(READ_REG | (this->address << 1));
        this->driver.write(reg);

        uint16_t ret = 0;

        ret = this->driver.transfer16(0xffff);

        this->driver.endTransaction();
        ExtIO::digitalWrite(selectPin, HIGH);

        return ret;
    }
};

/**
 * @brief   Class for MCP23x08 8 pin I/O expanders.
 *
 * @tparam  Driver
 *          The type of the I²C/SPI driver to use.
 *
 * @tparam  N
 *          The number of pins
 */
template <class Driver, const uint16_t N = 8>
class MCP23x08 : 
    public virtual MCP23xxx<Driver, N> {
  protected:
    MCP23x08() {
        // Input mode by default
        this->bufferedPinModes.setByte(0, 0xFF);
    };

  public:
    void updateBufferedOutputs() override {
        updateBufferedPinModes();

        if (!this->outputsDirty)
            return;

        this->writeReg(MCP23xxx<Driver, N>::GPIO, this->bufferedOutputs.getByte(0));

        this->outputsDirty = false;
    }

    void updateBufferedInputs() override {
        // Only update if at least one pin is configured as input
        if (!hasInputs())
            return;
        // Only update if a pin change interrupt happened
        if (this->interruptPin != NO_PIN && ExtIO::digitalRead(this->interruptPin) == HIGH)
            return;

        this->bufferedInputs.setByte(0, this->readReg(MCP23xxx<Driver, N>::GPIO));
    }

    void updateBufferedPinModes() override {
        if (this->pinModesDirty) {
            this->writeReg(MCP23xxx<Driver, N>::IODIR, this->bufferedPinModes.getByte(0));
            this->writeReg(MCP23xxx<Driver, N>::GPINTEN, this->bufferedPinModes.getByte(0));

            this->pinModesDirty = false;
        }
        if (this->pullupsDirty) {
            this->writeReg(MCP23xxx<Driver, N>::GPPU, this->bufferedPullups.getByte(0));

            this->pullupsDirty = false;
        }
    }

    bool hasInputs() const override {
        return this->bufferedPinModes.getByte(0) != 0;
    }
};

/**
 * @brief   Class for MCP23x17 16 pin I/O expanders.
 *
 * @tparam  Driver
 *          The type of the I²C/SPI driver to use.
 *
 * @tparam  N
 *          The number of pins
 */
template <class Driver, const uint16_t N = 16>
class MCP23x17 : 
    public virtual MCP23xxx<Driver, N> {
  protected:
    MCP23x17() {
        // Input mode by default
        this->bufferedPinModes.setByte(0, 0xFF);
        this->bufferedPinModes.setByte(1, 0xFF);
    };

  public:
    void updateBufferedOutputs() override {
        updateBufferedPinModes();

        if (!this->outputsDirty)
            return;

        this->writeReg16(MCP23xxx<Driver, N>::GPIOA, (this->bufferedOutputs.getByte(0) << 8) | this->bufferedOutputs.getByte(1));

        this->outputsDirty = false;
    }

    void updateBufferedInputs() override {
        // Only update if at least one pin is configured as input
        if (!hasInputs())
            return;
        // Only update if a pin change interrupt happened
        if (this->interruptPin != NO_PIN && ExtIO::digitalRead(this->interruptPin) == HIGH)
            return;

        uint16_t val;
        
        if (this->interruptPin != NO_PIN) {
            // read from INTCAP instead of GPIO to work around GPA7/GPB7 input bug
            // see: https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library/issues/96#issuecomment-1644464978
            val = this->readReg16(MCP23xxx<Driver, N>::INTCAPA);
        } else {
            val = this->readReg16(MCP23xxx<Driver, N>::GPIOA);
        }

        this->bufferedInputs.setByte(0, val >> 8);
        this->bufferedInputs.setByte(1, val & 0xff);
    }

    void updateBufferedPinModes() override {
        if (this->pinModesDirty) {
            this->writeReg16(MCP23xxx<Driver, N>::IODIRA, (this->bufferedPinModes.getByte(0) << 8) | this->bufferedPinModes.getByte(1));
            this->writeReg16(MCP23xxx<Driver, N>::GPINTENA, (this->bufferedPinModes.getByte(0) << 8) | this->bufferedPinModes.getByte(1));

            this->pinModesDirty = false;
        }
        if (this->pullupsDirty) {
            this->writeReg16(MCP23xxx<Driver, N>::GPPUA, (this->bufferedPullups.getByte(0) << 8) | this->bufferedPullups.getByte(1));

            this->pullupsDirty = false;
        }
    }

    bool hasInputs() const override {
        return this->bufferedPinModes.getByte(0) != 0
            || this->bufferedPinModes.getByte(1) != 0;
    }
};

/**
 * @brief   Class for MCP23S08 SPI I/O expanders.
 *
 * @tparam  Driver
 *          The type of the SPI driver to use.
 *
 * @ingroup AH_ExtIO
 */
template <class Driver = decltype(SPI), const uint16_t N = 8>
class MCP23S08 : 
    public virtual MCP23x08<Driver>, 
    public virtual MCP23Sxx<Driver, N> {
  public:
    MCP23S08(Driver driver, pin_t selectPin, uint8_t address = 0, pin_t interruptPin = NO_PIN) : 
             DigitalMultiplex<Driver, N>(driver),
             MCP23xxx<Driver, N>(address & 0b11, interruptPin),
             MCP23Sxx<Driver, N>(selectPin) {}

    void begin() override {
        this->init();
        MCP23Sxx<Driver, N>::begin();

        uint8_t val = 
            0b00000000;
        //    │││││││└─ unimplemented
        //    ││││││└── INTPOL = Active-low
        //    │││││└─── ODR    = Active driver output
        //    │││││              (INTPOL bit sets the polarity)
        //    ││││└──── HAEN   = Disables the MCP23S08 address pins
        //    │││└───── DISSLW = Slew rate enabled
        //    ││└────── SEQOP  = Sequential operation enabled,
        //    ││                 address pointer increments
        //    │└─────── unimplemented
        //    └──────── unimplemented

        uint8_t tmpAddr = this->address;

        // HAEN is guaranteed to be turned off, default address is 0 now
        // MCP23S17 datasheet:
        // If disabled (HAEN = 0), the device’s hardware address is A2 = A1 = A0 = 0.
        // The MCP23S08 datasheet does not state the same for A1, A0, so we're assuming here
        if(this->address != 0) {
            val |= 0b00001000;
            this->address = 0;  // write to default address to enable addressing
        }

        this->writeReg(MCP23xxx<Driver, N>::IOCON, val);

        // HAEN is on if address != 0
        this->address = tmpAddr;
    }
};

/**
 * @brief   Class for MCP23008 I²C I/O expanders.
 *
 * @tparam  Driver
 *          The type of the I²C driver to use.
 *
 * @ingroup AH_ExtIO
 */
template <class Driver = decltype(Wire), const uint16_t N = 8>
class MCP23008 : 
    public virtual MCP23x08<Driver>, 
    public virtual MCP230xx<Driver, N> {
  public:
    MCP23008(Driver driver, uint8_t address, pin_t interruptPin = NO_PIN) : 
             DigitalMultiplex<Driver, N>(driver),
             MCP23xxx<Driver, N>(address & 0b111, interruptPin) {}

    void begin() override {
        this->init();

        uint8_t val = 
            0b00000000;
        //    │││││││└─ unimplemented
        //    ││││││└── INTPOL = Active-low
        //    │││││└─── ODR    = Active driver output
        //    │││││              (INTPOL bit sets the polarity)
        //    ││││└──── HAEN   = Disables the MCP23S08 address pins
        //    │││└───── DISSLW = Slew rate enabled
        //    ││└────── SEQOP  = Sequential operation enabled,
        //    ││                 address pointer increments
        //    │└─────── unimplemented
        //    └──────── unimplemented

        this->writeReg(MCP23xxx<Driver, N>::IOCON, val);
    }
};

/**
 * @brief   Class for MCP23S17 SPI I/O expanders.
 * 
 * @tparam  Driver 
 *          The type of the SPI driver to use.
 *
 * @ingroup AH_ExtIO
 */
template <class Driver = decltype(SPI), const uint16_t N = 16>
class MCP23S17 : 
    public virtual MCP23x17<Driver>, 
    public virtual MCP23Sxx<Driver, N> {
  public:
    MCP23S17(Driver driver, pin_t selectPin, uint8_t address = 0, pin_t interruptPin = NO_PIN) : 
             DigitalMultiplex<Driver, N>(driver),
             MCP23xxx<Driver, N>(address & 0b111, interruptPin),
             MCP23Sxx<Driver, N>(selectPin) {}

    void begin() override {
        this->init();
        MCP23Sxx<Driver, N>::begin();

        // First make sure to set BANK = 1 on MSCP23017 in register 0x0B,
        // which is not a valid register when BANK is already 1
        uint8_t val = 
            0b11000000;
        //    │││││││└─ unimplemented
        //    ││││││└── INTPOL = Active-low
        //    │││││└─── ODR    = Active driver output
        //    │││││              (INTPOL bit sets the polarity)
        //    ││││└──── HAEN   = Disables the MCP23S17 address pins
        //    │││└───── DISSLW = Slew rate enabled
        //    ││└────── SEQOP  = Sequential operation enabled,
        //    ││                 address pointer increments
        //    │└─────── MIRROR = The INT pins are internally connected
        //    └──────── BANK   = The registers are in different banks
        //                       (addresses are segregated)

        this->writeReg(MCP23xxx<Driver, N>::IOCONB, val);

        // IOCON is guaranteed to be at 0x05 with the previous write
        // to 0x0B, so now switch to BANK = 0 again 
        // (register are in the same bank, addresses are sequential)
        val &= 0b01111111;

        uint8_t tmpAddr = this->address;

        // HAEN is guaranteed to be turned off, default address is 0 now
        // MCP23S17 datasheet:
        // If disabled (HAEN = 0), the device’s hardware address is A2 = A1 = A0 = 0.
        if(this->address != 0) {
            val |= 0b00001000;
            this->address = 0;  // write to default address to enable addressing
        }

        this->writeReg(MCP23xxx<Driver, N>::IOCON, val);

        // Bank is now 0, HAEN is on if address != 0
        this->address = tmpAddr;
    }
};

/**
 * @brief   Class for MCP23017 I²C I/O expanders.
 * 
 * @tparam  Driver 
 *          The type of the I²C driver to use.
 *
 * @ingroup AH_ExtIO
 */
template <class Driver = decltype(Wire), const uint16_t N = 16>
class MCP23017 : 
    public virtual MCP23x17<Driver>, 
    public virtual MCP230xx<Driver, N> {
  public:
    MCP23017() = default;
    MCP23017(Driver driver, uint8_t address, pin_t interruptPin = NO_PIN) : 
             DigitalMultiplex<Driver, N>(driver),
             MCP23xxx<Driver, N>(address & 0b111, interruptPin) {}

    void begin() override {
        this->init();
        // First make sure to set BANK = 1 on MSCP23017 in register 0x0B,
        // which is not a valid register when BANK is already 1
        uint8_t val = 
            0b11000000;
        //    │││││││└─ unimplemented
        //    ││││││└── INTPOL = Active-low
        //    │││││└─── ODR    = Active driver output
        //    │││││              (INTPOL bit sets the polarity)
        //    ││││└──── HAEN   = Disables the MCP23S17 address pins
        //    │││└───── DISSLW = Slew rate enabled
        //    ││└────── SEQOP  = Sequential operation enabled,
        //    ││                 address pointer increments
        //    │└─────── MIRROR = The INT pins are internally connected
        //    └──────── BANK   = The registers are in different banks
        //                       (addresses are segregated)

        this->writeReg(MCP23xxx<Driver, N>::IOCONB, val);

        // IOCON is guaranteed to be at 0x05 with the previous write
        // to 0x0B, so now switch to BANK = 0 again 
        // (register are in the same bank, addresses are sequential)
        val &= 0b01111111;

        this->writeReg(MCP23xxx<Driver, N>::IOCON, val);
    }
};

END_AH_NAMESPACE