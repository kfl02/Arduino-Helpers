#pragma once

#include <AH/Settings/Warnings.hpp>
AH_DIAGNOSTIC_WERROR() // Enable errors on warnings

#include "AH/Hardware/ExtendedInputOutput/ADDAMultiplex/ADDAMultiplex.hpp"

AH_DIAGNOSTIC_EXTERNAL_HEADER()
#include <SPI.h>
AH_DIAGNOSTIC_POP()

BEGIN_AH_NAMESPACE

/**
 * @brief   A class for MCP3xxx ADCs.
 *
 * @tparam  N
 *          The number of analog pins. Last digit of the MCP3x0x model.
 *
 * @tparam  R
 *          ADC resolution. 10 for MCP30xx, 12 for MCP32xx and 13 for MCP32xx.
 *
 * @tparam  S
 *          The size of an SPI request to read out the complete data.
 *          MCP3001/3002/3201/3301 need two bytes, the others three.
 *
 * @tparam  SPIDriver
 *          The SPI class to use. Usually, the default is fine.
 *
 * @ingroup AH_ExtIO
 */

template <uint16_t N,
          size_t R,
          size_t S,
          class SPIDriver = decltype(SPI) &>
class MCP3xxx : 
    public ADMultiplex<N, R, SPIDriver> {
  protected:
    const pin_t selectPin;

  public:
    SPISettings settings{SPI_MAX_SPEED, MSBFIRST, SPI_MODE0};

  protected:
    /**
     * @brief   Create a new MCP3xxx ADC object.
     *          Can only be called from subclasses.
     *
     * @param   spi
     *          The SPI interface to use.
     * 
     * @param   selectPin
     *          The digital output pin connected to the CS pin.
     */
    MCP3xxx(SPIDriver spi, pin_t selectPin = SS)
        : ADMultiplex<N, R, SPIDriver>(spi),
          selectPin(selectPin) {}

  public:
    using native_analog_t = ADMultiplex<N, R, SPIDriver>::native_analog_t;

    /**
     * @brief   Read the analog value of the given input.
     * 
     * @param   pin
     *          The ADC's pin number to read from.
     */
    native_analog_t analogReadNative(pin_t pin) final {
        return readADC(pin, true);
    }

    /**
     * @brief   Read the difference of two analog input.
     *
     * The pin argument is interpreted as a combination of two input pins.
     *
     * Pin | IN+ | IN- | MCP3x
     * ----|-----|-----|---------
     * 0   | 0   | 1   | 02,04,08
     * 1   | 1   | 0   | 02,04,08
     * 2   | 2   | 3   | 04,08
     * 3   | 3   | 2   | 04,08
     * 4   | 4   | 5   | 08
     * 5   | 5   | 4   | 08
     * 6   | 6   | 7   | 08
     * 7   | 7   | 6   | 08
     *
     * Note that if IN+ equal is less tan IN-, the result is 0.
     * On MCP3x01 the value of the single analog pin will be returned.
     *
     * @param   pin
     *          See above.
     */
    native_analog_t differentialRead(pin_t pin) {
        return readADC(pin, false);
    }

    /**
     * @brief   Initialize the ADC.
     *          Setup the SPI interface, set the CS pin to output mode.
     */
    void begin() final {
        ExtIO::pinMode(this->selectPin, OUTPUT);
        this->driver.begin();
    }

  protected:
    /**
     * @brief   Read an analog value from the ADC.
     *
     * @param   pin
     *          The pin(s) to read from.
     *
     * @param   single
     *          Read in single (true) or differential mode.
     */
    native_analog_t readADC(pin_t pin, bool single) {
        this->driver.beginTransaction(settings);
        ExtIO::digitalWrite(selectPin, LOW);

        uint8_t request[S];
        uint8_t result[S];

        buildRequest(pin, single, request);

        for(size_t i = 0; i < S; i++) {
            result[i] = this->driver.transfer(request[i]);
        }

        ExtIO::digitalWrite(selectPin, HIGH);
        this->driver.endTransaction();

        return fixResult(result) & ((1 << N) - 1);
    }

    /**
     * @brief   Build an SPI request for reading a specific pin.
     *
     * @param   pin
     *          The pin to read from.
     *
     * @param   single
     *          Read in single (true) or differential mode.
     *
     * @param   data
     *          An array to write the request to.
     */
    virtual void buildRequest(uint8_t pin, bool single, uint8_t data[S]) = 0;

    /**
     * @brief   Fix the SPI response to an analog_t.
     *          Especially the MCP3x01 responses have duplicated bits.
     *
     * @param   data
     *          An array containing the SPI response.
     */
    virtual native_analog_t fixResult(uint8_t data[S]) = 0;
};

/**
 * @brief   Class for MCP3001 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 1,
          size_t R = 10,
          size_t S = 2>
class MCP3001 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3001(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0;
        data[1] = 0;
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return (((data[0] & 0x1f) << 8) | data[1]) >> 3;
    }
};

/**
 * @brief   Class for MCP3002 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 2,
          size_t R = 10,
          size_t S = 2>
class MCP3002 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3002(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 
            0b01001000;
        //    │││││││└─ X
        //    ││││││└── X
        //    │││││└─── X
        //    ││││└──── MSB first
        //    │││└───── ODD/SIGN
        //    ││└────── SGL/DIFF
        //    │└─────── Start bit
        //    └──────── X

        if(single) {
            data[0] |= 
                0b00100000;
            //    │││││││└─ X
            //    ││││││└── X
            //    │││││└─── X
            //    ││││└──── MSB first
            //    │││└───── ODD/SIGN
            //    ││└────── SGL/DIFF
            //    │└─────── Start bit
            //    └──────── X
        }
        
        data[0] = pin << 4; // ODD/SIGN
        data[1] = 0;
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return ((data[0] & 0x03) << 8) | data[1];
    }
};

/**
 * @brief   Class for MCP3004 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 4,
          size_t R = 10,
          size_t S = 3>
class MCP3004 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3004(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0x01;         // Start bit
        data[1] = pin << 4;     // don't care/D1/D0

        if(single) {
            data[1] |= 0x80;    // SGL/DIFF
        }

        data[2] = 0;
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return ((data[1] & 0x03) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3008 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 8,
          size_t R = 10,
          size_t S = 3>
class MCP3008 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3008(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0x01;         // Start bit
        data[1] = pin << 4;     // D2/D1/D0

        if(single) {
            data[1] |= 0x80;    // SGL/DIFF
        }

        data[2] = 0;
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return ((data[1] & 0x03) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3201 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 1,
          size_t R = 12,
          size_t S = 2>
class MCP3201 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3201(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0;
        data[1] = 0;
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return (((data[0] & 0x1F) << 8) | data[1]) >> 1;
    }
};

/**
 * @brief   Class for MCP3202 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 2,
          size_t R = 13,
          size_t S = 3>
class MCP3202 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3202(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0x01;         // Start bit
        data[1] = 0x20          // MSB first
                  | (pin << 6); // ODD/SIGN

        if(single) {
            data[1] |= 0x80;    // SGL/DIFF
        }

        data[2] = 0;            // don't care
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return ((data[1] & 0x0F) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3204 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 4,
          size_t R = 12,
          size_t S = 3>
class MCP3204 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3204(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0x04;         // Start bit

        if(single) {
            data[0] |= 0x02;    // SGL/DIFF
        }

        data[1] = pin << 6;     // D1/D0
        data[2] = 0;            // don't care
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return ((data[1] & 0x0F) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3208 ADCs.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI),
          uint16_t N = 8,
          size_t R = 12,
          size_t S = 3>
class MCP3208 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3208(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0x04;         // Start bit

        if(single) {
            data[0] |= 0x02;    // SGL/DIFF
        }

        if (pin > 3) {
            data[0] |= 0x01;    // D2
        }

        data[1] = pin << 6;     // D1/D0
        data[2] = 0;            // don't care
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return ((data[1] & 0x0F) << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3301 ADCs.
 *          Note that the analog values are in signed representation.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 1,
          size_t R = 13,
          size_t S = 2>
class MCP3301 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3301(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0;
        data[1] = 0;
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return (data[0] & 0x10 ? 0xf000 : 0x0000)  | (data[0] << 8) | data[1];
    }
};

/**
 * @brief   Class for MCP3302 ADCs.
 *          Note that the analog values are in signed representation.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 2,
          size_t R = 13,
          size_t S = 3>
class MCP3302 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3302(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0x08;         // Start bit
        data[0] |= (pin >> 1);  // D2/D1 
        data[1] = (pin << 7);   // D0

        if(single) {
            data[0] |= 0x04;    // SGL/DIFF
        }

        data[2] = 0;            // don't care
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return (data[1] & 0x10 ? 0xf000 : 0x0000)  | (data[1] << 8) | data[2];
    }
};

/**
 * @brief   Class for MCP3304 ADCs.
 *          Note that the analog values are in signed representation.
 *
 * @tparam  SPIDriver
 *          The type of the SPI driver to use.
 */
template <class SPIDriver = decltype(SPI) &,
          uint16_t N = 4,
          size_t R = 13,
          size_t S = 3>
class MCP3304 : public MCP3xxx<N, R, S, SPIDriver> {
  public:
    MCP3304(SPIDriver spi, pin_t selectPin = SS)
        : MCP3xxx<N, R, S, SPIDriver>(spi, selectPin) {}

  protected:
    using native_analog_t = MCP3xxx<N, R, S, SPIDriver>::native_analog_t;

    void buildRequest(uint8_t pin, bool single, uint8_t data[S]) final {
        data[0] = 0x08;         // Start bit
        data[0] |= (pin >> 1);  // don't care/D1 
        data[1] = (pin << 7);   // D0

        if(single) {
            data[0] |= 0x04;    // SGL/DIFF
        }

        data[2] = 0;            // don't care
    }

    native_analog_t fixResult(uint8_t data[S]) final {
        return (data[1] & 0x10 ? 0xf000 : 0x0000)  | (data[1] << 8) | data[2];
    }
};

END_AH_NAMESPACE