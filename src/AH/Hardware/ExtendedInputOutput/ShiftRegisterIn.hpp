/* ✔ */

#pragma once

#include "ShiftRegisterInBase.hpp"

#include <AH/Arduino-Wrapper.h> // MSBFIRST

BEGIN_AH_NAMESPACE

/**
 * @brief   A class for parallel-in/serial-out shift registers, 
 *          like the 74HC165.
 * 
 * @ingroup AH_ExtIO
 */
template <uint16_t N>
class ShiftRegisterIn : public ShiftRegisterInBase<N> {
  public:
    /**
     * @brief   Create a new ShiftRegisterIn object with a shift register
     *          connected to the given pins, with a given bit order,
     *          and a given number of outputs.
     * 
     * Multiple shift registers can be cascaded by connecting the serial output
     * of the first one to the input of the second one:
     * ```
     * clockPin >───────────┬──────────────────────┬─────────── ⋯
     *              ┏━━━━━━━┷━━━━━━━┓      ┏━━━━━━━┷━━━━━━━┓ 
     *              ┃     CLK       ┃      ┃     CLK       ┃ 
     *        ⋯ >───┨ SER        QH ┠──────┨ SER        QH ┠─> dataPin
     *              ┃     sH_LD     ┃      ┃     SH_LD     ┃ 
     *              ┗━━━━━━━┯━━━━━━━┛      ┗━━━━━━━┯━━━━━━━┛ 
     * latchPin >───────────┴──────────────────────┴─────────── ⋯
     * ```
     * 
     * @param   dataPin
     *          The digital input pin connected to the serial data output (QH)
     *          of the shift register.
     * @param   clockPin
     *          The digital output pin connected to the clock input (CLK)
     *          of the shift register.
     * @param   latchPin
     *          The digital output pin connected to the latch pin (SH_LD) 
     *          of the shift register.
     * @param   bitOrder
     *          Either `MSBFIRST` (most significant bit first) or `LSBFIRST`
     *          (least significant bit first).
     */
    ShiftRegisterIn(pin_t dataPin, pin_t clockPin, pin_t latchPin,
                     BitOrder_t bitOrder = MSBFIRST);

    /**
     * @brief   Initialize the shift register.  
     *          Set the data and clock pins to output mode, 
     *          and set all shift register outputs to `LOW`.
     */
    void begin() override;

    /**
     * @brief   Write physical inputs to the state buffer.
     */
    void updateBufferedInputs() override;

  private:
    const pin_t dataPin;
    const pin_t clockPin;
};

END_AH_NAMESPACE

#include "ShiftRegisterIn.ipp"
