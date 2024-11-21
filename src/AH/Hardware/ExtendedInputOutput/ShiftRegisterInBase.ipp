#include "ExtendedInputOutput.hpp"
#include "ShiftRegisterInBase.hpp"

BEGIN_AH_NAMESPACE

template <uint16_t N>
ShiftRegisterInBase<N>::ShiftRegisterInBase(pin_t latchPin,
                                            BitOrder_t bitOrder)
    : latchPin(latchPin), bitOrder(bitOrder) {}

template <uint16_t N>
void ShiftRegisterInBase<N>::digitalWrite(pin_t pin, PinStatus_t val) {
    buffer.set(pin, val);
}

template <uint16_t N>
void ShiftRegisterInBase<N>::digitalWriteBuffered(pin_t pin, PinStatus_t val) {
    buffer.set(pin, val);
}

template <uint16_t N>
PinStatus_t ShiftRegisterInBase<N>::digitalRead(pin_t pin) {
    return buffer.get(pin) ? HIGH : LOW;
}

END_AH_NAMESPACE