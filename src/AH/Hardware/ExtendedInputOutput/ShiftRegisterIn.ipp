#include "ExtendedInputOutput.hpp"
#include "ShiftRegisterIn.hpp"

BEGIN_AH_NAMESPACE

template <uint16_t N>
ShiftRegisterIn<N>::ShiftRegisterIn(pin_t dataPin, pin_t clockPin,
                                    pin_t latchPin, BitOrder_t bitOrder)
    : ShiftRegisterInBase<N>(latchPin, bitOrder), dataPin(dataPin),
      clockPin(clockPin) {}

template <uint16_t N>
void ShiftRegisterIn<N>::begin() {
    ExtIO::pinMode(dataPin, INPUT);
    ExtIO::pinMode(clockPin, OUTPUT);
    ExtIO::pinMode(this->latchPin, OUTPUT);
    updateBufferedInputs();
}

template <uint16_t N>
void ShiftRegisterIn<N>::updateBufferedInputs() {
    ExtIO::digitalWrite(this->latchPin, LOW);
    const uint8_t bufferLength = this->buffer.getBufferLength();
    if (this->bitOrder == LSBFIRST)
        for (uint8_t i = 0; i < bufferLength; i++)
            this->buffer.setByte(i, ExtIO::shiftIn(dataPin, clockPin, LSBFIRST);
    else
        for (int8_t i = bufferLength - 1; i >= 0; i--)
            this->buffer.setByte(i, ExtIO::shiftIn(dataPin, clockPin, MSBFIRST);

    ExtIO::digitalWrite(this->latchPin, HIGH);
}

END_AH_NAMESPACE