#pragma once

#include <AH/Settings/SettingsWrapper.hpp>

/// Helpers to determine the value of a class member called analogResolution. 
/// If one is present, it is used as the analog bit width, otherwise the machine's
/// default ADC_BITS is used.

template<typename T>
class has_analog_resolution {
    using yes = char[1];
    using no = char[2];

    template<typename C> static yes& test( decltype(&C::analogResolution));
    template<typename C> static no& test(...);

public:
    const static bool value = sizeof(test<T>(0)) == sizeof(yes);
};

template<typename T>
constexpr uint8_t analogResolution() {
    if constexpr (has_analog_resolution<T>::value) {
        return T().analogResolution;
    } else {
        return AH::ADC_BITS;
    } 
}

template<size_t N>
constexpr auto bitWidthType() {
    static_assert(N > 0 && N <= 64);

    if constexpr(N <= 8) {
        return uint8_t(0);
    } else if constexpr(N > 8 && N <= 16) {
        return uint16_t(0);
    } else if constexpr(N > 16 && N <= 32) {
        return uint32_t(0);
    } else if constexpr(N > 32 && N <= 64) {
        return uint64_t(0);
    }
}