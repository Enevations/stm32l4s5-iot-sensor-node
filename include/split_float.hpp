#pragma once

#include <cstdint>

/**
 * @brief Components of a signed float rendered with two fractional digits.
 *
 * Fraction digits are rounded, not truncated, so 9.999 renders as "10.00";
 * the carry from a rounded fraction of 100 is applied to whole. The sign is
 * kept separate from whole so values in (-1, 0) render as "-0.40" instead of
 * "0.40".
 */
struct SplitFloat {
    /** @brief True when the original value was negative. */
    bool negative{false};
    /** @brief Integer part of the absolute value. */
    int whole{0};
    /** @brief Fraction part in hundredths, always in [0, 99]. */
    int frac{0};
};

/**
 * @brief Splits a signed float into sign, integer part, and rounded hundredths.
 *
 * Header-only and constexpr so host-side unit tests can exercise the logic
 * without a firmware build; avoids the float formatter in std::snprintf,
 * which would pull the heavyweight newlib float-printf path into the firmware.
 *
 * @param value Value to split, rounded to two fractional digits.
 * @return Components suitable for "%s%d.%02d"-style formatting; the sign is a
 *         plain "-" prefix, so tiny negatives such as -0.001 render "-0.00".
 */
constexpr SplitFloat split_float(float value) noexcept {
    SplitFloat out;
    if (value < 0.0f) {
        out.negative = true;
        value = -value;
    }
    out.whole = static_cast<int>(value);
    const int rounded_frac =
        static_cast<int>((value - static_cast<float>(out.whole)) * 100.0f + 0.5f);
    if (rounded_frac >= 100) {
        out.whole += 1;
    } else {
        out.frac = rounded_frac;
    }
    return out;
}
