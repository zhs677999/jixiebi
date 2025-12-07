/**Include Header Files**/
#include "sysconfig.h"

/**
 * @description: Convert float to uint
 * @param
 * @return: int
 * @note:
 */
static float clampf(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    const float span = x_max - x_min;
    if (span <= 0.0f || bits <= 0)
    {
        return 0;
    }

    const float clamped = clampf(x, x_min, x_max);
    const unsigned int scale = (1u << bits) - 1u;
    return (int)((clamped - x_min) * ((float)scale) / span);
}

/**
 * @description: Convert uint to float
 * @param
 * @return: float
 * @note:
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    const float span = x_max - x_min;
    if (span <= 0.0f || bits <= 0)
    {
        return x_min;
    }

    const float scale = (float)((1u << bits) - 1u);
    return ((float)x_int) * span / scale + x_min;
}
