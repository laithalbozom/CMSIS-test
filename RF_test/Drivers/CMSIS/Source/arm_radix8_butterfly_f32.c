#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"


void arm_radix8_butterfly_f32(
    float32_t *pSrc,
    uint16_t fftLen,
    const float32_t *pCoef,
    uint16_t twidCoefModifier) {

    uint32_t i, j, k;
    float32_t xr, xi, yr, yi, tr, ti;

    for (i = 0; i < fftLen; i += 8) {
        for (j = 0; j < 8; j++) {
            k = i + j;
            xr = pSrc[2 * k];
            xi = pSrc[2 * k + 1];
            yr = pCoef[2 * j * twidCoefModifier];
            yi = pCoef[2 * j * twidCoefModifier + 1];

            // Perform radix-8 butterfly operations
            tr = xr * yr - xi * yi;
            ti = xi * yr + xr * yi;

            pSrc[2 * k] = tr;
            pSrc[2 * k + 1] = ti;
        }
    }
}
