#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"

arm_status arm_rfft_fast_init_f32(arm_rfft_fast_instance_f32 *S, uint16_t fftLen) {
    arm_status status;

    if (fftLen == 32 || fftLen == 64 || fftLen == 128 || fftLen == 256 ||
        fftLen == 512 || fftLen == 1024 || fftLen == 2048 || fftLen == 4096) {

        S->fftLenRFFT = fftLen;
        S->pTwiddleRFFT = arm_cfft_sR_f32_len4096.pTwiddle; // Replace appropriately

        status = arm_cfft_init_f32(&S->Sint, fftLen / 2);
    } else {
        status = ARM_MATH_ARGUMENT_ERROR;
    }

    return status;
}
