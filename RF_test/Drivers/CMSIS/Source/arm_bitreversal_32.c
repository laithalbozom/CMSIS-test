#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"


void arm_bitreversal_32(uint32_t *pSrc, const uint16_t bitRevLen, const uint16_t *pBitRevTable) {
    uint16_t i, j;
    uint32_t temp;

    for (i = 0; i < bitRevLen; i++) {
        j = pBitRevTable[i];
        if (j > i) {
            // Swap elements
            temp = pSrc[i];
            pSrc[i] = pSrc[j];
            pSrc[j] = temp;
        }
    }
}
