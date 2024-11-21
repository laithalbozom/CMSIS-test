/* ----------------------------------------------------------------------
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.
*
* $Date:        19. March 2015
* $Revision: 	V.1.4.5
*
* Project: 	    CMSIS DSP Library
* Title:	    arm_rfft_f32.c
*
* Description:	RFFT & RIFFT Floating point process function
*
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
* -------------------------------------------------------------------- */

#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"


// Updated function prototypes with const qualifier
void stage_rfft_f32(const arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);
void merge_rfft_f32(const arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut);

// Updated `stage_rfft_f32` function
void stage_rfft_f32(const arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut) {
   uint32_t k;  // Loop Counter
   float32_t twR, twI;  // RFFT Twiddle coefficients
   const float32_t *pCoeff = S->pTwiddleRFFT;  // Points to RFFT Twiddle factors
   float32_t *pA = p;  // increasing pointer
   float32_t *pB = p;  // decreasing pointer
   float32_t xAR, xAI, xBR, xBI;  // temporary variables
   float32_t t1a, t1b;  // temporary variables
   float32_t p0, p1, p2, p3;  // temporary variables

   k = (S->Sint).fftLen - 1;

   // Pack first and last sample of the frequency domain together
   xBR = pB[0];
   xBI = pB[1];
   xAR = pA[0];
   xAI = pA[1];

   twR = *pCoeff++;
   twI = *pCoeff++;

   // U1 = XA(1) + XB(1); % It is real
   t1a = xBR + xAR;

   // U2 = XB(1) - XA(1); % It is imaginary
   t1b = xBI + xAI;

   *pOut++ = 0.5f * (t1a + t1b);
   *pOut++ = 0.5f * (t1a - t1b);

   pB = p + 2 * k;
   pA += 2;

   do {
      xBI = pB[1];
      xBR = pB[0];
      xAR = pA[0];
      xAI = pA[1];

      twR = *pCoeff++;
      twI = *pCoeff++;

      t1a = xBR - xAR;
      t1b = xBI + xAI;

      p0 = twR * t1a;
      p1 = twI * t1a;
      p2 = twR * t1b;
      p3 = twI * t1b;

      *pOut++ = 0.5f * (xAR + xBR + p0 + p3);
      *pOut++ = 0.5f * (xAI - xBI + p1 - p2);

      pA += 2;
      pB -= 2;
      k--;
   } while (k > 0u);
}

// Updated `merge_rfft_f32` function
void merge_rfft_f32(const arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut) {
   uint32_t k;  // Loop Counter
   float32_t twR, twI;  // RFFT Twiddle coefficients
   const float32_t *pCoeff = S->pTwiddleRFFT;  // Points to RFFT Twiddle factors
   float32_t *pA = p;  // increasing pointer
   float32_t *pB = p;  // decreasing pointer
   float32_t xAR, xAI, xBR, xBI;  // temporary variables
   float32_t t1a, t1b, r, s, t, u;  // temporary variables

   k = (S->Sint).fftLen - 1;

   xAR = pA[0];
   xAI = pA[1];

   pCoeff += 2;

   *pOut++ = 0.5f * (xAR + xAI);
   *pOut++ = 0.5f * (xAR - xAI);

   pB = p + 2 * k;
   pA += 2;

   while (k > 0u) {
      xBI = pB[1];
      xBR = pB[0];
      xAR = pA[0];
      xAI = pA[1];

      twR = *pCoeff++;
      twI = *pCoeff++;

      t1a = xAR - xBR;
      t1b = xAI + xBI;

      r = twR * t1a;
      s = twI * t1b;
      t = twI * t1a;
      u = twR * t1b;

      *pOut++ = 0.5f * (xAR + xBR - r - s);
      *pOut++ = 0.5f * (xAI - xBI + t - u);

      pA += 2;
      pB -= 2;
      k--;
   }
}

// Updated `arm_rfft_fast_f32` function
void arm_rfft_fast_f32(const arm_rfft_fast_instance_f32 *S, float32_t *p, float32_t *pOut, uint8_t ifftFlag) {
    arm_cfft_instance_f32 *Sint = (arm_cfft_instance_f32 *)&(S->Sint);  // Cast away const
    Sint->fftLen = S->fftLenRFFT / 2;  // Now you can modify fftLen

    if (ifftFlag) {
        // Real FFT compression
        merge_rfft_f32(S, p, pOut);

        // Complex radix-4 IFFT process
        arm_cfft_f32(Sint, pOut, ifftFlag, 1);
    } else {
        // Calculation of RFFT of input
        arm_cfft_f32(Sint, p, ifftFlag, 1);

        // Real FFT extraction
        stage_rfft_f32(S, p, pOut);
    }
}


