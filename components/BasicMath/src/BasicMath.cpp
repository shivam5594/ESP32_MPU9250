/*
 * BasicMath.cpp
 *
 *  Created on: Aug 15, 2020
 *      Author: shivamchauhan
 */

#include "stdio.h"
#include "BasicMath.h"
#include <math.h>

/**
 * Vector Addition
 *
 * Element-by-element addition of two vectors.
 *
 *     pDst[n] = pSrcA[n] + pSrcB[n],   0 <= n < blockSize.
 *
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @brief Floating-point vector addition.
 * @param[in]       *pSrcA points to the first input vector
 * @param[in]       *pSrcB points to the second input vector
 * @param[out]      *pDst points to the output vector
 * @param[in]       blockSize number of samples in each vector
 * @return 			none.
 */

void add_f32(float * pSrcA, float * pSrcB, float * pDst, uint32_t blockSize)
{

  while(blockSize > 0u)
  {
    /* C = A + B */
    /* Add and then store the results in the destination buffer. */
    *pDst++ = (*pSrcA++) + (*pSrcB++);

    /* Decrement the loop counter */
    blockSize--;
  }
}

/**
  @defgroup BasicSub Vector Subtraction
  Element-by-element subtraction of two vectors.
      pDst[n] = pSrcA[n] - pSrcB[n],   0 <= n < blockSize.
  There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
  @brief Floating-point vector subtraction.
  @param[in]     pSrcA      points to the first input vector
  @param[in]     pSrcB      points to the second input vector
  @param[out]    pDst       points to the output vector
  @param[in]     blockSize  number of samples in each vector
  @return        none
 */

void sub_f32( const float * pSrcA, const float * pSrcB, float * pDst, uint32_t blockSize)
{
        uint32_t blkCnt;                               /* Loop counter */

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = blockSize >> 2U;

  while (blkCnt > 0U)
  {
    /* C = A - B */

    /* Subtract and store result in destination buffer. */
    *pDst++ = (*pSrcA++) - (*pSrcB++);

    *pDst++ = (*pSrcA++) - (*pSrcB++);

    *pDst++ = (*pSrcA++) - (*pSrcB++);

    *pDst++ = (*pSrcA++) - (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = blockSize % 0x4U;


  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

  while (blkCnt > 0U)
  {
    /* C = A - B */

    /* Subtract and store result in destination buffer. */
    *pDst++ = (*pSrcA++) - (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

}

/**
 * Vector Scale
 *
 * Multiply a vector by a scalar value.  For floating-point data, the algorithm used is:
 *
 *     pDst[n] = pSrc[n] * scale,   0 <= n < blockSize.
 *
 * In the fixed-point Q7, Q15, and Q31 functions, scale is represented by
 * a fractional multiplication caleFract and an arithmetic shift shift.
 * The shift allows the gain of the scaling operation to exceed 1.0.
 * The algorithm used with fixed-point data is:
 *
 *     pDst[n] = (pSrc[n] * scaleFract) << shift,   0 <= n < blockSize.
 *
 * The overall scale factor applied to the fixed-point data is
 *
 *     scale = scaleFract * 2^shift.
 *
 * The functions support in-place computation allowing the source and destination
 * pointers to reference the same memory buffer.
 */

/**
 * @brief Multiplies a floating-point vector by a scalar.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       scale scale factor to be applied
 * @param[out]      *pDst points to the output vector
 * @param[in]       blockSize number of samples in the vector
 * @return 			none.
 */

void scale_f32(const float *pSrc, float scale, float *pDst, uint32_t blockSize)
{

	while(blockSize > 0u)
	{
	/* C = A * scale */
	/* Scale the input and then store the result in the destination buffer. */
	*pDst++ = (*pSrc++) * scale;

	/* Decrement the loop counter */
	blockSize--;
	}
}

/**
 * Dot Product
 *
 * Computes the dot product of two vectors.
 * The vectors are multiplied element-by-element and then summed.
 *
 *     sum = pSrcA[0]*pSrcB[0] + pSrcA[1]*pSrcB[1] + ... + pSrcA[blockSize-1]*pSrcB[blockSize-1]
 *
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @brief Dot product of floating-point vectors.
 * @param[in]       *pSrcA points to the first input vector
 * @param[in]       *pSrcB points to the second input vector
 * @param[in]       blockSize number of samples in each vector
 * @param[out]      *result output result returned here
 * @return 			none.
 */


void dot_prod_f32 (const float *pSrcA, const float *pSrcB, uint32_t blockSize, float *result)
{
	float sum = 0.0f;                          /* Temporary result storage */
	uint32_t blkCnt;                               /* loop counter */

	/* Initialize blkCnt with number of samples */
	blkCnt = blockSize;

	while(blkCnt > 0u)
	{
	/* C = A[0]* B[0] + A[1]* B[1] + A[2]* B[2] + .....+ A[blockSize-1]* B[blockSize-1] */
	/* Calculate dot product and then store the result in a temporary buffer. */
	sum += (*pSrcA++) * (*pSrcB++);

	/* Decrement the loop counter */
	blkCnt--;
	}
	/* Store the result back in the destination buffer */
	*result = sum;
}

/*
 * Vector Multiplication
 * Element-by-element multiplication of two vectors.
 * pDst[n] = pSrcA[n] * pSrcB[n],   0 <= n < blockSize.
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @brief Floating-point vector multiplication.
 * @param[in]     pSrcA      points to the first input vector.
 * @param[in]     pSrcB      points to the second input vector.
 * @param[out]    pDst       points to the output vector.
 * @param[in]     blockSize  number of samples in each vector.
 * @return        none
*/

void mult_f32( const float * pSrcA, const float * pSrcB, float * pDst, uint32_t blockSize)
{
    uint32_t blkCnt;                               /* Loop counter */

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = blockSize >> 2U;

  while (blkCnt > 0U)
  {
    /* C = A * B */

    /* Multiply inputs and store result in destination buffer. */
    *pDst++ = (*pSrcA++) * (*pSrcB++);

    *pDst++ = (*pSrcA++) * (*pSrcB++);

    *pDst++ = (*pSrcA++) * (*pSrcB++);

    *pDst++ = (*pSrcA++) * (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = blockSize % 0x4U;

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

  while (blkCnt > 0U)
  {
    /* C = A * B */

    /* Multiply input and store result in destination buffer. */
    *pDst++ = (*pSrcA++) * (*pSrcB++);

    /* Decrement loop counter */
    blkCnt--;
  }

}

/**
 * Vector Copy
 *
 * Copies sample by sample from source vector to destination vector.
 *
 * 	pDst[n] = pSrc[n];   0 <= n < blockSize.
 *
 * There are separate functions for floating point, Q31, Q15, and Q7 data types.
 */

/**
 * @brief Copies the elements of a floating-point vector.
 * @param[in]       *pSrc points to input vector
 * @param[out]      *pDst points to output vector
 * @param[in]       blockSize length of the input vector
 * @return none.
 *
 */

void copy_f32 (const float *pSrc, float *pDst, uint32_t blockSize)
{
	  uint32_t blkCnt;                               /* loop counter */

	#ifndef CM0_FAMILY

	  /* Run the below code for Cortex-M4 and Cortex-M3 */
	  float in1, in2, in3, in4;

	  /*loop Unrolling */
	  blkCnt = blockSize >> 2u;

	  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	   ** a second loop below computes the remaining 1 to 3 samples. */
	  while(blkCnt > 0u)
	  {
	    /* C = A */
	    /* Copy and then store the results in the destination buffer */
	    in1 = *pSrc++;
	    in2 = *pSrc++;
	    in3 = *pSrc++;
	    in4 = *pSrc++;

	    *pDst++ = in1;
	    *pDst++ = in2;
	    *pDst++ = in3;
	    *pDst++ = in4;

	    /* Decrement the loop counter */
	    blkCnt--;
	  }

	  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
	   ** No loop unrolling is used. */
	  blkCnt = blockSize % 0x4u;

	#else

	  /* Run the below code for Cortex-M0 */

	  /* Loop over blockSize number of values */
	  blkCnt = blockSize;

	#endif /* #ifndef CM0_FAMILY */

	  while(blkCnt > 0u)
	  {
	    /* C = A */
	    /* Copy and then store the results in the destination buffer */
	    *pDst++ = *pSrc++;

	    /* Decrement the loop counter */
	    blkCnt--;
	  }
}

/**
 * Vector Absolute Value
 *
 * Computes the absolute value of a vector on an element-by-element basis.
 *
 *     pDst[n] = abs(pSrc[n]),   0 <= n < blockSize.
 *
 * The functions support in-place computation allowing the source and
 * destination pointers to reference the same memory buffer.
 * There are separate functions for floating-point, Q7, Q15, and Q31 data types.
 */

/**
 * @brief Floating-point vector absolute value.
 * @param[in]       *pSrc points to the input buffer
 * @param[out]      *pDst points to the output buffer
 * @param[in]       blockSize number of samples in each vector
 * @return none.
 */

void abs_f32( float * pSrc, float * pDst, uint32_t blockSize)
{
  uint32_t blkCnt;                               /* loop counter */

#ifndef CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  float in1, in2, in3, in4;                  /* temporary variables */

  /*loop Unrolling */
  blkCnt = blockSize >> 2u;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* C = |A| */
    /* Calculate absolute and then store the results in the destination buffer. */
    /* read sample from source */
    in1 = *pSrc;
    in2 = *(pSrc + 1);
    in3 = *(pSrc + 2);

    /* find absolute value */
    in1 = fabsf(in1);

    /* read sample from source */
    in4 = *(pSrc + 3);

    /* find absolute value */
    in2 = fabsf(in2);

    /* read sample from source */
    *pDst = in1;

    /* find absolute value */
    in3 = fabsf(in3);

    /* find absolute value */
    in4 = fabsf(in4);

    /* store result to destination */
    *(pDst + 1) = in2;

    /* store result to destination */
    *(pDst + 2) = in3;

    /* store result to destination */
    *(pDst + 3) = in4;


    /* Update source pointer to process next sampels */
    pSrc += 4u;

    /* Update destination pointer to process next sampels */
    pDst += 4u;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;

#else

  /* Run the below code for Cortex-M0 */

  /* Initialize blkCnt with number of samples */
  blkCnt = blockSize;

#endif /*   #ifndef CM0_FAMILY   */

  while(blkCnt > 0u)
  {
    /* C = |A| */
    /* Calculate absolute and then store the results in the destination buffer. */
    *pDst++ = fabsf(*pSrc++);

    /* Decrement the loop counter */
    blkCnt--;
  }
}

/**
  Complex Conjugate
  Conjugates the elements of a complex data vector.
  The pSrc points to the source data and
  pDst points to the destination data where the result should be written.
  numSamples specifies the number of complex samples
  and the data in each array is stored in an interleaved fashion
  (real, imag, real, imag, ...).
  Each array has a total of 2*numSamples values.
  The underlying algorithm is used:
  for (n = 0; n < numSamples; n++) {
      pDst[(2*n)  ] =  pSrc[(2*n)  ];    // real part
      pDst[(2*n)+1] = -pSrc[(2*n)+1];    // imag part
  }
  There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
  @brief Floating-point complex conjugate.
  @param[in]     pSrc        points to the input vector
  @param[out]    pDst        points to the output vector
  @param[in]     numSamples  number of samples in each vector
  @return        none
 */


void cmplx_conj_f32( const float * pSrc, float * pDst, uint32_t numSamples)
{
        uint32_t blkCnt;                               /* Loop counter */

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = numSamples >> 2U;

  while (blkCnt > 0U)
  {
    /* C[0] + jC[1] = A[0]+ j(-1)A[1] */

    /* Calculate Complex Conjugate and store result in destination buffer. */
    *pDst++ =  *pSrc++;
    *pDst++ = -*pSrc++;

    *pDst++ =  *pSrc++;
    *pDst++ = -*pSrc++;

    *pDst++ =  *pSrc++;
    *pDst++ = -*pSrc++;

    *pDst++ =  *pSrc++;
    *pDst++ = -*pSrc++;

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = numSamples % 0x4U;


  /* Initialize blkCnt with number of samples */
  blkCnt = numSamples;

  while (blkCnt > 0U)
  {
    /* C[0] + jC[1] = A[0]+ j(-1)A[1] */

    /* Calculate Complex Conjugate and store result in destination buffer. */
    *pDst++ =  *pSrc++;
    *pDst++ = -*pSrc++;

    /* Decrement loop counter */
    blkCnt--;
  }

}

/**
  Complex-by-Complex Multiplication
  Multiplies a complex vector by another complex vector and generates a complex result.
  The data in the complex arrays is stored in an interleaved fashion
  (real, imag, real, imag, ...).
  The parameter numSamples represents the number of complex
  samples processed.  The complex arrays have a total of 2*numSamples
  real values.
  The underlying algorithm is used:
  for (n = 0; n < numSamples; n++) {
      pDst[(2*n)+0] = pSrcA[(2*n)+0] * pSrcB[(2*n)+0] - pSrcA[(2*n)+1] * pSrcB[(2*n)+1];
      pDst[(2*n)+1] = pSrcA[(2*n)+0] * pSrcB[(2*n)+1] + pSrcA[(2*n)+1] * pSrcB[(2*n)+0];
  }
  There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
  @brief Floating-point complex-by-complex multiplication.
  @param[in]     pSrcA       points to first input vector
  @param[in]     pSrcB       points to second input vector
  @param[out]    pDst        points to output vector
  @param[in]     numSamples  number of samples in each vector
  @return        none
 */

void cmplx_mult_cmplx_f32( const float * pSrcA, const float * pSrcB, float * pDst, uint32_t numSamples)
{
    uint32_t blkCnt;                               /* Loop counter */
    float a, b, c, d;  /* Temporary variables to store real and imaginary values */

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = numSamples >> 2U;

  while (blkCnt > 0U)
  {
    /* C[2 * i    ] = A[2 * i] * B[2 * i    ] - A[2 * i + 1] * B[2 * i + 1]. */
    /* C[2 * i + 1] = A[2 * i] * B[2 * i + 1] + A[2 * i + 1] * B[2 * i    ]. */

    a = *pSrcA++;
    b = *pSrcA++;
    c = *pSrcB++;
    d = *pSrcB++;
    /* store result in destination buffer. */
    *pDst++ = (a * c) - (b * d);
    *pDst++ = (a * d) + (b * c);

    a = *pSrcA++;
    b = *pSrcA++;
    c = *pSrcB++;
    d = *pSrcB++;
    *pDst++ = (a * c) - (b * d);
    *pDst++ = (a * d) + (b * c);

    a = *pSrcA++;
    b = *pSrcA++;
    c = *pSrcB++;
    d = *pSrcB++;
    *pDst++ = (a * c) - (b * d);
    *pDst++ = (a * d) + (b * c);

    a = *pSrcA++;
    b = *pSrcA++;
    c = *pSrcB++;
    d = *pSrcB++;
    *pDst++ = (a * c) - (b * d);
    *pDst++ = (a * d) + (b * c);

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = numSamples % 0x4U;

  /* Initialize blkCnt with number of samples */
  blkCnt = numSamples;

  while (blkCnt > 0U)
  {
    /* C[2 * i    ] = A[2 * i] * B[2 * i    ] - A[2 * i + 1] * B[2 * i + 1]. */
    /* C[2 * i + 1] = A[2 * i] * B[2 * i + 1] + A[2 * i + 1] * B[2 * i    ]. */

    a = *pSrcA++;
    b = *pSrcA++;
    c = *pSrcB++;
    d = *pSrcB++;

    /* store result in destination buffer. */
    *pDst++ = (a * c) - (b * d);
    *pDst++ = (a * d) + (b * c);

    /* Decrement loop counter */
    blkCnt--;
  }

}

/**
 * Complex-by-Real Multiplication
 * Multiplies a complex vector by a real vector and generates a complex result.
 * The data in the complex arrays is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * The parameter numSamples represents the number of complex
 * samples processed.  The complex arrays have a total of 2*numSamples
 * real values while the real array has a total of numSamples
 * real values.
 * The underlying algorithm is used:
 * for (n = 0; n < numSamples; n++) {
 * 		pCmplxDst[(2*n)+0] = pSrcCmplx[(2*n)+0] * pSrcReal[n];
 * 		pCmplxDst[(2*n)+1] = pSrcCmplx[(2*n)+1] * pSrcReal[n];
 * 	}
 * 	There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
  @brief         Floating-point complex-by-real multiplication.
  @param[in]     pSrcCmplx   points to complex input vector
  @param[in]     pSrcReal    points to real input vector
  @param[out]    pCmplxDst   points to complex output vector
  @param[in]     numSamples  number of samples in each vector
  @return        none
 */

void cmplx_mult_real_f32( const float * pSrcCmplx, const float * pSrcReal, float * pCmplxDst, uint32_t numSamples)
{
        uint32_t blkCnt;                               /* Loop counter */
        float in;                                  /* Temporary variable */

  /* Loop unrolling: Compute 4 outputs at a time */
  blkCnt = numSamples >> 2U;

  while (blkCnt > 0U)
  {
    /* C[2 * i    ] = A[2 * i    ] * B[i]. */
    /* C[2 * i + 1] = A[2 * i + 1] * B[i]. */

    in = *pSrcReal++;
    /* store result in destination buffer. */
    *pCmplxDst++ = *pSrcCmplx++ * in;
    *pCmplxDst++ = *pSrcCmplx++ * in;

    in = *pSrcReal++;
    *pCmplxDst++ = *pSrcCmplx++ * in;
    *pCmplxDst++ = *pSrcCmplx++ * in;

    in = *pSrcReal++;
    *pCmplxDst++ = *pSrcCmplx++ * in;
    *pCmplxDst++ = *pSrcCmplx++ * in;

    in = *pSrcReal++;
    *pCmplxDst++ = *pSrcCmplx++* in;
    *pCmplxDst++ = *pSrcCmplx++ * in;

    /* Decrement loop counter */
    blkCnt--;
  }

  /* Loop unrolling: Compute remaining outputs */
  blkCnt = numSamples % 0x4U;


  /* Initialize blkCnt with number of samples */
  blkCnt = numSamples;

  while (blkCnt > 0U)
  {
    /* C[2 * i    ] = A[2 * i    ] * B[i]. */
    /* C[2 * i + 1] = A[2 * i + 1] * B[i]. */

    in = *pSrcReal++;
    /* store result in destination buffer. */
    *pCmplxDst++ = *pSrcCmplx++ * in;
    *pCmplxDst++ = *pSrcCmplx++ * in;

    /* Decrement loop counter */
    blkCnt--;
  }

}

math_status sqrt_f32( float in, float * pOut)
{
	if(in >= 0.0f)
    {

#if   (__FPU_USED == 1) && defined ( __CC_ARM   )
      *pOut = __sqrtf(in);
#elif (__FPU_USED == 1) && (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
      *pOut = __builtin_sqrtf(in);
#elif (__FPU_USED == 1) && defined(__GNUC__)
      *pOut = __builtin_sqrtf(in);
#elif (__FPU_USED == 1) && defined ( __ICCARM__ ) && (__VER__ >= 6040000)
      __ASM("VSQRT.F32 %0,%1" : "=t"(*pOut) : "t"(in));
#else
      *pOut = sqrtf(in);
#endif

      return (MATH_SUCCESS);
    }
    else
    {
      *pOut = 0.0f;
      return (MATH_ARGUMENT_ERROR);
    }
}
