/* Copyright 2015 The MathWorks, Inc. */

/****************************************************
*                                                   *   
* wrapper fuctions for CMSIS  functions             *
*                                                   *  
****************************************************/

#ifndef MW_CMSIS_H
#define MW_CMSIS_H

#include "stdio.h"
#include "rtwtypes.h"
#include "BasicMath.h"

#define mw_abs_f32(pSrc, pDst, blockSize) abs_f32((float *)pSrc, (float *)pDst, blockSize)

#define mw_sqrt_f32(in, pOut) sqrt_f32((float)in,(float *)pOut)

#define mw_float_to_q31(pSrc, pDst, blockSize) float_to_q31((float *)pSrc, (q31_t *)pDst, blockSize)
#define mw_float_to_q15(pSrc, pDst, blockSize) float_to_q15((float *)pSrc, (q15_t *)pDst, blockSize)
#define mw_float_to_q7(pSrc, pDst, blockSize) float_to_q7((float *)pSrc, (q7_t *)pDst, blockSize)

//#define mw_q15_to_float(pSrc, pDst, blockSize) q15_to_float((q15_t *)pSrc, (float *)pDst, blockSize)

//#define mw_q31_to_float(pSrc, pDst, blockSize) q31_to_float((q31_t *)pSrc, (float *)pDst, blockSize)

//#define mw_q7_to_float(pSrc, pDst, blockSize) q7_to_float((q7_t *)pSrc, (float *)pDst, blockSize)

//#define mw_add_f32(pSrcA, pSrcB, pDst, blockSize) add_f32((float *)pSrcA, (float *)pSrcB, (float *)pDst, blockSize)

//#define mw_sub_f32(pSrcA, pSrcB, pDst, blockSize) sub_f32((float *)pSrcA, (float *)pSrcB, (float *)pDst, blockSize)

//#define mw_mult_f32(pSrcA, pSrcB, pDst, blockSize) mult_f32((float *)pSrcA, (float *)pSrcB, (float *)pDst, blockSize)

#define mw_cmplx_conj_f32(pSrc, pDst, numSamples) cmplx_conj_f32((float *)pSrc, (float *)pDst, numSamples)

#define mw_cmplx_mult_cmplx_f32(pSrcA, pSrcB, pDst, blockSize) cmplx_mult_cmplx_f32((float *)pSrcA, (float *)pSrcB, (float *)pDst, blockSize)

#define mw_cmplx_mult_real_f32(pSrcA, pSrcB, pDst, blockSize) cmplx_mult_real_f32((float *)pSrcA, (float *)pSrcB, (float *)pDst, blockSize)

/* Wrapper function prototypes for Matrix Addition */
//void mw_mat_add_f32(real32_T * pSrcA, real32_T * pSrcB, real32_T * pDst, uint16_t nRows, uint16_t nCols);
//void mw_mat_add_q15(int16_T * pSrcA, int16_T * pSrcB, int16_T * pDst, uint16_t nRows, uint16_t nCols);
//void mw_mat_add_q31(int32_T * pSrcA, int32_T * pSrcB, int32_T * pDst, uint16_t nRows, uint16_t nCols);
/* Wrapper function prototypes for Matrix Subtraction */
//void mw_mat_sub_f32(real32_T * pSrcA, real32_T * pSrcB, real32_T * pDst, uint16_t nRows, uint16_t nCols);
//void mw_mat_sub_q15(int16_T * pSrcA, int16_T * pSrcB, int16_T * pDst, uint16_t nRows, uint16_t nCols);
//void mw_mat_sub_q31(int32_T * pSrcA, int32_T * pSrcB, int32_T * pDst, uint16_t nRows, uint16_t nCols);

#endif
