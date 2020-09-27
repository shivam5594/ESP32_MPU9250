/*
 * BasicMath.h
 *
 *  Created on: Aug 15, 2020
 *      Author: shivamchauhan
 */

#ifndef COMPONENTS_BASICMATH_BASICMATH_H_
#define COMPONENTS_BASICMATH_BASICMATH_H_

enum math_status
{
	MATH_SUCCESS,			//No error
	MATH_ARGUMENT_ERROR, 	//One or more arguments are incorrect
	MATH_LENGTH_ERROR, 		//Length of data buffer is incorrect
	MATH_SIZE_MISMATCH, 	//Size of matrices is not compatible with the operation
	MATH_NANINF, 			//Not-a-number (NaN) or infinity is generated
	MATH_SINGULAR, 			//Input matrix is singular and cannot be inverted
	MATH_TEST_FAILURE 		//Test Failed
};

void add_f32(float * pSrcA, float * pSrcB, float * pDst, uint32_t blockSize);
void sub_f32( const float * pSrcA, const float * pSrcB, float * pDst, uint32_t blockSize);
void scale_f32(const float *pSrc, float scale, float *pDst, uint32_t blockSize);
void dot_prod_f32 (const float *pSrcA, const float *pSrcB, uint32_t blockSize, float *result);
void mult_f32( const float * pSrcA, const float * pSrcB, float * pDst, uint32_t blockSize);
void copy_f32 (const float *pSrc, float *pDst, uint32_t blockSize);
void abs_f32( float * pSrc, float * pDst, uint32_t blockSize);
void cmplx_conj_f32( const float * pSrc, float * pDst, uint32_t numSamples);
void cmplx_mult_cmplx_f32( const float * pSrcA, const float * pSrcB, float * pDst, uint32_t numSamples);
void cmplx_mult_real_f32( const float * pSrcCmplx, const float * pSrcReal, float * pCmplxDst, uint32_t numSamples);
math_status sqrt_f32( float in, float * pOut);

#endif /* COMPONENTS_BASICMATH_BASICMATH_H_ */
