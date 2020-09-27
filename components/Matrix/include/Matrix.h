/*
 * Matrix.h
 *
 *  Created on: Aug 15, 2020
 *      Author: shivamchauhan
 */

#ifndef COMPONENTS_MATRIX_MATRIX_H_
#define COMPONENTS_MATRIX_MATRIX_H_

#include "stdio.h"

enum matrix_status
{
	MAT_SUCCESS,			//No error

	MAT_ARGUMENT_ERROR, 	//One or more arguments are incorrect

	MAT_LENGTH_ERROR, 		//Length of data buffer is incorrect

	MAT_SIZE_MISMATCH, 	//Size of matrices is not compatible with the operation

	MAT_NANINF, 			//Not-a-number (NaN) or infinity is generated

	MAT_SINGULAR, 			//Input matrix is singular and cannot be inverted

	MAT_TEST_FAILURE 		//Test Failed
};

typedef struct matrix_instance_f32
{
	uint16_t numRows;
	uint16_t numCols;
	float*   pData;
}matrix_instance_f32_t;


void mat_init_f32(matrix_instance_f32 * S, uint16_t nRows, uint16_t nColumns, float * pData);
matrix_status mat_trans_f32 (const matrix_instance_f32 *pSrc, matrix_instance_f32 *pDst);
matrix_status mat_mult_f32 (const matrix_instance_f32 *pSrcA, const matrix_instance_f32 *pSrcB, matrix_instance_f32 *pDst);
matrix_status mat_add_f32 (const matrix_instance_f32 *pSrcA, const matrix_instance_f32 *pSrcB, matrix_instance_f32 *pDst);
matrix_status mat_scale_f32 (const matrix_instance_f32 *pSrc, float scale, matrix_instance_f32 *pDst);

#endif /* COMPONENTS_MATRIX_MATRIX_H_ */
