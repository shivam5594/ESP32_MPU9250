/*
 * Matrix.cpp
 *
 *  Created on: Aug 15, 2020
 *      Author: shivamchauhan
 */

#include "Matrix.h"

/**
 * Matrix Initialization
 *
 * Initializes the underlying matrix data structure.
 * The functions set the numRows, numCols, and pData fields of the matrix data structure.
 */

/**
 * @brief  Floating-point matrix initialization.
 * @param[in,out] *S             points to an instance of the floating-point matrix structure.
 * @param[in]     nRows          number of rows in the matrix.
 * @param[in]     nColumns       number of columns in the matrix.
 * @param[in]     *pData	   	   points to the matrix data array.
 * @return        none
 */

void mat_init_f32(matrix_instance_f32 * S, uint16_t nRows, uint16_t nColumns, float * pData)
{
	  /* Assign Number of Rows */
	  S->numRows = nRows;

	  /* Assign Number of Columns */
	  S->numCols = nColumns;

	  /* Assign Data pointer */
	  S->pData = pData;
}

/**
 * Matrix Transpose
 *
 * Tranposes a matrix.
 * Transposing an M x N matrix flips it around the center diagonal and results in an N x M matrix.
 */

/**
 * @brief Floating-point matrix transpose.
 * @param[in]  *pSrc points to the input matrix
 * @param[out] *pDst points to the output matrix
 * @return 	The function returns either MAT_SIZE_MISMATCH
 * 			or MAT_SUCCESS based on the outcome of size checking.
 */

matrix_status mat_trans_f32 (const matrix_instance_f32 *pSrc, matrix_instance_f32 *pDst)
{
	float *pIn = pSrc->pData;                  /* input data matrix pointer */
	float *pOut = pDst->pData;                 /* output data matrix pointer */
	float *px;                                 /* Temporary output data matrix pointer */
	uint16_t nRows = pSrc->numRows;                /* number of rows */
	uint16_t nColumns = pSrc->numCols;             /* number of columns */

	#ifndef CM0_FAMILY

	  /* Run the below code for Cortex-M4 and Cortex-M3 */

	  uint16_t blkCnt, i = 0u, row = nRows;          /* loop counters */
	  matrix_status status;                             /* status of matrix transpose  */


	#ifdef MATRIX_CHECK


	  /* Check for matrix mismatch condition */
	  if((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
	  {
	    /* Set status as MAT_SIZE_MISMATCH */
	    status = MAT_SIZE_MISMATCH;
	  }
	  else
	#endif /*    #ifdef MATRIX_CHECK    */

	  {
	    /* Matrix transpose by exchanging the rows with columns */
	    /* row loop     */
	    do
	    {
	      /* Loop Unrolling */
	      blkCnt = nColumns >> 2;

	      /* The pointer px is set to starting address of the column being processed */
	      px = pOut + i;

	      /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	       ** a second loop below computes the remaining 1 to 3 samples. */
	      while(blkCnt > 0u)        /* column loop */
	      {
	        /* Read and store the input element in the destination */
	        *px = *pIn++;

	        /* Update the pointer px to point to the next row of the transposed matrix */
	        px += nRows;

	        /* Read and store the input element in the destination */
	        *px = *pIn++;

	        /* Update the pointer px to point to the next row of the transposed matrix */
	        px += nRows;

	        /* Read and store the input element in the destination */
	        *px = *pIn++;

	        /* Update the pointer px to point to the next row of the transposed matrix */
	        px += nRows;

	        /* Read and store the input element in the destination */
	        *px = *pIn++;

	        /* Update the pointer px to point to the next row of the transposed matrix */
	        px += nRows;

	        /* Decrement the column loop counter */
	        blkCnt--;
	      }

	      /* Perform matrix transpose for last 3 samples here. */
	      blkCnt = nColumns % 0x4u;

	      while(blkCnt > 0u)
	      {
	        /* Read and store the input element in the destination */
	        *px = *pIn++;

	        /* Update the pointer px to point to the next row of the transposed matrix */
	        px += nRows;

	        /* Decrement the column loop counter */
	        blkCnt--;
	      }

	#else

	  /* Run the below code for Cortex-M0 */

	  uint16_t col, i = 0u, row = nRows;             /* loop counters */
	  matrix_status status;                             /* status of matrix transpose  */


	#ifdef MATRIX_CHECK

	  /* Check for matrix mismatch condition */
	  if((pSrc->numRows != pDst->numCols) || (pSrc->numCols != pDst->numRows))
	  {
	    /* Set status as MAT_SIZE_MISMATCH */
	    status = MAT_SIZE_MISMATCH;
	  }
	  else
	#endif /*    #ifdef MATRIX_CHECK    */

	  {
	    /* Matrix transpose by exchanging the rows with columns */
	    /* row loop     */
	    do
	    {
	      /* The pointer px is set to starting address of the column being processed */
	      px = pOut + i;

	      /* Initialize column loop counter */
	      col = nColumns;

	      while(col > 0u)
	      {
	        /* Read and store the input element in the destination */
	        *px = *pIn++;

	        /* Update the pointer px to point to the next row of the transposed matrix */
	        px += nRows;

	        /* Decrement the column loop counter */
	        col--;
	      }

	#endif /* #ifndef CM0_FAMILY */

	      i++;

	      /* Decrement the row loop counter */
	      row--;

	    } while(row > 0u);          /* row loop end  */

	    /* Set status as MAT_SUCCESS */
	    status = MAT_SUCCESS;
	  }

	  /* Return to application */
	  return (status);
}

/**
 * Matrix Multiplication
 *
 * Multiplies two matrices.
 *
 * Matrix multiplication is only defined if the number of columns of the
 * first matrix equals the number of rows of the second matrix.
 * Multiplying an M x N matrix with an N x P matrix results in an M x P matrix.
 * When matrix size checking is enabled, the functions check:
 * (1) that the inner dimensions of pSrcA and pSrcB are equal; and
 * (2) that the size of the output
 * matrix equals the outer dimensions of pSrcA and pSrcB.
 */

/**
 * @brief Floating-point matrix multiplication.
 * @param[in]       *pSrcA points to the first input matrix structure
 * @param[in]       *pSrcB points to the second input matrix structure
 * @param[out]      *pDst points to output matrix structure
 * @return     		The function returns either
 * 					MAT_SIZE_MISMATCH or MAT_SUCCESS based on the outcome of size checking.
 */

matrix_status 	mat_mult_f32 (const matrix_instance_f32 *pSrcA, const matrix_instance_f32 *pSrcB, matrix_instance_f32 *pDst)
{
  float *pIn1 = pSrcA->pData;                /* input data matrix pointer A */
  float *pIn2 = pSrcB->pData;                /* input data matrix pointer B */
  float *pInA = pSrcA->pData;                /* input data matrix pointer A  */
  float *pOut = pDst->pData;                 /* output data matrix pointer */
  float *px;                                 /* Temporary output data matrix pointer */
  float sum;                                 /* Accumulator */
  uint16_t numRowsA = pSrcA->numRows;            /* number of rows of input matrix A */
  uint16_t numColsB = pSrcB->numCols;            /* number of columns of input matrix B */
  uint16_t numColsA = pSrcA->numCols;            /* number of columns of input matrix A */

#ifndef CM0_FAMILY

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  float in1, in2, in3, in4;
  uint16_t col, i = 0u, j, row = numRowsA, colCnt;      /* loop counters */
  matrix_status status;                             /* status of matrix multiplication */

#ifdef MATRIX_CHECK


  /* Check for matrix mismatch condition */
  if((pSrcA->numCols != pSrcB->numRows) ||
	 (pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
  {

	/* Set status as MAT_SIZE_MISMATCH */
	status = MAT_SIZE_MISMATCH;
  }
  else
#endif /*      #ifdef MATRIX_CHECK    */

  {
	/* The following loop performs the dot-product of each row in pSrcA with each column in pSrcB */
	/* row loop */
	do
	{
	  /* Output pointer is set to starting address of the row being processed */
	  px = pOut + i;

	  /* For every row wise process, the column loop counter is to be initiated */
	  col = numColsB;

	  /* For every row wise process, the pIn2 pointer is set
	   ** to the starting address of the pSrcB data */
	  pIn2 = pSrcB->pData;

	  j = 0u;

	  /* column loop */
	  do
	  {
		/* Set the variable sum, that acts as accumulator, to zero */
		sum = 0.0f;

		/* Initiate the pointer pIn1 to point to the starting address of the column being processed */
		pIn1 = pInA;

		/* Apply loop unrolling and compute 4 MACs simultaneously. */
		colCnt = numColsA >> 2u;

		/* matrix multiplication        */
		while(colCnt > 0u)
		{
		  /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
		  in3 = *pIn2;
		  pIn2 += numColsB;
		  in1 = pIn1[0];
		  in2 = pIn1[1];
		  sum += in1 * in3;
		  in4 = *pIn2;
		  pIn2 += numColsB;
		  sum += in2 * in4;

		  in3 = *pIn2;
		  pIn2 += numColsB;
		  in1 = pIn1[2];
		  in2 = pIn1[3];
		  sum += in1 * in3;
		  in4 = *pIn2;
		  pIn2 += numColsB;
		  sum += in2 * in4;
		  pIn1 += 4u;

		  /* Decrement the loop count */
		  colCnt--;
		}

		/* If the columns of pSrcA is not a multiple of 4, compute any remaining MACs here.
		 ** No loop unrolling is used. */
		colCnt = numColsA % 0x4u;

		while(colCnt > 0u)
		{
		  /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
		  sum += *pIn1++ * (*pIn2);
		  pIn2 += numColsB;

		  /* Decrement the loop counter */
		  colCnt--;
		}

		/* Store the result in the destination buffer */
		*px++ = sum;

		/* Update the pointer pIn2 to point to the  starting address of the next column */
		j++;
		pIn2 = pSrcB->pData + j;

		/* Decrement the column loop counter */
		col--;

	  } while(col > 0u);

#else

  /* Run the below code for Cortex-M0 */

  float *pInB = pSrcB->pData;                /* input data matrix pointer B */
  uint16_t col, i = 0u, row = numRowsA, colCnt;  /* loop counters */
  matrix_status status;                             /* status of matrix multiplication */

#ifdef MATRIX_CHECK

  /* Check for matrix mismatch condition */
  if((pSrcA->numCols != pSrcB->numRows) ||
	 (pSrcA->numRows != pDst->numRows) || (pSrcB->numCols != pDst->numCols))
  {

	/* Set status as MAT_SIZE_MISMATCH */
	status = MAT_SIZE_MISMATCH;
  }
  else
#endif /*      #ifdef MATRIX_CHECK    */

  {
	/* The following loop performs the dot-product of each row in pInA with each column in pInB */
	/* row loop */
	do
	{
	  /* Output pointer is set to starting address of the row being processed */
	  px = pOut + i;

	  /* For every row wise process, the column loop counter is to be initiated */
	  col = numColsB;

	  /* For every row wise process, the pIn2 pointer is set
	   ** to the starting address of the pSrcB data */
	  pIn2 = pSrcB->pData;

	  /* column loop */
	  do
	  {
		/* Set the variable sum, that acts as accumulator, to zero */
		sum = 0.0f;

		/* Initialize the pointer pIn1 to point to the starting address of the row being processed */
		pIn1 = pInA;

		/* Matrix A columns number of MAC operations are to be performed */
		colCnt = numColsA;

		while(colCnt > 0u)
		{
		  /* c(m,n) = a(1,1)*b(1,1) + a(1,2) * b(2,1) + .... + a(m,p)*b(p,n) */
		  sum += *pIn1++ * (*pIn2);
		  pIn2 += numColsB;

		  /* Decrement the loop counter */
		  colCnt--;
		}

		/* Store the result in the destination buffer */
		*px++ = sum;

		/* Decrement the column loop counter */
		col--;

		/* Update the pointer pIn2 to point to the  starting address of the next column */
		pIn2 = pInB + (numColsB - col);

	  } while(col > 0u);

#endif /* #ifndef CM0_FAMILY */

	  /* Update the pointer pInA to point to the  starting address of the next row */
	  i = i + numColsB;
	  pInA = pInA + numColsA;

	  /* Decrement the row loop counter */
	  row--;

	} while(row > 0u);
	/* Set status as MAT_SUCCESS */
	status = MAT_SUCCESS;
  }

  /* Return to application */
  return (status);
}

/**
 * Matrix Addition
 * Adds two matrices.
 *
 * The functions check to make sure that
 * pSrcA, pSrcB, and pDst have the same
 * number of rows and columns.
 */

/**
 * @brief Floating-point matrix addition.
 * @param[in]       *pSrcA points to the first input matrix structure
 * @param[in]       *pSrcB points to the second input matrix structure
 * @param[out]      *pDst points to output matrix structure
 * @return     		The function returns either
 * 					MAT_SIZE_MISMATCH or MAT_SUCCESS based on the outcome of size checking.
 */

matrix_status mat_add_f32 (const matrix_instance_f32 *pSrcA, const matrix_instance_f32 *pSrcB, matrix_instance_f32 *pDst)
{
	float *pIn1 = pSrcA->pData;                /* input data matrix pointer A  */
	float *pIn2 = pSrcB->pData;                /* input data matrix pointer B  */
	float *pOut = pDst->pData;                 /* output data matrix pointer   */

#ifndef ACM0_FAMILY

	float inA1, inA2, inB1, inB2, out1, out2;  /* temporary variables */

#endif //      #ifndef MAT_CM0_FAMILY

	uint32_t numSamples;                           /* total number of elements in the matrix  */
	uint32_t blkCnt;                               /* loop counters */
	matrix_status status;                             /* status of matrix addition */

	#ifdef MATRIX_CHECK
	/* Check for matrix mismatch condition */
	if((pSrcA->numRows != pSrcB->numRows) ||
	 (pSrcA->numCols != pSrcB->numCols) ||
	 (pSrcA->numRows != pDst->numRows) || (pSrcA->numCols != pDst->numCols))
	{
	/* Set status as MAT_SIZE_MISMATCH */
	status = MAT_SIZE_MISMATCH;
	}
	else
	#endif
	{

	/* Total number of samples in the input matrix */
	numSamples = (uint32_t) pSrcA->numRows * pSrcA->numCols;

	#ifndef ARM_MATH_CM0_FAMILY

	/* Loop unrolling */
	blkCnt = numSamples >> 2u;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	 ** a second loop below computes the remaining 1 to 3 samples. */
	while(blkCnt > 0u)
	{
	  /* C(m,n) = A(m,n) + B(m,n) */
	  /* Add and then store the results in the destination buffer. */
	  /* Read values from source A */
	  inA1 = pIn1[0];

	  /* Read values from source B */
	  inB1 = pIn2[0];

	  /* Read values from source A */
	  inA2 = pIn1[1];

	  /* out = sourceA + sourceB */
	  out1 = inA1 + inB1;

	  /* Read values from source B */
	  inB2 = pIn2[1];

	  /* Read values from source A */
	  inA1 = pIn1[2];

	  /* out = sourceA + sourceB */
	  out2 = inA2 + inB2;

	  /* Read values from source B */
	  inB1 = pIn2[2];

	  /* Store result in destination */
	  pOut[0] = out1;
	  pOut[1] = out2;

	  /* Read values from source A */
	  inA2 = pIn1[3];

	  /* Read values from source B */
	  inB2 = pIn2[3];

	  /* out = sourceA + sourceB */
	  out1 = inA1 + inB1;

	  /* out = sourceA + sourceB */
	  out2 = inA2 + inB2;

	  /* Store result in destination */
	  pOut[2] = out1;

	  /* Store result in destination */
	  pOut[3] = out2;


	  /* update pointers to process next samples */
	  pIn1 += 4u;
	  pIn2 += 4u;
	  pOut += 4u;
	  /* Decrement the loop counter */
	  blkCnt--;
	}

	/* If the numSamples is not a multiple of 4, compute any remaining output samples here.
	 ** No loop unrolling is used. */
	blkCnt = numSamples % 0x4u;

#else

	/* Run the below code for Cortex-M0 */

	/* Initialize blkCnt with number of samples */
	blkCnt = numSamples;

#endif /* #ifndef CM0_FAMILY */

	while(blkCnt > 0u)
	{
	  /* C(m,n) = A(m,n) + B(m,n) */
	  /* Add and then store the results in the destination buffer. */
	  *pOut++ = (*pIn1++) + (*pIn2++);

	  /* Decrement the loop counter */
	  blkCnt--;
	}

	/* set status as MAT_SUCCESS */
	status = MAT_SUCCESS;

	}

	/* Return to application */
	return (status);
}

/**
 * Matrix Scale
 *
 * Multiplies a matrix by a scalar.  This is accomplished by multiplying each element in the
 * matrix by the scalar.
 *
 * The function checks to make sure that the input and output matrices are of the same size.
 *
 * In the fixed-point Q15 and Q31 functions, scale is represented by
 * a fractional multiplication scaleFract and an arithmetic shift shift.
 * The shift allows the gain of the scaling operation to exceed 1.0.
 * The overall scale factor applied to the fixed-point data is
 *     scale = scaleFract * 2^shift.
 */

/**
 * @brief Floating-point matrix scaling.
 * @param[in]       *pSrc points to input matrix structure
 * @param[in]       scale scale factor to be applied
 * @param[out]      *pDst points to output matrix structure
 * @return     		The function returns either MAT_SIZE_MISMATCH
 * 					or MAT_SUCCESS based on the outcome of size checking.
 *
 */

matrix_status mat_scale_f32 (const matrix_instance_f32 *pSrc, float scale, matrix_instance_f32 *pDst)
{
	float *pIn = pSrc->pData;                  /* input data matrix pointer */
	float *pOut = pDst->pData;                 /* output data matrix pointer */
	uint32_t numSamples;                           /* total number of elements in the matrix */
	uint32_t blkCnt;                               /* loop counters */
	matrix_status status;                             /* status of matrix scaling     */

#ifndef CM0_FAMILY

	float in1, in2, in3, in4;                  /* temporary variables */
	float out1, out2, out3, out4;              /* temporary variables */

#endif // #ifndef CM0_FAMILY

#ifdef MATRIX_CHECK
	/* Check for matrix mismatch condition */
	if((pSrc->numRows != pDst->numRows) || (pSrc->numCols != pDst->numCols))
	{
	/* Set status as MAT_SIZE_MISMATCH */
	status = MAT_SIZE_MISMATCH;
	}
	else
#endif /*    #ifdef MATRIX_CHECK    */
	{
	/* Total number of samples in the input matrix */
	numSamples = (uint32_t) pSrc->numRows * pSrc->numCols;

#ifndef CM0_FAMILY

	/* Run the below code for Cortex-M4 and Cortex-M3 */

	/* Loop Unrolling */
	blkCnt = numSamples >> 2;

	/* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
	 ** a second loop below computes the remaining 1 to 3 samples. */
	while(blkCnt > 0u)
	{
	  /* C(m,n) = A(m,n) * scale */
	  /* Scaling and results are stored in the destination buffer. */
	  in1 = pIn[0];
	  in2 = pIn[1];
	  in3 = pIn[2];
	  in4 = pIn[3];

	  out1 = in1 * scale;
	  out2 = in2 * scale;
	  out3 = in3 * scale;
	  out4 = in4 * scale;


	  pOut[0] = out1;
	  pOut[1] = out2;
	  pOut[2] = out3;
	  pOut[3] = out4;

	  /* update pointers to process next samples */
	  pIn += 4u;
	  pOut += 4u;

	  /* Decrement the numSamples loop counter */
	  blkCnt--;
	}

	/* If the numSamples is not a multiple of 4, compute any remaining output samples here.
	 ** No loop unrolling is used. */
	blkCnt = numSamples % 0x4u;

#else

	/* Run the below code for Cortex-M0 */

	/* Initialize blkCnt with number of samples */
	blkCnt = numSamples;

#endif /* #ifndef CM0_FAMILY */

	while(blkCnt > 0u)
	{
	  /* C(m,n) = A(m,n) * scale */
	  /* The results are stored in the destination buffer. */
	  *pOut++ = (*pIn++) * scale;

	  /* Decrement the loop counter */
	  blkCnt--;
	}

	/* Set status as ARM_MATH_SUCCESS */
	status = MAT_SUCCESS;
	}

	/* Return to application */
	return (status);
}
