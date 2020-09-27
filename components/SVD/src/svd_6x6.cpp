//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:30:05
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "svd_6x6.h"
#include "svd_6x6_impl.h"
#include "Matrix.h"
#include "BasicMath.h"

// Function Definitions

//
// Arguments    : const float A[36]
//                float U[36]
//                float S[36]
//                float V[36]
// Return Type  : void
//
void svd_6x6(const float Ain[36], float U[36], float S[36], float V[36])
{
  boolean_T p;
  int i;
  float fv0[36];
  float s[6];
  float U1[36];
  float V1[36];
  p = true;
  
  matrix_instance_f32 Ain_; mat_init_f32(&Ain_, 6, 6, (float *)Ain);
  float A[6*6]; matrix_instance_f32 A_; mat_init_f32(&A_, 6, 6, (float *)A);
  mat_trans_f32(&Ain_, &A_);
  
  for (i = 0; i < 36; i++) {
    if (p && ((!rtIsInfF(A[i])) && (!rtIsNaNF(A[i])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (p) {
    b_svd_6x6(A, U, s, V);
  } else {
    memset(&fv0[0], 0, 36U * sizeof(float));
    b_svd_6x6(fv0, U1, s, V1);
    for (i = 0; i < 36; i++) {
      U[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 6; i++) {
      s[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 36; i++) {
      V[i] = ((real32_T)rtNaN);
    }
  }

  memset(&S[0], 0, 36U * sizeof(float));
  for (i = 0; i < 6; i++) {
    S[i + 6 * i] = s[i];
  }
  
  /* Transpose matrices since MATLAB coder export of SVD with row-major order does not seem to work properly */
  matrix_instance_f32 U_; mat_init_f32(&U_, 6, 6, (float *)U);
  matrix_instance_f32 S_; mat_init_f32(&S_, 6, 6, (float *)S);
  matrix_instance_f32 V_; mat_init_f32(&V_, 6, 6, (float *)V);
  mat_trans_f32(&U_, &A_);
  copy_f32(A, U, 36);
  mat_trans_f32(&S_, &A_);
  copy_f32(A, S, 36);
  mat_trans_f32(&V_, &A_);
  copy_f32(A, V, 36);
}

//
// File trailer for svd.cpp
//
// [EOF]
//
