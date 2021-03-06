//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 04-May-2020 22:25:44
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "svd_5x5.h"
#include "svd_5x5_impl.h"
#include "Matrix.h"
#include "BasicMath.h"

// Function Definitions

//
// Arguments    : const float A[25]
//                float U[25]
//                float S[25]
//                float V[25]
// Return Type  : void
//
void svd_5x5(const float Ain[25], float U[25], float S[25], float V[25])
{
  boolean_T p;
  int i;
  float fv0[25];
  float s[5];
  float U1[25];
  float V1[25];
  p = true;
  
  matrix_instance_f32 Ain_; mat_init_f32(&Ain_, 5, 5, (float *)Ain);
  float A[5*5]; matrix_instance_f32 A_; mat_init_f32(&A_, 5, 5, (float *)A);
  mat_trans_f32(&Ain_, &A_);
  
  for (i = 0; i < 25; i++) {
    if (p && ((!rtIsInfF(A[i])) && (!rtIsNaNF(A[i])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (p) {
    b_svd_5x5(A, U, s, V);
  } else {
    memset(&fv0[0], 0, 25U * sizeof(float));
    b_svd_5x5(fv0, U1, s, V1);
    for (i = 0; i < 25; i++) {
      U[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 5; i++) {
      s[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 25; i++) {
      V[i] = ((real32_T)rtNaN);
    }
  }

  memset(&S[0], 0, 25U * sizeof(float));
  for (i = 0; i < 5; i++) {
    S[i + 5 * i] = s[i];
  }
  
  /* Transpose matrices since MATLAB coder export of SVD with row-major order does not seem to work properly */
  matrix_instance_f32 U_; mat_init_f32(&U_, 5, 5, (float *)U);
  matrix_instance_f32 S_; mat_init_f32(&S_, 5, 5, (float *)S);
  matrix_instance_f32 V_; mat_init_f32(&V_, 5, 5, (float *)V);
  mat_trans_f32(&U_, &A_);
  copy_f32(A, U, 25);
  mat_trans_f32(&S_, &A_);
  copy_f32(A, S, 25);
  mat_trans_f32(&V_, &A_);
  copy_f32(A, V, 25);
}

//
// File trailer for svd.cpp
//
// [EOF]
//
