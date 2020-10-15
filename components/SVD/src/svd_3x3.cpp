//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 12-Feb-2019 21:10:49
//

// Include Files
#include "rt_nonfinite.h"
#include "svd_3x3.h"
#include "svd_3x3_impl.h"
#include "Matrix.h"
#include "BasicMath.h"

// Function Definitions

//
// Arguments    : const float A[9]
//                float U[9]
//                float S[9]
//                float V[9]
// Return Type  : void
//
void svd_3x3(const float Ain[9], float U[9], float S[9], float V[9])
{
  bool p;
  int i;
  float s[3];
  float fv0[9];
  float U1[9];
  float V1[9];
  p = true;

  matrix_instance_f32 Ain_; mat_init_f32(&Ain_, 3, 3, (float *)Ain);
  float A[3*3]; matrix_instance_f32 A_; mat_init_f32(&A_, 3, 3, (float *)A);
  mat_trans_f32(&Ain_, &A_);

  for (i = 0; i < 9; i++) {
    if (p && ((!rtIsInfF(A[i])) && (!rtIsNaNF(A[i])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (p) {
    b_svd_3x3(A, U, s, V);
  } else {
    for (i = 0; i < 9; i++) {
      fv0[i] = 0.0F;
    }

    b_svd_3x3(fv0, U1, s, V1);
    for (i = 0; i < 9; i++) {
      U[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 3; i++) {
      s[i] = ((real32_T)rtNaN);
    }

    for (i = 0; i < 9; i++) {
      V[i] = ((real32_T)rtNaN);
    }
  }

  for (i = 0; i < 9; i++) {
    S[i] = 0.0F;
  }

  for (i = 0; i < 3; i++) {
    S[i + 3 * i] = s[i];
  }

  /* Transpose matrices since MATLAB coder export of SVD with row-major order does not seem to work properly */
  matrix_instance_f32 U_; mat_init_f32(&U_, 3, 3, (float *)U);
  matrix_instance_f32 S_; mat_init_f32(&S_, 3, 3, (float *)S);
  matrix_instance_f32 V_; mat_init_f32(&V_, 3, 3, (float *)V);
  mat_trans_f32(&U_, &A_);
  copy_f32(A, U, 9);
  mat_trans_f32(&S_, &A_);
  copy_f32(A, S, 9);
  mat_trans_f32(&V_, &A_);
  copy_f32(A, V, 9);
}

//
// File trailer for svd.cpp
//
// [EOF]
//
