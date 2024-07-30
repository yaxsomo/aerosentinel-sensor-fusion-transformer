#ifndef UKF_H
#define UKF_H

#include "config.h"
#include "matrix.h"
#include <stdbool.h>

#if (MATRIX_MAXIMUM_SIZE < (2*SS_X_LEN + 1))
    #error("MATRIX_MAXIMUM_SIZE is not big enough for UKF (need at least (2*SS_X_LEN + 1))");
#endif
#if ((MATRIX_MAXIMUM_SIZE < SS_U_LEN) || (MATRIX_MAXIMUM_SIZE < SS_X_LEN) || (MATRIX_MAXIMUM_SIZE < SS_Z_LEN))
    #error("MATRIX_MAXIMUM_SIZE is not big enough for UKF (need at least SS_U_LEN / SS_X_LEN / SS_Z_LEN)");
#endif

typedef struct {
    Matrix X_Est;
    Matrix X_Sigma;
    Matrix Y_Est;
    Matrix Y_Sigma;
    Matrix P;
    Matrix P_Chol;
    Matrix DX;
    Matrix DY;
    Matrix Py;
    Matrix Pxy;
    Matrix Wm;
    Matrix Wc;
    Matrix Rv;
    Matrix Rn;
    Matrix Err;
    Matrix Gain;
    float_prec Gamma;
    bool (*bNonlinearUpdateX)(Matrix *X_Next, Matrix *X, Matrix *U);
    bool (*bNonlinearUpdateY)(Matrix *Y, Matrix *X, Matrix *U);
} UKF;

void UKF_Init(UKF *ukf, Matrix *XInit, Matrix *P, Matrix *Rv, Matrix *Rn, 
              bool (*bNonlinearUpdateX)(Matrix *, Matrix *, Matrix *), 
              bool (*bNonlinearUpdateY)(Matrix *, Matrix *, Matrix *));
void UKF_Reset(UKF *ukf, Matrix *XInit, Matrix *P, Matrix *Rv, Matrix *Rn);
bool UKF_Update(UKF *ukf, Matrix *Y, Matrix *U);

Matrix UKF_GetX(UKF *ukf);
Matrix UKF_GetY(UKF *ukf);
Matrix UKF_GetP(UKF *ukf);
Matrix UKF_GetErr(UKF *ukf);

bool UKF_CalculateSigmaPoint(UKF *ukf);
bool UKF_UnscentedTransform(UKF *ukf, Matrix *Out, Matrix *OutSigma, Matrix *P, Matrix *DSig,
                            bool (*_vFuncNonLinear)(Matrix *, Matrix *, Matrix *),
                            Matrix *InpSigma, Matrix *InpVector,
                            Matrix *_CovNoise);

#endif // UKF_H


