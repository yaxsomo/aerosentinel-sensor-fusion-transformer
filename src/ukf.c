#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "ukf.h"

typedef struct {
    float **data;
    int rows;
    int cols;
} Matrix;

Matrix Matrix_Copy(Matrix *src);
Matrix Matrix_Transpose(Matrix *src);
Matrix Matrix_Inverse(Matrix *src);
Matrix Matrix_CholeskyDec(Matrix *src);
Matrix Matrix_InsertVector(Matrix *src, Matrix *vec, int col);
Matrix Matrix_InsertSubMatrix(Matrix *src, Matrix *sub, int row, int col);
Matrix Matrix_Subtract(Matrix *a, Matrix *b);
Matrix Matrix_Add(Matrix *a, Matrix *b);
Matrix Matrix_Multiply(Matrix *a, Matrix *b);
void Matrix_SetToZero(Matrix *m);
bool Matrix_IsValid(Matrix *m);

typedef struct {
    Matrix X_Est;
    Matrix P;
    Matrix Rv;
    Matrix Rn;
    Matrix Wm;
    Matrix Wc;
    Matrix X_Sigma;
    Matrix DX;
    Matrix DY;
    Matrix Pxy;
    Matrix Gain;
    Matrix Err;
    Matrix Y_Est;
    Matrix Y_Sigma;
    Matrix Py;
    Matrix P_Chol;
    float Gamma;
    bool (*bNonlinearUpdateX)(Matrix *, Matrix *, Matrix *);
    bool (*bNonlinearUpdateY)(Matrix *, Matrix *, Matrix *);
} UKF;

void UKF_Init(UKF *ukf, Matrix *XInit, Matrix *P, Matrix *Rv, Matrix *Rn, 
              bool (*bNonlinearUpdateX)(Matrix *, Matrix *, Matrix *), 
              bool (*bNonlinearUpdateY)(Matrix *, Matrix *, Matrix *)) {
    ukf->X_Est = *XInit;
    ukf->P = Matrix_Copy(P);
    ukf->Rv = Matrix_Copy(Rv);
    ukf->Rn = Matrix_Copy(Rn);
    ukf->bNonlinearUpdateX = bNonlinearUpdateX;
    ukf->bNonlinearUpdateY = bNonlinearUpdateY;

    float _alpha = 1e-2;
    float _k = 0.0;
    float _beta = 2.0;

    float _lambda = (_alpha * _alpha) * (SS_X_LEN + _k) - SS_X_LEN;
    ukf->Gamma = sqrt((SS_X_LEN + _lambda));

    ukf->Wm.data[0][0] = _lambda / (SS_X_LEN + _lambda);
    for (int _i = 1; _i < ukf->Wm.cols; _i++) {
        ukf->Wm.data[0][_i] = 0.5 / (SS_X_LEN + _lambda);
    }

    ukf->Wc = Matrix_Copy(&ukf->Wm);
    ukf->Wc.data[0][0] += (1.0 - (_alpha * _alpha) + _beta);
}

void UKF_Reset(UKF *ukf, Matrix *XInit, Matrix *P, Matrix *Rv, Matrix *Rn) {
    ukf->X_Est = *XInit;
    ukf->P = Matrix_Copy(P);
    ukf->Rv = Matrix_Copy(Rv);
    ukf->Rn = Matrix_Copy(Rn);
}

bool UKF_Update(UKF *ukf, Matrix *Y, Matrix *U) {
    if (!UKF_CalculateSigmaPoint(ukf)) {
        return false;
    }

    if (!UKF_UnscentedTransform(ukf, &ukf->X_Est, &ukf->X_Sigma, &ukf->P, &ukf->DX, 
                                ukf->bNonlinearUpdateX, &ukf->X_Sigma, U, &ukf->Rv)) {
        return false;
    }

    if (!UKF_UnscentedTransform(ukf, &ukf->Y_Est, &ukf->Y_Sigma, &ukf->Py, &ukf->DY, 
                                ukf->bNonlinearUpdateY, &ukf->X_Sigma, U, &ukf->Rn)) {
        return false;
    }

    for (int _i = 0; _i < ukf->DX.rows; _i++) {
        for (int _j = 0; _j < ukf->DX.cols; _j++) {
            ukf->DX.data[_i][_j] *= ukf->Wc.data[0][_j];
        }
    }
    ukf->Pxy = Matrix_Multiply(&ukf->DX, Matrix_Transpose(&ukf->DY));

    Matrix PyInv = Matrix_Inverse(&ukf->Py);
    if (!Matrix_IsValid(&PyInv)) {
        return false;
    }
    ukf->Gain = Matrix_Multiply(&ukf->Pxy, &PyInv);

    ukf->Err = Matrix_Subtract(Y, &ukf->Y_Est);
    ukf->X_Est = Matrix_Add(&ukf->X_Est, Matrix_Multiply(&ukf->Gain, &ukf->Err));

    ukf->P = Matrix_Subtract(&ukf->P, Matrix_Multiply(Matrix_Multiply(&ukf->Gain, &ukf->Py), Matrix_Transpose(&ukf->Gain)));

    return true;
}

bool UKF_CalculateSigmaPoint(UKF *ukf) {
    ukf->P_Chol = Matrix_CholeskyDec(&ukf->P);
    if (!Matrix_IsValid(&ukf->P_Chol)) {
        return false;
    }
    ukf->P_Chol = Matrix_Multiply(&ukf->P_Chol, ukf->Gamma);

    Matrix _Y = Matrix_Create(SS_X_LEN, SS_X_LEN);
    for (int _i = 0; _i < SS_X_LEN; _i++) {
        _Y = Matrix_InsertVector(&_Y, &ukf->X_Est, _i);
    }

    Matrix_SetToZero(&ukf->X_Sigma);
    ukf->X_Sigma = Matrix_InsertVector(&ukf->X_Sigma, &ukf->X_Est, 0);
    ukf->X_Sigma = Matrix_InsertSubMatrix(&ukf->X_Sigma, Matrix_Add(&_Y, &ukf->P_Chol), 0, 1);
    ukf->X_Sigma = Matrix_InsertSubMatrix(&ukf->X_Sigma, Matrix_Subtract(&_Y, &ukf->P_Chol), 0, 1 + SS_X_LEN);

    return true;
}

bool UKF_UnscentedTransform(UKF *ukf, Matrix *Out, Matrix *OutSigma, Matrix *P, Matrix *DSig,
                            bool (*_vFuncNonLinear)(Matrix *, Matrix *, Matrix *),
                            Matrix *InpSigma, Matrix *InpVector, Matrix *_CovNoise) {
    Matrix_SetToZero(Out);
    for (int _j = 0; _j < InpSigma->cols; _j++) {
        Matrix _AuxSigma1 = Matrix_Create(InpSigma->rows, 1);
        Matrix _AuxSigma2 = Matrix_Create(OutSigma->rows, 1);
        for (int _i = 0; _i < InpSigma->rows; _i++) {
            _AuxSigma1.data[_i][0] = InpSigma->data[_i][_j];
        }
        if (!_vFuncNonLinear(&_AuxSigma2, &_AuxSigma1, InpVector)) {
            return false;
        }

        *OutSigma = Matrix_InsertVector(OutSigma, &_AuxSigma2, _j);
        _AuxSigma2 = Matrix_Multiply(&_AuxSigma2, ukf->Wm.data[0][_j]);
        *Out = Matrix_Add(Out, &_AuxSigma2);
    }

    Matrix _AuxSigma1 = Matrix_Create(OutSigma->rows, OutSigma->cols);
    for (int _j = 0; _j < OutSigma->cols; _j++) {
        _AuxSigma1 = Matrix_InsertVector(&_AuxSigma1, Out, _j);
    }
    *DSig = Matrix_Subtract(OutSigma, &_AuxSigma1);

    _AuxSigma1 = Matrix_Copy(DSig);
    for (int _i = 0; _i < DSig->rows; _i++) {
        for (int _j = 0; _j < DSig->cols; _j++) {
            _AuxSigma1.data[_i][_j] *= ukf->Wc.data[0][_j];
        }
    }
    *P = Matrix_Add(Matrix_Multiply(&_AuxSigma1, Matrix_Transpose(DSig)), _CovNoise);

    return true;
}
