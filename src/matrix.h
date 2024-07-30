#ifndef MATRIX_H
#define MATRIX_H

#include "config.h"

#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
    #include <stdio.h>
    #include <math.h>
    #include <stdbool.h>
#endif

typedef enum {
    InitMatWithZero,    /* Initialize matrix with zero */
    NoInitMatZero
} InitZero;

typedef struct {
    int16_t i16row;
    int16_t i16col;
    float **f32data;
} Matrix;

Matrix* Matrix_create(const int16_t _i16row, const int16_t _i16col, InitZero _init) {
    Matrix *mat = (Matrix*)malloc(sizeof(Matrix));
    mat->i16row = _i16row;
    mat->i16col = _i16col;
    mat->f32data = (float**)malloc(_i16row * sizeof(float*));
    for (int16_t _i = 0; _i < _i16row; _i++) {
        mat->f32data[_i] = (float*)malloc(_i16col * sizeof(float));
        if (_init == InitMatWithZero) {
            for (int16_t _j = 0; _j < _i16col; _j++) {
                mat->f32data[_i][_j] = 0.0;
            }
        }
    }
    return mat;
}

Matrix* Matrix_create_with_data(const int16_t _i16row, const int16_t _i16col, float *initData, InitZero _init) {
    Matrix *mat = Matrix_create(_i16row, _i16col, _init);
    for (int16_t _i = 0; _i < _i16row; _i++) {
        for (int16_t _j = 0; _j < _i16col; _j++) {
            mat->f32data[_i][_j] = *initData++;
        }
    }
    return mat;
}

bool Matrix_is_valid(Matrix *mat) {
    if ((mat->i16row > 0) && (mat->i16row <= MATRIX_MAXIMUM_SIZE) && (mat->i16col > 0) && (mat->i16col <= MATRIX_MAXIMUM_SIZE)) {
        return true;
    } else {
        return false;
    }
}

void Matrix_set_invalid(Matrix *mat) {
    mat->i16row = -1;
    mat->i16col = -1;
}

bool Matrix_is_square(Matrix *mat) {
    return (mat->i16row == mat->i16col);
}

int16_t Matrix_get_row(Matrix *mat) {
    return mat->i16row;
}

int16_t Matrix_get_col(Matrix *mat) {
    return mat->i16col;
}

void Matrix_set_homogen(Matrix *mat, const float _val) {
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        for (int16_t _j = 0; _j < mat->i16col; _j++) {
            mat->f32data[_i][_j] = _val;
        }
    }
}

void Matrix_set_to_zero(Matrix *mat) {
    Matrix_set_homogen(mat, 0.0);
}

void Matrix_set_random(Matrix *mat, const int32_t _maxRand, const int32_t _minRand) {
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        for (int16_t _j = 0; _j < mat->i16col; _j++) {
            mat->f32data[_i][_j] = (float)((rand() % (_maxRand - _minRand + 1)) + _minRand);
        }
    }
}

void Matrix_set_diag(Matrix *mat, const float _val) {
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        for (int16_t _j = 0; _j < mat->i16col; _j++) {
            if (_i == _j) {
                mat->f32data[_i][_j] = _val;
            } else {
                mat->f32data[_i][_j] = 0.0;
            }
        }
    }
}

void Matrix_set_identity(Matrix *mat) {
    Matrix_set_diag(mat, 1.0);
}

Matrix* Matrix_copy(Matrix *mat) {
    Matrix *copy = Matrix_create(mat->i16row, mat->i16col, NoInitMatZero);
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        for (int16_t _j = 0; _j < mat->i16col; _j++) {
            copy->f32data[_i][_j] = mat->f32data[_i][_j];
        }
    }
    return copy;
}

Matrix* Matrix_transpose(Matrix *mat) {
    Matrix *trans = Matrix_create(mat->i16col, mat->i16row, NoInitMatZero);
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        for (int16_t _j = 0; _j < mat->i16col; _j++) {
            trans->f32data[_j][_i] = mat->f32data[_i][_j];
        }
    }
    return trans;
}

bool Matrix_norm_vector(Matrix *mat) {
    float _normM = 0.0;
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        for (int16_t _j = 0; _j < mat->i16col; _j++) {
            _normM += (mat->f32data[_i][_j] * mat->f32data[_i][_j]);
        }
    }

    if (_normM < 0.0) {
        return false;
    }

    if (fabs(_normM) < 0.0) {
        _normM = 0.0;
    }
    _normM = sqrt(_normM);
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        for (int16_t _j = 0; _j < mat->i16col; _j++) {
            mat->f32data[_i][_j] /= _normM;
        }
    }
    return true;
}

Matrix* Matrix_inverse(Matrix *mat) {
    Matrix *outp = Matrix_create(mat->i16row, mat->i16col, NoInitMatZero);
    Matrix *temp = Matrix_create(mat->i16row, mat->i16col, NoInitMatZero);
    Matrix_set_identity(outp);
    temp = Matrix_copy(mat);

    for (int16_t _j = 0; _j < (temp->i16row)-1; _j++) {
        for (int16_t _i = _j+1; _i < temp->i16row; _i++) {
            if (fabs(temp->f32data[_j][_j]) < 0.0) {
                Matrix_set_invalid(outp);
                return outp;
            }

            float _tempfloat = temp->f32data[_i][_j] / temp->f32data[_j][_j];

            for (int16_t _k = 0; _k < temp->i16col; _k++) {
                temp->f32data[_i][_k] -= (temp->f32data[_j][_k] * _tempfloat);
                outp->f32data[_i][_k] -= (outp->f32data[_j][_k] * _tempfloat);

                if (fabs(temp->f32data[_i][_k]) < 0.0) {
                    temp->f32data[_i][_k] = 0.0;
                }
                if (fabs(outp->f32data[_i][_k]) < 0.0) {
                    outp->f32data[_i][_k] = 0.0;
                }
            }
        }
    }

    for (int16_t _i = 1; _i < temp->i16row; _i++) {
        for (int16_t _j = 0; _j < _i; _j++) {
            temp->f32data[_i][_j] = 0.0;
        }
    }

    for (int16_t _j = (temp->i16row)-1; _j > 0; _j--) {
        for (int16_t _i = _j-1; _i >= 0; _i--) {
            if (fabs(temp->f32data[_j][_j]) < 0.0) {
                Matrix_set_invalid(outp);
                return outp;
            }

            float _tempfloat = temp->f32data[_i][_j] / temp->f32data[_j][_j];
            temp->f32data[_i][_j] -= (temp->f32data[_j][_j] * _tempfloat);
            if (fabs(temp->f32data[_i][_j]) < 0.0) {
                temp->f32data[_i][_j] = 0.0;
            }

            for (int16_t _k = (temp->i16row - 1); _k >= 0; _k--) {
                outp->f32data[_i][_k] -= (outp->f32data[_j][_k] * _tempfloat);
                if (fabs(outp->f32data[_i][_k]) < 0.0) {
                    outp->f32data[_i][_k] = 0.0;
                }
            }
        }
    }

    for (int16_t _i = 0; _i < temp->i16row; _i++) {
        if (fabs(temp->f32data[_i][_i]) < 0.0) {
            Matrix_set_invalid(outp);
            return outp;
        }

        float _tempfloat = temp->f32data[_i][_i];
        temp->f32data[_i][_i] = 1.0;

        for (int16_t _j = 0; _j < temp->i16row; _j++) {
            outp->f32data[_i][_j] /= _tempfloat;
        }
    }
    return outp;
}

void Matrix_free(Matrix *mat) {
    for (int16_t _i = 0; _i < mat->i16row; _i++) {
        free(mat->f32data[_i]);
    }
    free(mat->f32data);
    free(mat);
}

bool bMatrixIsPositiveDefinite(Matrix *this, bool checkPosSemidefinite) {
    bool _posDef, _posSemiDef;
    Matrix _temp;
    _temp.i16row = this->i16row;
    _temp.i16col = this->i16col;
    for (int16_t i = 0; i < _temp.i16row; i++)
        for (int16_t j = 0; j < _temp.i16col; j++)
            _temp.f32data[i][j] = this->f32data[i][j];

    for (int16_t _j = 0; _j < (_temp.i16row) - 1; _j++) {
        for (int16_t _i = _j + 1; _i < _temp.i16row; _i++) {
            if (fabs(_temp.f32data[_j][_j]) < float_prec_ZERO) {
                return false;
            }

            float_prec _tempfloat = _temp.f32data[_i][_j] / _temp.f32data[_j][_j];

            for (int16_t _k = 0; _k < _temp.i16col; _k++) {
                _temp.f32data[_i][_k] -= (_temp.f32data[_j][_k] * _tempfloat);
                if (fabs(_temp.f32data[_i][_k]) < float_prec_ZERO) {
                    _temp.f32data[_i][_k] = 0.0;
                }
            }
        }
    }

    _posDef = true;
    _posSemiDef = true;
    for (int16_t _i = 0; _i < _temp.i16row; _i++) {
        if (_temp.f32data[_i][_i] < float_prec_ZERO) {
            _posDef = false;
        }
        if (_temp.f32data[_i][_i] < -float_prec_ZERO) {
            _posSemiDef = false;
        }
    }

    if (checkPosSemidefinite) {
        return _posSemiDef;
    } else {
        return _posDef;
    }
}

Matrix GetDiagonalEntries(Matrix *this) {
    Matrix _temp;
    _temp.i16row = this->i16row;
    _temp.i16col = 1;

    if (this->i16row != this->i16col) {
        // Set matrix invalid (not implemented here)
        return _temp;
    }
    for (int16_t _i = 0; _i < this->i16row; _i++) {
        _temp.f32data[_i][0] = this->f32data[_i][_i];
    }
    return _temp;
}

Matrix CholeskyDec(Matrix *this) {
    float_prec _tempFloat;
    Matrix _outp;
    _outp.i16row = this->i16row;
    _outp.i16col = this->i16col;

    if (this->i16row != this->i16col) {
        // Set matrix invalid (not implemented here)
        return _outp;
    }
    for (int16_t _j = 0; _j < this->i16col; _j++) {
        for (int16_t _i = _j; _i < this->i16row; _i++) {
            _tempFloat = this->f32data[_i][_j];
            if (_i == _j) {
                for (int16_t _k = 0; _k < _j; _k++) {
                    _tempFloat -= (_outp.f32data[_i][_k] * _outp.f32data[_i][_k]);
                }
                if (_tempFloat < float_prec_ZERO) {
                    // Matrix is not positive definite
                    // Set matrix invalid (not implemented here)
                    return _outp;
                }
                if (fabs(_tempFloat) < float_prec_ZERO) {
                    _tempFloat = 0.0;
                }
                _outp.f32data[_i][_i] = sqrt(_tempFloat);
            } else {
                for (int16_t _k = 0; _k < _j; _k++) {
                    _tempFloat -= (_outp.f32data[_i][_k] * _outp.f32data[_j][_k]);
                }
                if (fabs(_outp.f32data[_j][_j]) < float_prec_ZERO) {
                    // Matrix is not positive definite
                    // Set matrix invalid (not implemented here)
                    return _outp;
                }
                _outp.f32data[_i][_j] = _tempFloat / _outp.f32data[_j][_j];
            }
        }
    }
    return _outp;
}

Matrix HouseholderTransformQR(Matrix *this, const int16_t _rowTransform, const int16_t _columnTransform) {
    float_prec _tempFloat;
    float_prec _xLen;
    float_prec _x1;
    float_prec _u1;
    float_prec _vLen2;

    Matrix _outp;
    _outp.i16row = this->i16row;
    _outp.i16col = this->i16row;
    Matrix _vectTemp;
    _vectTemp.i16row = this->i16row;
    _vectTemp.i16col = 1;

    if ((_rowTransform >= this->i16row) || (_columnTransform >= this->i16col)) {
        // Set matrix invalid (not implemented here)
        return _outp;
    }

    _x1 = this->f32data[_rowTransform][_columnTransform];
    _xLen = _x1 * _x1;
    _vLen2 = 0.0;
    for (int16_t _i = _rowTransform + 1; _i < this->i16row; _i++) {
        _vectTemp.f32data[_i][0] = this->f32data[_i][_columnTransform];
        _tempFloat = _vectTemp.f32data[_i][0] * _vectTemp.f32data[_i][0];
        _xLen += _tempFloat;
        _vLen2 += _tempFloat;
    }
    _xLen = sqrt(_xLen);

    if (_x1 < 0.0) {
        _u1 = _x1 + _xLen;
    } else {
        _u1 = _x1 - _xLen;
    }

    _vLen2 += (_u1 * _u1);
    _vectTemp.f32data[_rowTransform][0] = _u1;

    if (fabs(_vLen2) < float_prec_ZERO) {
        // x vector is collinear with basis vector e, return result = I
        // Set identity matrix (not implemented here)
    } else {
        for (int16_t _i = 0; _i < this->i16row; _i++) {
            _tempFloat = _vectTemp.f32data[_i][0];
            if (fabs(_tempFloat) > float_prec_ZERO) {
                for (int16_t _j = 0; _j < this->i16row; _j++) {
                    if (fabs(_vectTemp.f32data[_j][0]) > float_prec_ZERO) {
                        _outp.f32data[_i][_j] = _vectTemp.f32data[_j][0];
                        _outp.f32data[_i][_j] *= _tempFloat;
                        _outp.f32data[_i][_j] *= (-2.0 / _vLen2);
                    }
                }
            }
            _outp.f32data[_i][_i] += 1.0;
        }
    }
    return _outp;
}

bool QRDec(Matrix *this, Matrix *Qt, Matrix *R) {
    Matrix Qn;
    Qn.i16row = Qt->i16row;
    Qn.i16col = Qt->i16col;

    if ((this->i16row < this->i16col) || (Qt->i16row != this->i16row) || (R->i16row != this->i16row) || (R->i16col != this->i16col)) {
        // Set matrix invalid (not implemented here)
        return false;
    }
    *R = *this;
    // Set identity matrix for Qt (not implemented here)
    for (int16_t _i = 0; (_i < (this->i16row - 1)) && (_i < this->i16col - 1); _i++) {
        Qn = HouseholderTransformQR(this, _i, _i);
        if (/* Check if Qn is valid (not implemented here) */ false) {
            // Set matrix invalid (not implemented here)
            return false;
        }
        // Qt = Qn * Qt (not implemented here)
        // R = Qn * R (not implemented here)
    }
    // Rounding matrix to zero (not implemented here)
    return true;
}

Matrix BackSubtitution(Matrix *A, Matrix *B) {
    Matrix _outp;
    _outp.i16row = A->i16row;
    _outp.i16col = 1;

    if ((A->i16row != A->i16col) || (A->i16row != B->i16row)) {
        // Set matrix invalid (not implemented here)
        return _outp;
    }

    for (int16_t _i = A->i16col - 1; _i >= 0; _i--) {
        _outp.f32data[_i][0] = B->f32data[_i][0];
        for (int16_t _j = _i + 1; _j < A->i16col; _j++) {
            _outp.f32data[_i][0] -= A->f32data[_i][_j] * _outp.f32data[_j][0];
        }
        if (fabs(A->f32data[_i][_i]) < float_prec_ZERO) {
            // Set matrix invalid (not implemented here)
            return _outp;
        }
        _outp.f32data[_i][0] /= A->f32data[_i][_i];
    }

    return _outp;
}

#endif

