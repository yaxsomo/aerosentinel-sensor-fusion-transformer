#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "matrix.h"

Matrix* scalar_add_matrix(float_prec _scalar, Matrix _mat) {
    Matrix *_outp = Matrix_create(Matrix_get_row(&_mat), Matrix_get_col(&_mat), NoInitMatZero);

    for (int16_t _i = 0; _i < Matrix_get_row(&_mat); _i++) {
        for (int16_t _j = 0; _j < Matrix_get_col(&_mat); _j++) {
            set_matrix_value(_outp, _i, _j, _scalar + get_matrix_value(_mat, _i, _j));
        }
    }
    return _outp;
}

Matrix* scalar_sub_matrix(float_prec _scalar, Matrix _mat) {
    Matrix *_outp = Matrix_create(Matrix_get_row(&_mat), Matrix_get_col(&_mat), NoInitMatZero);

    for (int16_t _i = 0; _i < Matrix_get_row(&_mat); _i++) {
        for (int16_t _j = 0; _j < Matrix_get_col(&_mat); _j++) {
            set_matrix_value(_outp, _i, _j, _scalar - get_matrix_value(_mat, _i, _j));
        }
    }
    return _outp;
}

Matrix* scalar_mul_matrix(float_prec _scalar, Matrix _mat) {
    Matrix *_outp = Matrix_create(Matrix_get_row(&_mat), Matrix_get_col(&_mat), NoInitMatZero);

    for (int16_t _i = 0; _i < Matrix_get_row(&_mat); _i++) {
        for (int16_t _j = 0; _j < Matrix_get_col(&_mat); _j++) {
            set_matrix_value(_outp, _i, _j, _scalar * get_matrix_value(_mat, _i, _j));
        }
    }
    return _outp;
}

Matrix* matrix_add_scalar(Matrix _mat, float_prec _scalar) {
    Matrix *_outp = Matrix_create(Matrix_get_row(&_mat), Matrix_get_col(&_mat), NoInitMatZero);

    for (int16_t _i = 0; _i < Matrix_get_row(&_mat); _i++) {
        for (int16_t _j = 0; _j < Matrix_get_col(&_mat); _j++) {
            set_matrix_value(_outp, _i, _j, get_matrix_value(_mat, _i, _j) + _scalar);
        }
    }
    return _outp;
}

Matrix* matrix_sub_scalar(Matrix _mat, float_prec _scalar) {
    Matrix *_outp = Matrix_create(Matrix_get_row(&_mat), Matrix_get_col(&_mat), NoInitMatZero);

    for (int16_t _i = 0; _i < Matrix_get_row(&_mat); _i++) {
        for (int16_t _j = 0; _j < Matrix_get_col(&_mat); _j++) {
            set_matrix_value(_outp, _i, _j, get_matrix_value(_mat, _i, _j) - _scalar);
        }
    }
    return _outp;
}

Matrix* matrix_mul_scalar(Matrix _mat, float_prec _scalar) {
    Matrix *_outp = Matrix_create(Matrix_get_row(&_mat), Matrix_get_col(&_mat), NoInitMatZero);

    for (int16_t _i = 0; _i < Matrix_get_row(&_mat); _i++) {
        for (int16_t _j = 0; _j < Matrix_get_col(&_mat); _j++) {
            set_matrix_value(_outp, _i, _j, get_matrix_value(_mat, _i, _j) * _scalar);
        }
    }
    return _outp;
}

Matrix* matrix_div_scalar(Matrix _mat, float_prec _scalar) {
    Matrix *_outp = Matrix_create(Matrix_get_row(&_mat), Matrix_get_col(&_mat), NoInitMatZero);

    if (fabs(_scalar) < float_prec_ZERO) {
        set_matrix_invalid(_outp);
        return _outp;
    }
    for (int16_t _i = 0; _i < Matrix_get_row(&_mat); _i++) {
        for (int16_t _j = 0; _j < Matrix_get_col(&_mat); _j++) {
            set_matrix_value(_outp, _i, _j, get_matrix_value(_mat, _i, _j) / _scalar);
        }
    }
    return _outp;
}


