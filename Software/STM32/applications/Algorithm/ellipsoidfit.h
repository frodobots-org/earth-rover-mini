/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-24     yuxing       the first version
 */
#ifndef APPLICATIONS_ALGORITHM_ELLIPSOIDFIT_H_
#define APPLICATIONS_ALGORITHM_ELLIPSOIDFIT_H_

#include "math.h"
#include "arm_math.h"
#include <stdio.h>
#include <stdlib.h>
#include "ulog.h"

//定义磁力计校准结构体
typedef struct {
    float *rawdata;
    float *D_matrix;
    float *D_T_matrix;
    float *ones_f32;
    int16_t num;
    int16_t nsamples;
    uint8_t full_flag;
} Ellipsoidfit;

void ellipsoidfit_hard_iron_offset(Ellipsoidfit *data, int32_t *mag_bias);
Ellipsoidfit *create_ellipsoidfit_data(int nsamples);
void ellipsoidfit_free_calibration_data(Ellipsoidfit *data);
void * ellipsoid_fit(Ellipsoidfit * data);
void refine_3D_fit(double *gain, double *rotM);
void handle_calibrate_magnetometer(double B_raw[3], double B_calibrated[3],
                            double offset[3], double gain[3], double eigV[3*3]);
void calibrate_sensor(double* rawdata, double* calibrated, double* offset,
        double* gain, double* rotM);
void jacobi_eigenvalue ( int n, double a[], int it_max, double v[],
  double d[], int *it_num, int *rot_num );
void r8mat_diag_get_vector ( int n, double a[], double v[] );
void r8mat_identity ( int n, double a[] );



#endif /* APPLICATIONS_ALGORITHM_ELLIPSOIDFIT_H_ */
