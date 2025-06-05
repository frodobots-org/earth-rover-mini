/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-24     yuxing       the first version
 */
#define DBG_TAG "Ellipsoidfit"
#define DBG_LVL DBG_LOG
#include "ellipsoidfit.h"

// 计算硬磁偏移
void ellipsoidfit_hard_iron_offset(Ellipsoidfit *data, int32_t *mag_bias) {
    float x_min = data->rawdata[0], x_max = data->rawdata[0];
    float y_min = data->rawdata[1], y_max = data->rawdata[1];
    float z_min = data->rawdata[2], z_max = data->rawdata[2];
    float mag_center[3] = {0};  // 计算中心点
    float max_radius = 0.0f;

    if(data->full_flag == 0)
    {
        data->nsamples = data->num;
        LOG_W("数据未满，nsamples = %d",data->nsamples);
    }
    else {
        LOG_W("数据已满，nsamples = %d",data->nsamples);
    }
    for (int i = 1; i < data->nsamples; i++) {
        if (data->rawdata[i * 3 + 0] < x_min) x_min = data->rawdata[i * 3 + 0];
        if (data->rawdata[i * 3 + 0] > x_max) x_max = data->rawdata[i * 3 + 0];
        if (data->rawdata[i * 3 + 1] < y_min) y_min = data->rawdata[i * 3 + 1];
        if (data->rawdata[i * 3 + 1] > y_max) y_max = data->rawdata[i * 3 + 1];
        if (data->rawdata[i * 3 + 2] < z_min) z_min = data->rawdata[i * 3 + 2];
        if (data->rawdata[i * 3 + 2] > z_max) z_max = data->rawdata[i * 3 + 2];
//        LOG_W("mag_data[%d]: %f %f %f\n", i, data->mag_data[i * 3 + 0], data->mag_data[i * 3 + 1], data->mag_data[i * 3 + 2]);
    }

    mag_center[0] = (x_max + x_min) / 2.0;
    mag_center[1] = (y_max + y_min) / 2.0;
    mag_center[2] = (z_max + z_min) / 2.0;

    mag_bias[0] = mag_center[0];
    mag_bias[1] = mag_center[1];
    mag_bias[2] = mag_center[2];

}

// 磁力计校准初始化
Ellipsoidfit *create_ellipsoidfit_data(int nsamples) {
    Ellipsoidfit *data = (Ellipsoidfit *)malloc(sizeof(Ellipsoidfit));
    if (data == NULL) {
        return NULL;  // 内存分配失败
    }

    data->num = 0;
    data->nsamples = nsamples;
    data->full_flag = 0;
    // 分配 rawdata
    data->rawdata = (float32_t *)malloc(nsamples * 3 * sizeof(float32_t));
    if (data->rawdata == NULL) {
        free(data);
        return NULL;
    }
    memset(data->rawdata, 0 ,nsamples * 3 * sizeof(float32_t));
//    rt_kprintf("malloc data->mag_data address:%p\n",data->mag_data);

    // 分配 D_matrix
    data->D_matrix = (float32_t *)malloc(nsamples * 9 * sizeof(float32_t));
    if (data->D_matrix == NULL) {
        free(data->rawdata);
        free(data);
        return NULL;
    }
    memset(data->D_matrix, 0 ,nsamples * 9 * sizeof(float32_t));

    // 分配 D_T_matrix
    data->D_T_matrix = (float32_t *)malloc(9 * nsamples * sizeof(float32_t));
    if (data->D_T_matrix == NULL) {
        free(data->rawdata);
        free(data->D_matrix);
        free(data);
        return NULL;
    }
    memset(data->D_T_matrix, 0 ,9 * nsamples * sizeof(float32_t));

    data->ones_f32 = (float32_t *)malloc(nsamples * sizeof(float32_t));
    if (data->ones_f32 == NULL) {
        free(data->rawdata);
        free(data->D_matrix);
        free(data->D_T_matrix);
        free(data);
        return NULL;
    }
    memset(data->ones_f32, 0 ,nsamples * sizeof(float32_t));

    return data;
}

// 释放 Ellipsoidfit 内存
void ellipsoidfit_free_calibration_data(Ellipsoidfit *data) {
    if (data != NULL) {
        if (data->rawdata != NULL) free(data->rawdata);
        if (data->D_matrix != NULL) free(data->D_matrix);
        if (data->D_T_matrix != NULL) free(data->D_T_matrix);
        if (data->ones_f32 != NULL) free(data->ones_f32);
        free(data);
    }
}

void * ellipsoid_fit(Ellipsoidfit * data){

    double offset[3] = {0};
    double gain[3] = {0};
    static double result[15] = {0};

    float32_t *D = data->D_matrix;
    float32_t *ones_f32 = data->ones_f32;
    float32_t *DT_f32 = data->D_T_matrix;
    float32_t DTMD_f32[81] ;
    float32_t DTMones_f32[9] ;
    float32_t DTMDI_f32[81] ;
    float32_t A_f32[16];
    float32_t V_f32[9] ;
    float32_t subA_f32[9];
    float32_t subAI_f32[9];
    float32_t subV_f32[3];
    float32_t subAIMSubV_f32[3];
    float32_t ofs[3];
    float32_t Tmtx_f32[16]={0};
    float32_t TmtxMA_f32[16];
    float32_t TmtxT_f32[16];
    float32_t AT_f32[16];
    uint32_t blockSize = data->nsamples;

    arm_matrix_instance_f32 DIN ;
    arm_matrix_instance_f32 DT;
    arm_matrix_instance_f32 ones ;
    arm_matrix_instance_f32 DTMD;
    arm_matrix_instance_f32 DTMones;
    arm_matrix_instance_f32 DTMDI;
    arm_matrix_instance_f32 V;
    arm_matrix_instance_f32 A ;
    arm_matrix_instance_f32 subA;
    arm_matrix_instance_f32 subAI;
    arm_matrix_instance_f32 subV;
    arm_matrix_instance_f32 subAIMSubV;
    arm_matrix_instance_f32 Tmtx;
    arm_matrix_instance_f32 TmtxMA;
    arm_matrix_instance_f32 TmtxT;
    arm_matrix_instance_f32 AT;

    uint32_t srcRows , srcColumns ;
//  int it_max = 100;
    int it_num ;
    int rot_num ;

    double subAT[9] ;
    double eig[3];//eigenvalues
    double eigV[9];//eigenVectors


    uint16_t index = 0 ;
    for (int i =0 ; i<(data->nsamples * 3);i=i+3)
    {
        float32_t x = data->rawdata[i];
        float32_t y = data->rawdata[i+1];
        float32_t z = data->rawdata[i+2];
        D[index++] = x * x;
        D[index++] = y *y;
        D[index++] = z * z;
        D[index++] = 2 * x * y ;
        D[index++] = 2 * x * z ;
        D[index++] = 2 * y * z ;
        D[index++] = 2 *x ;
        D[index++] = 2 * y ;
        D[index++] = 2 * z ;
    }
    //init Matrice D
    srcRows = data->nsamples;
    srcColumns = 9;

    arm_mat_init_f32(&DIN, srcRows, srcColumns,D);
    //init and calcute transpose of DIN as DT
    srcRows = 9;
    srcColumns = data->nsamples;
    arm_mat_init_f32(&DT, srcRows, srcColumns,DT_f32);

    arm_mat_trans_f32(&DIN, &DT);
    //init and calcute DT * D
    srcRows = 9;
    srcColumns = 9;
    arm_mat_init_f32(&DTMD, srcRows, srcColumns, DTMD_f32);

    arm_mat_mult_f32(&DT, &DIN, &DTMD);

    //init and calcute inverse of (DT * D)
    srcRows = 9;
    srcColumns = 9;
    arm_mat_init_f32(&DTMDI, srcRows, srcColumns, DTMDI_f32);

    arm_mat_inverse_f32(&DTMD, &DTMDI);
    arm_fill_f32(1.0f,ones_f32,blockSize);
    //init ones matrices
    srcRows = data->nsamples;
    srcColumns = 1;
    arm_mat_init_f32(&ones, srcRows, srcColumns, ones_f32);
    srcRows = 9;
    srcColumns = 1;
    arm_mat_init_f32(&DTMones, srcRows, srcColumns, DTMones_f32);
    arm_mat_mult_f32(&DT, &ones, &DTMones);

    srcRows = 9;
    srcColumns = 1;
    arm_mat_init_f32(&V, srcRows, srcColumns, V_f32);
    arm_mat_mult_f32(&DTMDI, &DTMones, &V);
    for(int i =0 ; i <9 ; i++)
        V_f32[i]=V.pData[i];

    A_f32[0] = V_f32[0], A_f32[1] = V_f32[3], A_f32[2] = V_f32[4], A_f32[3] = V_f32[6];
    A_f32[4] = V_f32[3], A_f32[5] = V_f32[1], A_f32[6] = V_f32[5], A_f32[7] = V_f32[7];
    A_f32[8] = V_f32[4], A_f32[9] = V_f32[5], A_f32[10] = V_f32[2], A_f32[11] = V_f32[8];
    A_f32[12] = V_f32[6], A_f32[13] = V_f32[7], A_f32[14] = V_f32[8], A_f32[15] = -1.0f;

    subA_f32[0] =  V_f32[0] , subA_f32[1] =  V_f32[3],subA_f32[2] =  V_f32[4] ;
    subA_f32[3] =  V_f32[3] , subA_f32[4] =  V_f32[1],subA_f32[5] =  V_f32[5] ;
    subA_f32[6] =  V_f32[4] , subA_f32[7] =  V_f32[5],subA_f32[8] =  V_f32[2] ;

    subV_f32[0] = V_f32[6];
    subV_f32[1] = V_f32[7];
    subV_f32[2] = V_f32[8];

    srcRows = 3;
    srcColumns = 3;
    arm_mat_init_f32(&subA, srcRows, srcColumns,subA_f32);
    srcRows = 3;
    srcColumns = 3;
    arm_mat_init_f32(&subAI, srcRows, srcColumns, subAI_f32);
    arm_mat_inverse_f32(&subA, &subAI);

    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&subV, srcRows, srcColumns, subV_f32);
    srcRows = 3;
    srcColumns = 1;
    arm_mat_init_f32(&subAIMSubV, srcRows, srcColumns, subAIMSubV_f32);
    arm_mat_mult_f32(&subAI, &subV, &subAIMSubV);
    for(int i =0 ; i <3 ; i++)
    {
        ofs[i]=-subAIMSubV.pData[i];
        offset[i] = ofs[i];
    }

    Tmtx_f32[0] = 1.0f/*,Tmtx_f32[1] = 0.0f,Tmtx_f32[2] = 0.0f,Tmtx_f32[3] = 0.0f*/;
    /*Tmtx_f32[4] = 0.0f,*/Tmtx_f32[5] = 1.0f/*,Tmtx_f32[6] = 0.0f,Tmtx_f32[7] = 0.0f*/;
    /*Tmtx_f32[8] = 0.0f,Tmtx_f32[9] = 0.0f,*/Tmtx_f32[10] = 1.0f/*,Tmtx_f32[11] = 0.0f*/;
    Tmtx_f32[12] = ofs[0],Tmtx_f32[13] = ofs[1],Tmtx_f32[14] = ofs[2],Tmtx_f32[15] = 1.0f;

    srcRows = 4;
    srcColumns = 4;
    arm_mat_init_f32(&Tmtx, srcRows, srcColumns, Tmtx_f32);

    arm_mat_init_f32(&TmtxT, srcRows, srcColumns, TmtxT_f32);
    arm_mat_trans_f32(&Tmtx, &TmtxT);
    srcRows = 4;
    srcColumns = 4;
    arm_mat_init_f32(&A, srcRows, srcColumns, A_f32);
    srcRows = 4;
    srcColumns = 4;
    arm_mat_init_f32(&TmtxMA, srcRows, srcColumns, TmtxMA_f32);

    arm_mat_mult_f32(&Tmtx, &A, &TmtxMA);

    arm_mat_init_f32(&AT, srcRows, srcColumns, AT_f32);

    arm_mat_mult_f32(&TmtxMA, &TmtxT, &AT);
    for(int i =0 ; i <16 ; i++)
        AT_f32[i]=AT.pData[i];

    subAT[0] =(double)(AT_f32[0]/-AT_f32[15]),subAT[1] = (double)(AT_f32[1]/-AT_f32[15]),subAT[2] = (double)(AT_f32[2]/-AT_f32[15]);
    subAT[3] = (double)(AT_f32[4]/-AT_f32[15]),subAT[4] = (double)(AT_f32[5]/-AT_f32[15]),subAT[5] = (double)(AT_f32[6]/-AT_f32[15]);
    subAT[6] = (double)(AT_f32[8]/-AT_f32[15]),subAT[7] = (double)(AT_f32[9]/-AT_f32[15]),subAT[8] = (double)(AT_f32[10]/-AT_f32[15]);

    jacobi_eigenvalue(3,subAT,100,eigV,eig,&it_num,&rot_num);

    for (int i= 0 ; i<3;i++)
    {
        gain[i] = sqrt(1/eig[i]);
        LOG_W("gain[%d] = %f\n",i,(float)gain[i]);
    }


    for(int i = 0 ; i < 3 ; i++)
        result[i] = offset[i] ;

    for(int i = 3; i < 6 ; i++)
        result[i] = gain[i-3];

    for(int i = 6; i < 15 ; i++)
        result[i] = eigV[i-6];

    for(int i= 0 ; i<15;i++)
    {
        LOG_W("result[%d] = %f\n",i,(float)result[i]);

    }
    return result;

}

void refine_3D_fit(double *gain, double *rotM){

    double m=0;
    int rm=0,cm = 0;
    double t[3] = {0};
    int i_matrix[2]={0};
    double rotM_3x3[3][3]={0};

    for(int j=0;j<3;j++)
        for(int i=0; i<3;i++)
            rotM_3x3[i][j]=rotM[j*3+i];

    for (int r=0; r<3;r++){
        for(int c=0; c<3;c++){
            if(fabs(rotM_3x3[r][c])>m){
                m = fabs(rotM_3x3[r][c]);
                rm = r;
                cm = c;
            }
        }
    }

    if (rm != cm){

        for(int i = 0; i <3;i++){
            t[i] = rotM_3x3[i][cm];
        }
        for(int i = 0; i <3;i++){
            rotM_3x3[i][cm] = rotM_3x3[i][rm];
        }
        for(int i = 0; i <3;i++){
            rotM_3x3[i][rm] = t[i];
        }
        t[0] = gain[cm];
        gain[cm] = gain[rm];
        gain[rm]=t[0];
    }
    switch (rm)
    {
        case 0:
            i_matrix[0] = 1;
            i_matrix[1] = 2;
            break;
        case 1:
            i_matrix[0] = 0;
            i_matrix[1] = 2;
            break;
        case 2:
            i_matrix[0] = 0;
            i_matrix[1] = 1;
            break;
    }
    m=0;
    rm=0;
    cm=0;
    for (int r=0; r<2;r++){
        for(int c=0; c<2;c++){
            if(fabs(rotM_3x3[i_matrix[r]][i_matrix[c]])>m){
                m = fabs(rotM_3x3[i_matrix[r]][i_matrix[c]]);
                rm = i_matrix[r];
                cm = i_matrix[c];
            }
        }
    }
    if (rm != cm){

        for(int i = 0; i <3;i++){
            t[i] = rotM_3x3[i][cm];
        }
        for(int i = 0; i <3;i++){
            rotM_3x3[i][cm] = rotM_3x3[i][rm];
        }
        for(int i = 0; i <3;i++){
            rotM_3x3[i][rm] = t[i];
        }
        t[0] = gain[cm];
        gain[cm] = gain[rm];
        gain[rm]=t[0];
    }
    for(int i=0;i<3;i++){
        if(rotM_3x3[i][i]<0){
            for(int j=0;j<3;j++)
                rotM_3x3[j][i] = rotM_3x3[j][i]*(-1);
        }
    }
    for(int j=0;j<3;j++)
        for(int i=0; i<3;i++)
            rotM[j*3+i] = rotM_3x3[i][j];


}

void handle_calibrate_magnetometer(double B_raw[3], double B_calibrated[3],
                            double offset[3], double gain[3], double eigV[3 * 3]) {
    double B_corrected[3];
    double B_scaled[3];
    double B_final[3] = {0, 0, 0};

    // Step 1: Subtract offset
    for (int i = 0; i < 3; i++) {
        B_corrected[i] = B_raw[i] - offset[i];
    }
    //检索缩放值是否为0
    if(gain[0] == 0 || gain[1] == 0 || gain[2] == 0)
    {
        for (int i = 0; i < 3; i++) {
            B_calibrated[i] = B_corrected[i];
        }
        return;
    }
    // Step 2: Apply gain scaling
    for (int i = 0; i < 3; i++) {
        B_scaled[i] = gain[i] * B_corrected[i];
    }

//    // Step 3: Apply rotation (eigV)
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            B_final[i] += eigV[i * 3 + j] * B_scaled[j];
//        }
//    }

    // Output calibrated data
    for (int i = 0; i < 3; i++) {
        B_calibrated[i] = B_scaled[i];
    }
}

void calibrate_sensor(double* rawdata, double* calibrated, double* offset,
        double* gain, double* rotM) {
    double x = rawdata[0]   - offset[0];
    double y = rawdata[1] - offset[1];
    double z = rawdata[2] - offset[2];

    // 应用旋转矩阵（列优先）
    double x_rot = rotM[0]*x + rotM[3]*y + rotM[6]*z;
    double y_rot = rotM[1]*x + rotM[4]*y + rotM[7]*z;
    double z_rot = rotM[2]*x + rotM[5]*y + rotM[8]*z;

    // 应用增益
    calibrated[0]   = x_rot * gain[0];
    calibrated[1] = y_rot * gain[1];
    calibrated[2] = z_rot * gain[2];

}

/******************************************************************************/
void jacobi_eigenvalue ( int n, double a[], int it_max, double v[],
  double d[], int *it_num, int *rot_num )

/******************************************************************************/
/*
  Purpose:

    JACOBI_EIGENVALUE carries out the Jacobi eigenvalue iteration.

  Discussion:

    This function computes the eigenvalues and eigenvectors of a
    real symmetric matrix, using Rutishauser's modfications of the classical
    Jacobi rotation method with threshold pivoting.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    17 September 2013

  Author:

    C version by John Burkardt

  Parameters:

    Input, int N, the order of the matrix.

    Input, double A[N*N], the matrix, which must be square, real,
    and symmetric.

    Input, int IT_MAX, the maximum number of iterations.

    Output, double V[N*N], the matrix of eigenvectors.

    Output, double D[N], the eigenvalues, in descending order.

    Output, int *IT_NUM, the total number of iterations.

    Output, int *ROT_NUM, the total number of rotations.
*/
{
  double *bw;
  double c;
  double g;
  double gapq;
  double h;
  int i;
  int j;
  int k;
  int l;
  int m;
  int p;
  int q;
  double s;
  double t;
  double tau;
  double term;
  double termp;
  double termq;
  double theta;
  double thresh;
  double w;
  double *zw;

  r8mat_identity ( n, v );

  r8mat_diag_get_vector ( n, a, d );

  bw = ( double * ) malloc ( n * sizeof ( double ) );
  zw = ( double * ) malloc ( n * sizeof ( double ) );

  for ( i = 0; i < n; i++ )
  {
    bw[i] = d[i];
    zw[i] = 0.0;
  }
  *it_num = 0;
  *rot_num = 0;

  while ( *it_num < it_max )
  {
    *it_num = *it_num + 1;
/*
  The convergence threshold is based on the size of the elements in
  the strict upper triangle of the matrix.
*/
    thresh = 0.0;
    for ( j = 0; j < n; j++ )
    {
      for ( i = 0; i < j; i++ )
      {
        thresh = thresh + a[i+j*n] * a[i+j*n];
      }
    }

    thresh = sqrt ( thresh ) / ( double ) ( 4 * n );

    if ( thresh == 0.0 )
    {
      break;
    }

    for ( p = 0; p < n; p++ )
    {
      for ( q = p + 1; q < n; q++ )
      {
        gapq = 10.0 * fabs ( a[p+q*n] );
        termp = gapq + fabs ( d[p] );
        termq = gapq + fabs ( d[q] );
/*
  Annihilate tiny offdiagonal elements.
*/
        if ( 4 < *it_num &&
             termp == fabs ( d[p] ) &&
             termq == fabs ( d[q] ) )
        {
          a[p+q*n] = 0.0;
        }
/*
  Otherwise, apply a rotation.
*/
        else if ( thresh <= fabs ( a[p+q*n] ) )
        {
          h = d[q] - d[p];
          term = fabs ( h ) + gapq;

          if ( term == fabs ( h ) )
          {
            t = a[p+q*n] / h;
          }
          else
          {
            theta = 0.5 * h / a[p+q*n];
            t = 1.0 / ( fabs ( theta ) + sqrt ( 1.0 + theta * theta ) );
            if ( theta < 0.0 )
            {
              t = - t;
            }
          }
          c = 1.0 / sqrt ( 1.0 + t * t );
          s = t * c;
          tau = s / ( 1.0 + c );
          h = t * a[p+q*n];
/*
  Accumulate corrections to diagonal elements.
*/
          zw[p] = zw[p] - h;
          zw[q] = zw[q] + h;
          d[p] = d[p] - h;
          d[q] = d[q] + h;

          a[p+q*n] = 0.0;
/*
  Rotate, using information from the upper triangle of A only.
*/
          for ( j = 0; j < p; j++ )
          {
            g = a[j+p*n];
            h = a[j+q*n];
            a[j+p*n] = g - s * ( h + g * tau );
            a[j+q*n] = h + s * ( g - h * tau );
          }

          for ( j = p + 1; j < q; j++ )
          {
            g = a[p+j*n];
            h = a[j+q*n];
            a[p+j*n] = g - s * ( h + g * tau );
            a[j+q*n] = h + s * ( g - h * tau );
          }

          for ( j = q + 1; j < n; j++ )
          {
            g = a[p+j*n];
            h = a[q+j*n];
            a[p+j*n] = g - s * ( h + g * tau );
            a[q+j*n] = h + s * ( g - h * tau );
          }
/*
  Accumulate information in the eigenvector matrix.
*/
          for ( j = 0; j < n; j++ )
          {
            g = v[j+p*n];
            h = v[j+q*n];
            v[j+p*n] = g - s * ( h + g * tau );
            v[j+q*n] = h + s * ( g - h * tau );
          }
          *rot_num = *rot_num + 1;
        }
      }
    }

    for ( i = 0; i < n; i++ )
    {
      bw[i] = bw[i] + zw[i];
      d[i] = bw[i];
      zw[i] = 0.0;
    }
  }
/*
  Restore upper triangle of input matrix.
*/
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < j; i++ )
    {
      a[i+j*n] = a[j+i*n];
    }
  }
/*
  Ascending sort the eigenvalues and eigenvectors.
*/
  for ( k = 0; k < n - 1; k++ )
  {
    m = k;
    for ( l = k + 1; l < n; l++ )
    {
      if ( d[l] < d[m] )
      {
        m = l;
      }
    }

    if ( m != k )
    {
      t    = d[m];
      d[m] = d[k];
      d[k] = t;
      for ( i = 0; i < n; i++ )
      {
        w        = v[i+m*n];
        v[i+m*n] = v[i+k*n];
        v[i+k*n] = w;
      }
    }
  }

  free ( bw );
  free ( zw );

  return;
}
/******************************************************************************/

void r8mat_diag_get_vector ( int n, double a[], double v[] )

/******************************************************************************/
/*
  Purpose:

    R8MAT_DIAG_GET_VECTOR gets the value of the diagonal of an R8MAT.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    15 July 2013

  Author:

    John Burkardt

  Parameters:

    Input, int N, the number of rows and columns of the matrix.

    Input, double A[N*N], the N by N matrix.

    Output, double V[N], the diagonal entries
    of the matrix.
*/
{
  int i;

  for ( i = 0; i < n; i++ )
  {
    v[i] = a[i+i*n];
  }

  return;
}
/******************************************************************************/

void r8mat_identity  ( int n, double a[] )

/******************************************************************************/
/*
  Purpose:

    R8MAT_IDENTITY sets an R8MAT to the identity matrix.

  Discussion:

    An R8MAT is a doubly dimensioned array of R8 values, stored as a vector
    in column-major order.

  Licensing:

    This code is distributed under the GNU LGPL license.

  Modified:

    06 September 2005

  Author:

    John Burkardt

  Parameters:

    Input, int N, the order of A.

    Output, double A[N*N], the N by N identity matrix.
*/
{
  int i;
  int j;
  int k;

  k = 0;
  for ( j = 0; j < n; j++ )
  {
    for ( i = 0; i < n; i++ )
    {
      if ( i == j )
      {
        a[k] = 1.0;
      }
      else
      {
        a[k] = 0.0;
      }
      k = k + 1;
    }
  }

  return;
}
