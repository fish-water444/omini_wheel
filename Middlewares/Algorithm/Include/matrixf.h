#ifndef MATRIXF_H
#define MATRIXF_H

#include <stdint.h>
#include "stm32f4xx.h"
#include "arm_math.h"

typedef struct
{
    uint32_t rows_; // 行数
    uint32_t cols_; // 栏数

    float *data_;                // 指向外部的buffer
    arm_matrix_instance_f32 arm; /* data */
} Matrixf_t;

void matrixfInit(Matrixf_t *m, uint32_t rows, uint32_t cols, float *buffer); // 初始化

void matrixfZero(Matrixf_t *m, uint32_t rows, uint32_t cols, float *buffer); // 全零矩阵

void matrixfEye(Matrixf_t *m, uint32_t rows, uint32_t cols, float *buffer); // 单位矩阵

arm_status matrixfAdd(Matrixf_t *dst, const Matrixf_t *a, const Matrixf_t *b); // 矩阵加法

arm_status matrixfSub(Matrixf_t *dst, const Matrixf_t *a, const Matrixf_t *b); // 矩阵减法

arm_status matrixfMult(Matrixf_t *dst, const Matrixf_t *a, const Matrixf_t *b); // 矩阵乘法

arm_status matrixfMulScalar(Matrixf_t *m, float k); // 数乘

arm_status matrixfDivScalar(Matrixf_t *m, float k); // 数除

arm_status matrixfTranspose(Matrixf_t *dst, const Matrixf_t *scr); // 转置

arm_status matrixfInverse(Matrixf_t *dst, const Matrixf_t *scr); // 逆矩阵

void matrixfCopy(Matrixf_t *dst, const Matrixf_t *src); // 拷贝

float matrixfTrace(const Matrixf_t *m); // 迹

float matrixfNorm(const Matrixf_t *m); // 范数

void matrixfBlock(Matrixf_t *dst, const Matrixf_t *src, uint32_t startRow, uint32_t startCol); // 子矩阵

#endif