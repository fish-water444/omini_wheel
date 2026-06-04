#include "matrixf.h"

void matrixfInit(Matrixf_t *m, uint32_t rows, uint32_t cols, float *buffer)
{
    m->rows_= rows;
    m->cols_ = cols;
    m->data_ = buffer;
    arm_mat_init_f32(&m->arm, rows, cols, buffer);
}

// 全零矩阵
void matrixfZero(Matrixf_t *m, uint32_t rows, uint32_t cols, float *buffer)
{
    matrixfInit(m, rows, cols, buffer);
    memset(buffer, 0, rows * cols * sizeof(float));
}

// 单位矩阵
void matrixfEye(Matrixf_t *m, uint32_t rows, uint32_t cols, float *buffer)
{
    matrixfZero(m, rows, cols, buffer);
    uint32_t n = rows < cols ? rows : cols;
    for (uint32_t i = 0; i < n; i++)
    {
        m->data_[i * cols + i] = 1.0f;
    }
}

// 矩阵加法
arm_status matrixfAdd(Matrixf_t *dst, const Matrixf_t *a, const Matrixf_t *b)
{
    return arm_mat_add_f32(&a->arm, &b->arm, &dst->arm);
}

// 矩阵减法
arm_status matrixfSub(Matrixf_t *dst, const Matrixf_t *a, const Matrixf_t *b)
{
    return arm_mat_sub_f32(&a->arm, &b->arm, &dst->arm);
}

// 矩阵乘法
arm_status matrixfMult(Matrixf_t *dst, const Matrixf_t *a, const Matrixf_t *b)
{
    return arm_mat_mult_f32(&a->arm, &b->arm, &dst->arm);
}

// 数乘
arm_status matrixfMulScalar(Matrixf_t *m, float k)
{
    return arm_mat_scale_f32(&m->arm, k, &m->arm);
}

// 数除
arm_status matrixfDivScalar(Matrixf_t *m, float k)
{
    return arm_mat_scale_f32(&m->arm, 1.0f / k, &m->arm);
}

// 转置
arm_status matrixfTranspose(Matrixf_t *dst, const Matrixf_t *scr)
{
    return arm_mat_trans_f32(&scr->arm, &dst->arm);
}

// 逆矩阵
arm_status matrixfInverse(Matrixf_t *dst, const Matrixf_t *scr)
{
    return arm_mat_inverse_f32(&scr->arm, &dst->arm);
}

// 拷贝
void matrixfCopy(Matrixf_t *dst, const Matrixf_t *src)
{
    memcpy(dst->data_, src->data_, sizeof(float) * src->rows_ * src->cols_);
}

// 迹
float matrixfTrace(const Matrixf_t *m)
{
    float sum = 0;
    uint32_t n = (m->rows_ < m->cols_ ? m->rows_ : m->cols_);
    for (uint32_t i = 0; i < n; i++)
        sum += m->data_[i * m->cols_ + i];

    return sum;
}

// 范数
float matrixfNorm(const Matrixf_t *m)
{
    float sum = 0;
    for (uint32_t i = 0; i < m->rows_; i++)
        sum += m->data_[i] * m->data_[i];
    return sqrtf(sum);
}

// 子矩阵
void matrixfBlock(Matrixf_t *dst, const Matrixf_t *src,
                  uint32_t startRow, uint32_t startCol)
{
    for (uint32_t r = 0; r < dst->rows_; r++)
    {
        memcpy(&dst->data_[r * dst->cols_],
               &src->data_[(startRow + r) * src->cols_ + startCol],
               sizeof(float) * dst->cols_);
    }
}