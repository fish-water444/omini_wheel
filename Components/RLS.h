#ifndef RLS_H
#define RLS_H

#include "stdint.h"
#include "math.h"
#include "matrixf.h"

typedef struct
{
    uint32_t dim_;            // RLS维度
    float lambda_;            // 遗忘因子
    float delta_;             // 初始P矩阵增益
    Matrixf_t transMatrix_;   // dim x dim 矩阵
    Matrixf_t gainVector_;    // dim x 1
    Matrixf_t paramsVector_;  // dim x 1
    Matrixf_t defaultParams_; // dim x 1

    uint32_t updateCnt_;

    // 缓冲区指针（需要外部分配）
    float *transMatrix_buffer_;
    float *gainVector_buffer_;
    float *paramsVector_buffer_;
    float *defaultParamsVector_buffer_;
    uint8_t inited_;//rls初始化标志位

} RLS_t;

extern RLS_t rls_;

// 初始化函数
void rlsInit(RLS_t *rls, uint32_t dim, float delta, float lambda, float *trans_buf, float *gain_buf, float *params_buf, float *default_buf);

// 重新初始化矩阵
void rlsReset(RLS_t *rls);
void rlsSetParams(RLS_t *rls, const Matrixf_t *updatedParams);

// RLS公式更新
const Matrixf_t *rlsUpdate(RLS_t *rls, const Matrixf_t *sample, float actualOutput);
void rlsSetParamVector(RLS_t *rls, const Matrixf_t *params);
const Matrixf_t *rlsGetParamsVector(RLS_t *rls);

#endif

#ifndef configASSERT
#define configASSERT(x) do { if(!(x)) { \
        /* 打个断点也行 */ \
        __BKPT(0); \
        while(1); \
    } } while(0)
#endif
