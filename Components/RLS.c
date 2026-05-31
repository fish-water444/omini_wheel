#include "RLS.h"

#define RLS_DEN_EPS (1e-6f)
#define RLS_P_DIAG_MIN (1e-4f)     // P 对角线最小值，防塌陷
#define RLS_P_TRACE_MIN (0.1f)     // trace 太小认为退化（归一化后一般不会太大）
#define RLS_P_TRACE_MAX (1e8f)     // trace 太大认为发散
#define RLS_P_SOFT_RESET_K (0.02f) // 软重置强度：0.01~0.05 比较常用
#define RLS_GAIN_CLAMP (100.0f)    // 单个增益分量限幅（保守一点）
#define RLS_ERR_CLAMP (1e4f)       // 误差限幅（按你 POWER_SCALE 后的量级调）

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}
// 初始化函数（等价于 C++ 构造函数）
void rlsInit(RLS_t *rls, uint32_t dim, float delta, float lambda, float *trans_buf, float *gain_buf, float *params_buf, float *default_buf)
{
    if (rls->inited_ != 0 && rls->inited_ != 1)
    {
        rls->inited_ = 0;
    }
    if (rls->inited_) // 仅初始化一次
    {
        return;
    }

    // 参数验证
    configASSERT(lambda >= 0.0f && lambda <= 1.0f); // 检查条件：lambda 必须在 [0.0, 1.0] 范围内
    configASSERT(delta > 0.0f);                     // 检查条件：delta 必须大于 0

    rls->dim_ = dim;
    rls->lambda_ = lambda;
    rls->delta_ = delta;
    rls->updateCnt_ = 0;

    rls->transMatrix_buffer_ = trans_buf;
    rls->gainVector_buffer_ = gain_buf;
    rls->paramsVector_buffer_ = params_buf;
    rls->defaultParamsVector_buffer_ = default_buf;

    // transMatrix = eye * delta
    matrixfEye(&rls->transMatrix_, dim, dim, trans_buf);
    matrixfMulScalar(&rls->transMatrix_, delta);

    matrixfZero(&rls->gainVector_, dim, 1, gain_buf);
    matrixfZero(&rls->paramsVector_, dim, 1, params_buf);
    matrixfZero(&rls->defaultParams_, dim, 1, default_buf);
    rls->inited_ = 1; // rls已完成初始化
}

void rlsReset(RLS_t *rls)
{ // 重新初始化矩阵
    matrixfEye(&rls->transMatrix_, rls->dim_, rls->dim_, rls->transMatrix_buffer_);
    matrixfMulScalar(&rls->transMatrix_, rls->delta_);

    matrixfZero(&rls->gainVector_, rls->dim_, 1, rls->gainVector_buffer_);
    matrixfZero(&rls->paramsVector_, rls->dim_, 1, rls->paramsVector_buffer_);

    rls->updateCnt_ = 0;
}
void rlsSetParams(RLS_t *rls, const Matrixf_t *updatedParams)
{
    matrixfCopy(&rls->paramsVector_, updatedParams);
    matrixfCopy(&rls->defaultParams_, updatedParams);
}

// // RLS公式更新
const Matrixf_t *rlsUpdate(RLS_t *rls, const Matrixf_t *sample, float actualOutput)
{
    if (rls == NULL || sample == NULL)
        return NULL;

    if (!rls->inited_)
        return &rls->paramsVector_;

    if (sample->rows_ != rls->dim_ || sample->cols_ != 1)
        return &rls->paramsVector_;

    if (rls->transMatrix_.data_ == NULL ||
        rls->gainVector_.data_ == NULL ||
        rls->paramsVector_.data_ == NULL)
        return &rls->paramsVector_;

    Matrixf_t Px, xTPx, xT, KxT, KxTP, tmp33, tmp11;

    // 说明：dim=3 时这些静态 buffer 足够；如果你未来 dim 可能变化，建议用 dim*dim 的最大值做宏
    static float Px_buf[3 * 1];
    static float xTPx_buf[1 * 1];
    static float xT_buf[1 * 3];
    static float KxT_buf[3 * 3];
    static float KxTP_buf[3 * 3];
    static float tmp33_buf[3 * 3];
    static float tmp11_buf[1 * 1];

    matrixfInit(&Px, rls->dim_, 1, Px_buf);
    matrixfInit(&xTPx, 1, 1, xTPx_buf);
    matrixfInit(&xT, 1, rls->dim_, xT_buf);
    matrixfInit(&KxT, rls->dim_, rls->dim_, KxT_buf);
    matrixfInit(&KxTP, rls->dim_, rls->dim_, KxTP_buf);
    matrixfInit(&tmp33, rls->dim_, rls->dim_, tmp33_buf);
    matrixfInit(&tmp11, 1, 1, tmp11_buf);

    // xT = x^T
    matrixfTranspose(&xT, sample);

    // Px = P * x
    matrixfMult(&Px, &rls->transMatrix_, sample);

    // xTPx = x^T * (P*x)
    matrixfMult(&xTPx, &xT, &Px);

    // K = (P*x) / (lambda + x^T P x)
    float denom = rls->lambda_ + xTPx.data_[0];
    if (denom < 1e-6f)
        denom = 1e-6f;

    matrixfCopy(&rls->gainVector_, &Px);
    matrixfDivScalar(&rls->gainVector_, denom);

    // 误差 e = y - x^T theta
    matrixfMult(&tmp11, &xT, &rls->paramsVector_);
    float err = actualOutput - tmp11.data_[0];

    // theta = theta + K * e
    Matrixf_t Ke;
    static float Ke_buf[3 * 1];
    matrixfInit(&Ke, rls->dim_, 1, Ke_buf);

    matrixfCopy(&Ke, &rls->gainVector_);
    matrixfMulScalar(&Ke, err);
    matrixfAdd(&rls->paramsVector_, &rls->paramsVector_, &Ke);

    // P = (P - K*x^T*P) / lambda
    // KxT = K * x^T
    matrixfMult(&KxT, &rls->gainVector_, &xT);

    // KxTP = (K*x^T) * P
    matrixfMult(&KxTP, &KxT, &rls->transMatrix_);

    // tmp33 = P - KxTP
    matrixfSub(&tmp33, &rls->transMatrix_, &KxTP);

    // P = tmp33 / lambda
    float lam = rls->lambda_;
    if (lam < 1e-3f)
        lam = 1e-3f;
    matrixfCopy(&rls->transMatrix_, &tmp33);
    matrixfDivScalar(&rls->transMatrix_, lam);

    // 防退化：trace 太小则轻微回拉
    float trace = 0.0f;
    for (int i = 0; i < rls->dim_; i++)
        trace += rls->transMatrix_.data_[i * rls->dim_ + i];

    if (trace < 1e-3f)
    {
        matrixfEye(&rls->transMatrix_, rls->dim_, rls->dim_, rls->transMatrix_buffer_);
        matrixfMulScalar(&rls->transMatrix_, rls->delta_);
    }

    rls->updateCnt_++;
    return &rls->paramsVector_;
}
// const Matrixf_t *rlsUpdate(RLS_t *rls, const Matrixf_t *sample, float actualOutput)
// {
//     if (rls == NULL || sample == NULL)
//         return NULL;

//     if (!rls->inited_)
//         return &rls->paramsVector_; // 或直接 return NULL

//     if (sample->rows_ != rls->dim_ || sample->cols_ != 1)
//         return &rls->paramsVector_;

//     if (rls->transMatrix_.data_ == NULL ||
//         rls->gainVector_.data_ == NULL ||
//         rls->paramsVector_.data_ == NULL)
//         return &rls->paramsVector_;
//     // 以上的操作都是为了在数据不正常的时候提前结束函数，防止有错误数据进入了RLS更新
//     Matrixf_t temp1, temp2, temp3, sampleT;
//     float temp_float;
//     // 分配临时缓冲区

//     static float temp1_buf[3 * 1];
//     static float temp2_buf[1 * 1];
//     static float temp3_buf[3 * 3];
//     static float sampleT_buf[1 * 3];

//     matrixfInit(&temp1, rls->dim_, 1, temp1_buf);
//     matrixfInit(&temp2, 1, 1, temp2_buf);
//     matrixfInit(&temp3, rls->dim_, rls->dim_, temp3_buf);
//     matrixfInit(&sampleT, 1, rls->dim_, sampleT_buf); // 在计算转置之前，先给 sampleT 分配 buffer
//     // 计算转置: sampleT = sampleVector^T
//     matrixfTranspose(&sampleT, sample);

//     // 计算增益向量: gainVector = (transMatrix * sample) / (1 + (sampleT * transMatrix * sample)[0][0] / lambda) / lambda
//     matrixfMult(&temp1, &rls->transMatrix_, sample); // temp1 = transMatrix * sample

//     matrixfMult(&temp2, &sampleT, &temp1); // temp2 = sampleT * temp1 = sampleT * transMatrix * sample

//     float denominator = 1.0f + temp2.data_[0] / rls->lambda_; // 分母计算

//     matrixfCopy(&rls->gainVector_, &temp1); // gainVector = temp1 / denominator / lambda
//     matrixfDivScalar(&rls->gainVector_, denominator);
//     matrixfDivScalar(&rls->gainVector_, rls->lambda_);

//     // 更新参数向量: paramsVector += gainVector * (actualOutput - (sampleT * paramsVector)[0][0])
//     matrixfMult(&temp2, &sampleT, &rls->paramsVector_); // temp2 = sampleT * paramsVector
//     float error = actualOutput - temp2.data_[0];

//     matrixfCopy(&temp1, &rls->gainVector_); // temp1 = gainVector * error
//     matrixfMulScalar(&temp1, error);

//     matrixfAdd(&rls->paramsVector_, &rls->paramsVector_, &temp1); // paramsVector += temp1

//     // 更新转移矩阵: transMatrix = (transMatrix - gainVector * sampleT * transMatrix) / lambda
//     matrixfMult(&temp3, &rls->gainVector_, &sampleT); // temp3 = gainVector * sampleT
//     matrixfMult(&temp2, &temp3, &rls->transMatrix_);  // temp2 = temp3 * transMatrix = gainVector * sampleT * transMatrix
//     matrixfSub(&temp3, &rls->transMatrix_, &temp2);   // temp3 = transMatrix - temp2

//     matrixfCopy(&rls->transMatrix_, &temp3); // transMatrix = temp3 / lambda
//     matrixfDivScalar(&rls->transMatrix_, rls->lambda_);
//     // P 矩阵软回拉，防止退化
//     float trace = 0.0f;
//     for (int i = 0; i < rls->dim_; i++)
//     {
//         trace += rls->transMatrix_.data_[i * rls->dim_ + i];
//     }

//     if (trace < 1.0f)
//     {
//         matrixfMulScalar(&rls->transMatrix_, 10.0f); // 温和拉回
//     }

//     // 更新计数和时间
//     rls->updateCnt_++;

//     return &rls->paramsVector_;
// }

void rlsSetParamVector(RLS_t *rls, const Matrixf_t *params)
{
    matrixfCopy(&rls->paramsVector_, params);
    matrixfCopy(&rls->defaultParams_, params);
}

const Matrixf_t *rlsGetParamsVector(RLS_t *rls)
{
    return &rls->paramsVector_;
}
