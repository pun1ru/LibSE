#ifndef _ARM_MATRIX_H_
#define _ARM_MATRIX_H_

#include <arm_math.h>

/**
 * \brief 矩阵数据类型定义
 */
typedef float matrix_data_t;

/**
 * \brief 矩阵类型定义 
 */
typedef arm_matrix_instance_f32 matrix_t ;

/**
 * \brief 矩阵初始化
 * \param[in] mat 矩阵类型指针
 * \param[in] col 矩阵列数
 * \param[in] row 矩阵行数
 * \param[in] buffer 矩阵数据缓冲
 */
#define matrix_init(mat, row, col, buffer)   arm_mat_init_f32(mat, row, col, buffer)

/**
 * \brief 矩阵乘法 Res = A * B
 * \param[in] pA 输入矩阵A指针
 * \param[in] pB 输入矩阵B指针
 * \param[out] pRes 结果矩阵指针
 */
#define matrix_mult(pA, pB, pRes)   arm_mat_mult_f32(pA, pB, pRes)


/**
 * \brief 矩阵减法 Res = A - B
 * \param[in] pA 输入矩阵A指针
 * \param[in] pB 输入矩阵B指针
 * \param[out] pRes 结果矩阵指针
 */
#define matrix_sub(pA, pB, pRes)   arm_mat_sub_f32(pA, pB, pRes)

//矩阵求逆
#define matrix_inv    arm_mat_inverse_f32
/**
 * \brief 矩阵转置 Res = A^T
 * \param[in] pSrc 输入矩阵指针
 * \param[out] pDst 结果矩阵指针
 因为矩阵求逆计算量大，对于角度矩阵，用求逆代替转置
 */
#define matrix_trans(pSrc, pDst)   arm_mat_trans_f32(pSrc, pDst)

#endif
