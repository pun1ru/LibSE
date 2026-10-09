#include "arm_matrix.h" // 包含你的抽象层头文件
#include <math.h>
#include <stdio.h>

// 如果编译器没有定义M_PI，手动定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/*
 * =================================================================================
 * 函数: create_rotation_matrix_rpy
 * 描述: 根据标准的Roll(X), Pitch(Y), Yaw(Z)角度生成旋转矩阵.
 *      采用 Z-Y-X (Yaw -> Pitch -> Roll) 旋转顺序.
 * 参数:
 *   - R: [输出] 指向要填充的目标矩阵 (matrix_t*)
 *   - yaw_deg, pitch_deg, roll_deg: [输入] 欧拉角，单位为度.
 * =================================================================================
 */
    float test_angle_deg = 30.0f;
float result_deg ;
void create_rotation_matrix_rpy(matrix_t* R, float yaw_deg, float pitch_deg, float roll_deg)
{			
		 float test_angle_rad = test_angle_deg * M_PI / 180.0f;
    
    float y_val = sinf(test_angle_rad);
    float x_val = cosf(test_angle_rad);
    
    float result_rad = atan2f(y_val, x_val);
    float result_deg = result_rad * 180.0f / M_PI;
	
    // --- 1. 创建各单轴旋转矩阵所需的数据和实例 ---
    matrix_data_t Rx_data[9], Ry_data[9], Rz_data[9];
    matrix_t Rx, Ry, Rz;

    matrix_init(&Rx, 3, 3, Rx_data);
    matrix_init(&Ry, 3, 3, Ry_data);
    matrix_init(&Rz, 3, 3, Rz_data);

    // 将角度转为弧度
    float yaw_rad = yaw_deg * M_PI / 180.0f;
    float pitch_rad = pitch_deg * M_PI / 180.0f;
    float roll_rad = roll_deg * M_PI / 180.0f;

    float c_y = cosf(yaw_rad),   s_y = sinf(yaw_rad);
    float c_p = cosf(pitch_rad), s_p = sinf(pitch_rad);
    float c_r = cosf(roll_rad),  s_r = sinf(roll_rad);

    // --- 2. 填充各单轴旋转矩阵的数据 ---
    // Rz (Yaw)
    matrix_data_t temp_Rz_data[] = {c_y, -s_y, 0, s_y, c_y, 0, 0, 0, 1};
    memcpy(Rz.pData, temp_Rz_data, sizeof(temp_Rz_data));

    // Ry (Pitch)
    matrix_data_t temp_Ry_data[] = {c_p, 0, s_p, 0, 1, 0, -s_p, 0, c_p};
    memcpy(Ry.pData, temp_Ry_data, sizeof(temp_Ry_data));

    // Rx (Roll)
    matrix_data_t temp_Rx_data[] = {1, 0, 0, 0, c_r, -s_r, 0, s_r, c_r};
    memcpy(Rx.pData, temp_Rx_data, sizeof(temp_Rx_data));
    
    // --- 3. 按 Z-Y-X 顺序进行矩阵乘法: R = Rz * Ry * Rx ---
    matrix_data_t temp_mat_data[9];
    matrix_t temp_mat;
    matrix_init(&temp_mat, 3, 3, temp_mat_data);

    // 使用抽象层提供的宏进行乘法
    matrix_mult(&Rz, &Ry, &temp_mat);
    matrix_mult(&temp_mat, &Rx, R); // 最终结果存入 R
}


/*
 * =================================================================================
 * 函数: extract_euler_angles_rpy
 * 描述: 从旋转矩阵中提取标准的Roll(X), Pitch(Y), Yaw(Z)欧拉角.
 *      该函数必须与 create_rotation_matrix_rpy 的旋转顺序(Z-Y-X)对应.
 * 参数:
 *   - R: [输入] 指向源旋转矩阵 (matrix_t*)
 *   - yaw_deg, pitch_deg, roll_deg: [输出] 指向浮点数的指针，用于存储结果 (单位:度).
 * =================================================================================
 */
void extract_euler_angles_rpy(const matrix_t* R, float* yaw_deg, float* pitch_deg, float* roll_deg)
{
    // 从扁平化数组中获取矩阵元素
    // R->pData[row * numCols + col]
    float r11 = R->pData[0 * 3 + 0];
    float r21 = R->pData[1 * 3 + 0];
    float r31 = R->pData[2 * 3 + 0];
    float r32 = R->pData[2 * 3 + 1];
    float r33 = R->pData[2 * 3 + 2];

    float pitch_rad, yaw_rad, roll_rad;
    
    // 计算 Pitch
    pitch_rad = asinf(-r31);

    // 处理万向节死锁 (Gimbal Lock)
    if (fabsf(cosf(pitch_rad)) < 1e-6f) {
        // Pitch is close to +/- 90 degrees
        *pitch_deg = pitch_rad * 180.0f / M_PI;
        *yaw_deg = 0.0f; // Convention: set yaw to 0
        
        float r12 = R->pData[0 * 3 + 1];
        float r13 = R->pData[0 * 3 + 2];
        roll_rad = atan2f(r12, r13);
        *roll_deg = roll_rad * 180.0f / M_PI;
    } else {
        // 通用情况
        roll_rad = atan2f(r32, r33);
        yaw_rad = atan2f(r21, r11);
        
        *pitch_deg = pitch_rad * 180.0f / M_PI;
        *roll_deg = roll_rad * 180.0f / M_PI;
        *yaw_deg = yaw_rad * 180.0f / M_PI;
    }
}
