/**
 * @file    diff_drive.h
 * @brief   四轮差速转向算法 - 独立驱动的 4 马达小车
 *
 * 小车布局 (俯视图):
 *
 *         前 ↑
 *    ┌─────────────┐
 *    │  M1    M3   │      M1 = 左前轮
 *    │             │      M2 = 左后轮
 *    │  M2    M4   │      M3 = 右前轮
 *    └─────────────┘      M4 = 右后轮
 *
 * 参考: protocol.md v1.0 "差速转向算法"
 */
#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 类型定义
 *===========================================================================*/

/**
 * @brief 四轮速度值 (百分比, -100 ~ 100)
 *        正值 = 正转 (前进方向), 负值 = 反转
 *        speed[0]=M1(左前), speed[1]=M2(左后),
 *        speed[2]=M3(右前), speed[3]=M4(右后)
 */
typedef struct {
    int8_t m1;   /**< 左前轮 */
    int8_t m2;   /**< 左后轮 */
    int8_t m3;   /**< 右前轮 */
    int8_t m4;   /**< 右后轮 */
} diff_drive_speeds_t;

/**
 * @brief 小车物理参数 (需实车标定)
 */
typedef struct {
    float wheel_track_cm;     /**< 左右轮距 (cm) */
    float wheel_base_cm;      /**< 前后轴距 (cm)，保留用于未来扩展 */
} diff_drive_params_t;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * @brief 初始化差速参数
 * @param p      参数结构体
 * @param track  左右轮距 (cm)
 */
void diff_drive_init(diff_drive_params_t *p, float track);

/**
 * @brief 直线运动 — 四轮同速
 * @param p       小车参数
 * @param speed   速度百分比 (0~100)，正值前进、负值后退
 * @param out     输出四轮速度
 */
void diff_drive_straight(const diff_drive_params_t *p,
                         int8_t speed, diff_drive_speeds_t *out);

/**
 * @brief 原地转向 — 左侧与右侧反向驱动
 * @param p       小车参数
 * @param speed   转向速度百分比 (1~100)
 * @param left    方向: 1=左转, 0=右转
 * @param out     输出四轮速度
 */
void diff_drive_spin(const diff_drive_params_t *p,
                     int8_t speed, int left, diff_drive_speeds_t *out);

/**
 * @brief 弧线运动 — 内外侧轮速差
 *
 * 算法:
 *   Vi = V * (R - W/2) / R     内侧轮速
 *   Vo = V * (R + W/2) / R     外侧轮速
 *
 * @param p           小车参数
 * @param speed       线速度百分比 (1~100)，正值前进、负值后退
 * @param radius_cm   转弯半径 (cm), 1~65535
 * @param left_arc    弧线方向: 1=左弧, 0=右弧
 * @param out         输出四轮速度
 */
void diff_drive_arc(const diff_drive_params_t *p,
                    int8_t speed, uint16_t radius_cm, int left_arc,
                    diff_drive_speeds_t *out);

/**
 * @brief 将 diff_drive_speeds_t 转换为 int8_t[4] 数组 (便于传给 platform_motor_set)
 */
static inline void diff_drive_to_array(const diff_drive_speeds_t *s,
                                       int8_t arr[4])
{
    arr[0] = s->m1;
    arr[1] = s->m2;
    arr[2] = s->m3;
    arr[3] = s->m4;
}

#ifdef __cplusplus
}
#endif

#endif /* DIFF_DRIVE_H */
