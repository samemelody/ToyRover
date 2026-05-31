/**
 * @file    diff_drive.c
 * @brief   四轮差速转向算法实现
 */

#include "diff_drive.h"
#include <stddef.h>

/*===========================================================================
 * 参数初始化
 *===========================================================================*/

void diff_drive_init(diff_drive_params_t *p, float track)
{
    if (p == NULL) return;
    p->wheel_track_cm = track;
    p->wheel_base_cm  = 0.0f;  /* 当前未使用，预留 */
}

/*===========================================================================
 * 直线运动 — 四轮同速
 *===========================================================================*/

void diff_drive_straight(const diff_drive_params_t *p,
                         int8_t speed, diff_drive_speeds_t *out)
{
    (void)p;
    if (out == NULL) return;

    int8_t spd = speed;
    /* 钳位 */
    if (spd > 100)  spd = 100;
    if (spd < -100) spd = -100;

    out->m1 = spd;
    out->m2 = spd;
    out->m3 = spd;
    out->m4 = spd;
}

/*===========================================================================
 * 原地转向
 *
 * 左转: M1=反转, M2=反转, M3=正转, M4=正转
 * 右转: M1=正转, M2=正转, M3=反转, M4=反转
 *===========================================================================*/

void diff_drive_spin(const diff_drive_params_t *p,
                     int8_t speed, int left, diff_drive_speeds_t *out)
{
    (void)p;
    if (out == NULL) return;

    int8_t spd = speed;
    if (spd > 100)  spd = 100;
    if (spd < 0)    spd = 1;     /* 至少 1%，防止死区 */

    if (left) {
        /* 左转: 左侧反转, 右侧正转 */
        out->m1 = (int8_t)(-spd);
        out->m2 = (int8_t)(-spd);
        out->m3 = spd;
        out->m4 = spd;
    } else {
        /* 右转: 左侧正转, 右侧反转 */
        out->m1 = spd;
        out->m2 = spd;
        out->m3 = (int8_t)(-spd);
        out->m4 = (int8_t)(-spd);
    }
}

/*===========================================================================
 * 弧线运动 — 内外侧轮速差
 *
 * 公式:
 *   Vi = V * (R - W/2) / R
 *   Vo = V * (R + W/2) / R
 *
 *   左弧: 内侧=左侧, 外侧=右侧
 *   右弧: 内侧=右侧, 外侧=左侧
 *
 * R → ∞ (直线):  Vi ≈ Vo ≈ V
 * R → 0:        死区保护, R 最小值 1cm
 *===========================================================================*/

/**
 * @brief 浮点钳位到 [-100, 100]
 */
static inline float clamp_speed(float v)
{
    if (v > 100.0f) return 100.0f;
    if (v < -100.0f) return -100.0f;
    return v;
}

void diff_drive_arc(const diff_drive_params_t *p,
                    int8_t speed, uint16_t radius_cm, int left_arc,
                    diff_drive_speeds_t *out)
{
    if (p == NULL || out == NULL) return;

    float V = (float)speed;
    float R = (float)radius_cm;
    float W = p->wheel_track_cm;

    /* R=0 保护 */
    if (R < 1.0f) R = 1.0f;

    float inner = V * (R - W * 0.5f) / R;
    float outer = V * (R + W * 0.5f) / R;

    inner = clamp_speed(inner);
    outer = clamp_speed(outer);

    if (left_arc) {
        /* 左弧: 内侧=左侧(M1,M2), 外侧=右侧(M3,M4) */
        out->m1 = (int8_t)inner;
        out->m2 = (int8_t)inner;
        out->m3 = (int8_t)outer;
        out->m4 = (int8_t)outer;
    } else {
        /* 右弧: 内侧=右侧(M3,M4), 外侧=左侧(M1,M2) */
        out->m1 = (int8_t)outer;
        out->m2 = (int8_t)outer;
        out->m3 = (int8_t)inner;
        out->m4 = (int8_t)inner;
    }
}
