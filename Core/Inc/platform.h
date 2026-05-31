/**
 * @file    platform.h
 * @brief   平台抽象层 - 硬件依赖接口
 *
 * 集成指南:
 *   将本文件中的接口函数在 STM32 工程中实现，替换 proto_cmd.c 中的 __weak 弱函数。
 *
 *   方案 A (推荐): 创建一个 platform_stm32.c，实现所有函数，
 *                  链接时会自动覆盖 proto_cmd.c 中的弱符号。
 *
 *   方案 B:         删掉 proto_cmd.c 中的 __weak 函数，
 *                  在集成文件中直接实现。
 *
 * 需要实现的接口:
 *   - platform_motor_set()   设置四路 PWM
 *   - platform_motor_stop()  急停
 *   - platform_send_frame()  通过 BLE/UART 发送数据帧
 *   - platform_get_tick_ms() 获取系统毫秒 tick
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 电机控制接口
 *===========================================================================*/

/**
 * @brief 设置四轮电机速度
 * @param speeds  [M1, M2, M3, M4]
 *                M1=左前, M2=左后, M3=右前, M4=右后
 *                正值 = 正转 (前进方向), 负值 = 反转, 0 = 停止
 *                范围: -100 ~ 100 (百分比)
 *
 * STM32 实现参考:
 *   使用 TIMx CH1~CH4 产生 PWM，GPIO 控制方向 (IN1/IN2)。
 *   例如 L298N / TB6612:
 *     - 正值: IN1=H, IN2=L, PWM duty = |speed|%
 *     - 负值: IN1=L, IN2=H, PWM duty = |speed|%
 *     - 0:    IN1=L, IN2=L (刹车) 或 PWM=0
 */
void platform_motor_set(const int8_t speeds[4]);

/**
 * @brief 立即停止所有电机 (紧急制动)
 */
void platform_motor_stop(void);

/*===========================================================================
 * 通信接口
 *===========================================================================*/

/**
 * @brief 通过 BLE 或 UART 发送一帧数据
 * @param data  帧数据
 * @param len   帧长度
 *
 * STM32 实现参考:
 *   - 如果 BLE 模块连在 USART1: HAL_UART_Transmit(&huart1, data, len, 100)
 *   - 如果使用 BLE Notify: 通过 BLE 协议栈的 notify API 发送
 *
 * 注意:
 *   协议帧最大 10 字节，BLE 4.2+ 默认 MTU 23 可直接发送，
 *   无需分包。
 */
void platform_send_frame(const uint8_t *data, uint8_t len);

/*===========================================================================
 * 定时器接口
 *===========================================================================*/

/**
 * @brief 获取系统运行毫秒数 (用于心跳超时和定时指令)
 * @return 系统运行时间 (ms)
 *
 * STM32 + FreeRTOS 实现:
 *   return xTaskGetTickCount() * portTICK_PERIOD_MS;
 *
 * STM32 bare-metal 实现:
 *   return HAL_GetTick();
 */
uint32_t platform_get_tick_ms(void);

/*===========================================================================
 * 传感器接口 (可选，按需实现)
 *===========================================================================*/

/**
 * @brief 读取电池电压并转换为百分比
 * @return 电量百分比 0~100
 *
 * 实现: ADC 采样电池电压 → 分压计算 → 映射 0~100%
 */
uint8_t platform_read_battery(void);

/**
 * @brief 读取 Z 轴角速度 (用于 CMD_TURN 角度控制)
 * @return 角速度 (°/s)
 *
 * 可选。如果没有陀螺仪，CMD_TURN 可使用定时估算转向时间。
 */
float platform_read_gyro_z(void);

/**
 * @brief 读取轮速编码器累计脉冲数 (用于直线 PID 保持)
 * @param motor_idx 电机索引 0~3 (M1~M4)
 * @return 编码器累计脉冲
 *
 * 可选。用于实现直线运动 PID 补偿。
 */
int32_t platform_read_encoder(uint8_t motor_idx);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_H */
