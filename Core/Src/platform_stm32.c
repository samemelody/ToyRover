/**
 * @file    platform_stm32.c
 * @brief   STM32F103 platform implementation for platform.h
 *
 * Overrides the __weak stubs in proto_cmd.c with real hardware implementations:
 *   - Motor control via TIM3 CH1-4 (placeholder — requires CubeMX TIM config)
 *   - Data transport via USB CDC (usb_cdc_send)
 *   - System tick via HAL_GetTick
 */
#include "platform.h"
#include "main.h"

/*===========================================================================
 * Motor Control (TIM3 PWM)
 *
 * Placeholder: TIM3 CH1-4 need to be configured in CubeMX for PWM output.
 * Each motor needs a PWM channel + a direction GPIO (IN1/IN2).
 *
 * Expected pin mapping (adjust to match your hardware):
 *   M1 (左前): TIM3_CH1 (PA6)  + dir GPIO
 *   M2 (左后): TIM3_CH2 (PA7)  + dir GPIO
 *   M3 (右前): TIM3_CH3 (PB0)  + dir GPIO
 *   M4 (右后): TIM3_CH4 (PB1)  + dir GPIO
 *===========================================================================*/

void platform_motor_set(const int8_t speeds[4])
{
    /* TODO: Implement when motor driver hardware is wired.
     * Convert speeds[i] (+100..-100) to PWM duty cycle and direction GPIO.
     *
     * Example for L298N / TB6612 (per motor):
     *   if (speed > 0): IN1=HIGH, IN2=LOW, PWM duty = speed%
     *   if (speed < 0): IN1=LOW,  IN2=HIGH, PWM duty = -speed%
     *   if (speed == 0): IN1=LOW, IN2=LOW   (brake/coast)
     */
    (void)speeds;
}

void platform_motor_stop(void)
{
    int8_t zero[4] = {0, 0, 0, 0};
    platform_motor_set(zero);
}

/*===========================================================================
 * Communication — UART transport (BLE CC2540 module on USART1)
 *===========================================================================*/

void platform_send_frame(const uint8_t *data, uint8_t len)
{
    extern UART_HandleTypeDef huart1;
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100);
}

/*===========================================================================
 * System tick
 *===========================================================================*/

uint32_t platform_get_tick_ms(void)
{
    return HAL_GetTick();
}

/*===========================================================================
 * Sensors — optional, stubbed
 *===========================================================================*/

uint8_t platform_read_battery(void)
{
    /* TODO: ADC1 channel read, map to 0-100% */
    return 100;
}

float platform_read_gyro_z(void)
{
    /* TODO: MPU6050 / ICM-42688 I2C read */
    return 0.0f;
}

int32_t platform_read_encoder(uint8_t motor_idx)
{
    /* TODO: Encoder TIM counter read */
    (void)motor_idx;
    return 0;
}
