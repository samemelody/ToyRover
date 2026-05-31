/**
 * @file    proto_cmd.h
 * @brief   协议命令处理层 - 命令分发、心跳管理、系统状态机、定时指令调度
 *
 * 本模块在 protocol.h/protocol.c 提供的帧解析基础上实现业务逻辑层:
 *   - 命令分发 (MOVE / TURN / ARC / STOP / HEARTBEAT)
 *   - 心跳超时检测与失控保护
 *   - 定时/定角度指令的倒计时管理
 *   - 系统运行状态维护 (STATE bitmask, error code)
 *
 * 依赖:
 *   - protocol.h  帧解析与构建
 *   - platform.h  硬件抽象 (电机控制、定时器、UART发送)
 *   - diff_drive.h 差速计算 (可选，弧线运动需用到)
 */
#ifndef PROTO_CMD_H
#define PROTO_CMD_H

#include "protocol.h"
#include "diff_drive.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 系统运行上下文
 *===========================================================================*/

/** MCU 系统运行上下文 (贯穿整个命令处理流程) */
typedef struct {
    /* ---- 系统状态 ---- */
    uint8_t  state;                     /**< 状态位掩码 (STATE_*) */
    uint8_t  error;                     /**< 当前故障码 (ERR_*) */
    uint8_t  current_cmd;               /**< 当前执行的命令码 (0 = 空闲) */
    uint8_t  battery;                   /**< 电量百分比 0~100 */

    /* ---- 心跳 ---- */
    uint8_t  hb_last_seq;               /**< 上一次收到的心跳序号 */
    uint32_t hb_last_tick;              /**< 上一次收到心跳时的系统 tick (ms) */
    uint8_t  hb_timeout_flag;           /**< 心跳超时标记 (1 = 已触发失控保护) */

    /* ---- 定时/定角度指令倒计时 ---- */
    uint8_t  timer_active;              /**< 定时器激活标记 */
    uint32_t timer_start_tick;          /**< 定时任务起始 tick */
    uint32_t timer_duration_ms;         /**< 定时任务总时长 (ms) */

    /* ---- 回调: 向手机发送帧 ---- */
    /**
     * @brief 发送一帧数据到手机 (由上层注入 BLE Notify / UART TX 实现)
     * @param data  帧数据
     * @param len   帧长度
     */
    void (*send_frame)(const uint8_t *data, uint8_t len);

} proto_sys_ctx_t;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * @brief 初始化系统上下文
 * @param ctx        系统上下文指针
 * @param send_func  帧发送回调 (BLE Notify / UART TX)
 */
void proto_cmd_init(proto_sys_ctx_t *ctx, void (*send_func)(const uint8_t *data, uint8_t len));

/**
 * @brief 协议命令分发入口。
 *        将此函数注册为 protocol_feed_byte 的回调 handler。
 *        收到完整有效帧后自动调用。
 * @param ctx  系统上下文
 * @param cmd  解析后的命令帧
 */
void proto_cmd_dispatch(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd);

/**
 * @brief 更新当前系统 tick，并设置 CMD_MOVE 的定时起点。
 *        应在 proto_cmd_dispatch() 之前由上层调用，
 *        或由 FreeRTOS 任务在每次命令到达时设置。
 *
 *        使用方式 (FreeRTOS):
 *          uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
 *          proto_cmd_set_tick(ctx, now);
 *          proto_cmd_dispatch(ctx, &cmd);
 *
 * @param ctx     系统上下文
 * @param now_ms  当前系统时间 (ms)
 */
void proto_cmd_set_tick(proto_sys_ctx_t *ctx, uint32_t now_ms);

/**
 * @brief 心跳超时检测，需每 ~100ms 调用一次 (在 FreeRTOS 定时器或任务中调用)。
 *        检测是否连续丢失 HB_TIMEOUT_COUNT 次心跳 (1500ms)。
 * @param ctx     系统上下文
 * @param now_ms  当前系统时间 (ms)
 */
void proto_cmd_heartbeat_check(proto_sys_ctx_t *ctx, uint32_t now_ms);

/**
 * @brief 定时指令超时检测，需每 ~50~100ms 调用一次。
 *        仅处理 CMD_MOVE 的到时 (CMD_TURN 由外部角度检测触发停止)。
 * @param ctx     系统上下文
 * @param now_ms  当前系统时间 (ms)
 */
void proto_cmd_timer_check(proto_sys_ctx_t *ctx, uint32_t now_ms);

/**
 * @brief 停止当前运动并发送 STA_CMD_DONE (用于角度传感器触发停止等场景)
 * @param ctx      系统上下文
 * @param result   完成结果 (CMD_RESULT_SUCCESS / CMD_RESULT_ABORT)
 */
void proto_cmd_complete_current(proto_sys_ctx_t *ctx, uint8_t result);

/**
 * @brief 更新电池电量 (由 ADC 采样后调用)
 * @param ctx      系统上下文
 * @param percent  电量百分比 0~100
 */
void proto_cmd_set_battery(proto_sys_ctx_t *ctx, uint8_t percent);

/**
 * @brief 设置故障状态
 * @param ctx       系统上下文
 * @param err_code  故障码 (ERR_*)
 */
void proto_cmd_set_error(proto_sys_ctx_t *ctx, uint8_t err_code);

/**
 * @brief 清除故障状态
 */
void proto_cmd_clear_error(proto_sys_ctx_t *ctx);

/**
 * @brief 获取当前系统状态快照
 */
void proto_cmd_get_status(const proto_sys_ctx_t *ctx, sta_status_t *status);

/**
 * @brief 设置差速驱动参数 (在 proto_cmd_init 之后调用)
 * @param params  差速参数 (轮距等)，可为 NULL (将使用默认值 15cm)
 */
void proto_cmd_set_drive_params(const diff_drive_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* PROTO_CMD_H */
