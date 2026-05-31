/**
 * @file    proto_cmd.c
 * @brief   协议命令处理层实现
 *
 * 流程:
 *   1. 收到帧 → proto_cmd_dispatch() 按 CMD 码分发内部 handler
 *   2. CMD_STOP    → 最高优先级, 中断一切
 *   3. CMD_MOVE    → 启动电机 + 可选定时器
 *   4. CMD_TURN    → 原地转向 (由外部角度检测触发停止)
 *   5. CMD_ARC     → 弧线运动 (持续指令, 直到下一帧)
 *   6. CMD_HEARTBEAT → 回复 STA_HB_ACK, 重置超时计数
 *   7. 定时器 tick → proto_cmd_timer_check() → 到时 → 停止 + CMD_DONE
 *   8. 心跳超时    → proto_cmd_heartbeat_check() → 失控保护
 */

#include "proto_cmd.h"
#include "diff_drive.h"
#include <string.h>

/*===========================================================================
 * 差速参数 (由外部通过 proto_cmd_set_drive_params 设置)
 *===========================================================================*/
static const diff_drive_params_t *g_drive_params;

/*===========================================================================
 * 发送辅助宏 (减少重复代码)
 *===========================================================================*/

#define SEND_STATUS(ctx_, s_) do { \
    uint8_t _b[PROTO_MAX_FRAME_SIZE]; uint8_t _l; \
    protocol_build_status(&(s_), _b, &_l); \
    if ((ctx_)->send_frame) (ctx_)->send_frame(_b, _l); \
} while(0)

#define SEND_HB_ACK(ctx_, a_) do { \
    uint8_t _b[PROTO_MAX_FRAME_SIZE]; uint8_t _l; \
    protocol_build_hb_ack(&(a_), _b, &_l); \
    if ((ctx_)->send_frame) (ctx_)->send_frame(_b, _l); \
} while(0)

#define SEND_CMD_DONE(ctx_, d_) do { \
    uint8_t _b[PROTO_MAX_FRAME_SIZE]; uint8_t _l; \
    protocol_build_cmd_done(&(d_), _b, &_l); \
    if ((ctx_)->send_frame) (ctx_)->send_frame(_b, _l); \
} while(0)

/*===========================================================================
 * 平台相关弱函数 (集成时由 STM32 HAL 实现覆盖)
 *===========================================================================*/

#ifndef __weak
#define __weak  __attribute__((weak))
#endif

/**
 * @brief 设置四轮电机速度。
 *        speeds[0]=M1(左前), speeds[1]=M2(左后),
 *        speeds[2]=M3(右前), speeds[3]=M4(右后)
 *        正值 = 正转, 负值 = 反转, 0 = 停止, 范围 0~±100 (百分比)
 */
__weak void platform_motor_set(const int8_t speeds[4])
{
    (void)speeds;
    /* TODO: 集成 STM32 工程后，用 TIM PWM + GPIO 方向控制实现 */
}

/**
 * @brief 立即停止所有电机 (紧急制动)
 */
__weak void platform_motor_stop(void)
{
    int8_t zero[4] = {0, 0, 0, 0};
    platform_motor_set(zero);
}

/*===========================================================================
 * 内部 handler 前向声明
 *===========================================================================*/

static void handle_stop(proto_sys_ctx_t *ctx);
static void handle_move(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd);
static void handle_turn(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd);
static void handle_arc(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd);
static void handle_heartbeat(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd);

/*===========================================================================
 * 公共 API
 *===========================================================================*/

void proto_cmd_init(proto_sys_ctx_t *ctx,
                    void (*send_func)(const uint8_t *data, uint8_t len))
{
    if (ctx == NULL) return;
    (void)memset(ctx, 0, sizeof(*ctx));
    ctx->send_frame = send_func;
    ctx->battery    = 100;  /* 默认满电 */
}

void proto_cmd_set_drive_params(const diff_drive_params_t *params)
{
    g_drive_params = params;
}

void proto_cmd_set_tick(proto_sys_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) return;
    ctx->hb_last_tick    = now_ms;      /* 记录心跳时间 */
    ctx->timer_start_tick = now_ms;     /* 记录定时起点 */
}

void proto_cmd_dispatch(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd)
{
    if (ctx == NULL || cmd == NULL) return;

    switch (cmd->cmd) {
    case CMD_STOP:      handle_stop(ctx);           break;
    case CMD_MOVE:      handle_move(ctx, cmd);      break;
    case CMD_TURN:      handle_turn(ctx, cmd);      break;
    case CMD_ARC:       handle_arc(ctx, cmd);       break;
    case CMD_HEARTBEAT: handle_heartbeat(ctx, cmd); break;
    default:            /* 未知命令, 忽略 */         break;
    }
}

/*===========================================================================
 * 命令内部处理器
 *===========================================================================*/

/*---------------------------------------------------------------------
 * CMD_STOP — 立即停止 (最高优先级)
 *-------------------------------------------------------------------*/
static void handle_stop(proto_sys_ctx_t *ctx)
{
    /* 如果之前有定时任务，发送中断通知 */
    if (ctx->timer_active) {
        sta_cmd_done_t done = { ctx->current_cmd, CMD_RESULT_STOPPED };
        SEND_CMD_DONE(ctx, done);
    }

    platform_motor_stop();

    ctx->current_cmd  = 0;
    ctx->timer_active = 0;
    ctx->state       &= (uint8_t)(~(STATE_MOVING | STATE_TIMED));

    /* 上报状态变更 */
    sta_status_t s;
    proto_cmd_get_status(ctx, &s);
    SEND_STATUS(ctx, s);
}

/*---------------------------------------------------------------------
 * CMD_MOVE — 直线运动
 *-------------------------------------------------------------------*/
static void handle_move(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd)
{
    cmd_move_t mv;
    if (!protocol_parse_move(cmd, &mv)) return;

    if (mv.speed == 0) {
        handle_stop(ctx);
        return;
    }

    int8_t spd = (int8_t)mv.speed;
    if (mv.direction == DIR_BACKWARD) {
        spd = (int8_t)(-spd);
    }
    diff_drive_speeds_t ds;
    int8_t speeds[4];
    diff_drive_straight(g_drive_params, spd, &ds);
    diff_drive_to_array(&ds, speeds);
    platform_motor_set(speeds);

    ctx->current_cmd = CMD_MOVE;
    ctx->state       |= STATE_MOVING;

    if (mv.duration_ms > 0) {
        ctx->state             |= STATE_TIMED;
        ctx->timer_active       = 1;
        ctx->timer_duration_ms  = mv.duration_ms;
        /* timer_start_tick 已由 proto_cmd_set_tick() 设置 */
    } else {
        ctx->timer_active = 0;
    }

    sta_status_t s;
    proto_cmd_get_status(ctx, &s);
    SEND_STATUS(ctx, s);
}

/*---------------------------------------------------------------------
 * CMD_TURN — 原地转向
 *
 * 左转: M1+M2 反转, M3+M4 正转
 * 右转: M1+M2 正转, M3+M4 反转
 *
 * 注意: CMD_TURN 的完成由外部角度传感器 (陀螺仪/编码器) 检测后
 *       调用 proto_cmd_complete_current(ctx, CMD_RESULT_SUCCESS) 停止。
 *-------------------------------------------------------------------*/
static void handle_turn(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd)
{
    cmd_turn_t tn;
    if (!protocol_parse_turn(cmd, &tn)) return;

    int8_t spd = (int8_t)tn.speed;
    int left = (tn.direction == TURN_LEFT) ? 1 : 0;

    diff_drive_speeds_t ds;
    int8_t speeds[4];
    diff_drive_spin(g_drive_params, spd, left, &ds);
    diff_drive_to_array(&ds, speeds);
    platform_motor_set(speeds);

    ctx->current_cmd = CMD_TURN;
    ctx->state       |= (STATE_MOVING | STATE_TIMED);
    ctx->timer_active = 1;
    ctx->timer_duration_ms = 0;  /* 不由时间停止，由角度传感器决定 */

    sta_status_t s;
    proto_cmd_get_status(ctx, &s);
    SEND_STATUS(ctx, s);
}

/*---------------------------------------------------------------------
 * CMD_ARC — 弧线运动
 *
 * 差速公式:
 *   Vi = V * (R - W/2) / R     内侧轮速
 *   Vo = V * (R + W/2) / R     外侧轮速
 *
 * 其中 V=线速度, R=转弯半径(cm), W=左右轮距(cm)
 *-------------------------------------------------------------------*/
static void handle_arc(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd)
{
    cmd_arc_t arc;
    if (!protocol_parse_arc(cmd, &arc)) return;

    int8_t spd = (int8_t)arc.speed;
    if (arc.direction == DIR_BACKWARD) {
        spd = (int8_t)(-spd);
    }
    int left = (arc.turn_dir == TURN_LEFT) ? 1 : 0;

    diff_drive_speeds_t ds;
    int8_t speeds[4];
    diff_drive_arc(g_drive_params, spd, arc.radius_cm, left, &ds);
    diff_drive_to_array(&ds, speeds);
    platform_motor_set(speeds);

    ctx->current_cmd  = CMD_ARC;
    ctx->state        |= STATE_MOVING;
    ctx->timer_active  = 0;  /* 弧线运动通常是持续指令 */

    sta_status_t s;
    proto_cmd_get_status(ctx, &s);
    SEND_STATUS(ctx, s);
}

/*---------------------------------------------------------------------
 * CMD_HEARTBEAT — 心跳
 *-------------------------------------------------------------------*/
static void handle_heartbeat(proto_sys_ctx_t *ctx, const proto_cmd_t *cmd)
{
    cmd_heartbeat_t hb;
    if (!protocol_parse_heartbeat(cmd, &hb)) return;

    ctx->hb_last_seq     = hb.seq;
    ctx->hb_timeout_flag = 0;

    /* 如果之前心跳超时故障，现在恢复 */
    if (ctx->error == ERR_COMM_TIMEOUT) {
        ctx->error  = ERR_NONE;
        ctx->state &= (uint8_t)(~STATE_FAULT);
    }

    sta_hb_ack_t ack = { hb.seq, ctx->battery, ctx->state };
    SEND_HB_ACK(ctx, ack);
}

/*===========================================================================
 * 定时器 & 心跳超时检查
 *===========================================================================*/

void proto_cmd_timer_check(proto_sys_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) return;
    if (!ctx->timer_active) return;

    /* 仅 CMD_MOVE 使用时间到时停止 */
    if (ctx->current_cmd == CMD_MOVE && ctx->timer_duration_ms > 0) {
        uint32_t elapsed = now_ms - ctx->timer_start_tick;
        if (elapsed >= ctx->timer_duration_ms) {
            proto_cmd_complete_current(ctx, CMD_RESULT_SUCCESS);
        }
    }
    /* CMD_TURN: 由外部角度检测触发 proto_cmd_complete_current() */
    /* CMD_ARC:  持续指令，无自动停止 */
}

void proto_cmd_heartbeat_check(proto_sys_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL || ctx->hb_timeout_flag) return;

    uint32_t elapsed = now_ms - ctx->hb_last_tick;
    if (elapsed >= HB_TIMEOUT_MS) {
        /* 失控保护 */
        ctx->hb_timeout_flag = 1;
        ctx->error           = ERR_COMM_TIMEOUT;
        ctx->state           |= STATE_FAULT;

        platform_motor_stop();
        ctx->current_cmd  = 0;
        ctx->timer_active = 0;
        ctx->state        &= (uint8_t)(~STATE_MOVING);

        sta_status_t s;
        proto_cmd_get_status(ctx, &s);
        SEND_STATUS(ctx, s);
    }
}

void proto_cmd_complete_current(proto_sys_ctx_t *ctx, uint8_t result)
{
    if (ctx == NULL) return;

    platform_motor_stop();

    sta_cmd_done_t done = { ctx->current_cmd, result };
    SEND_CMD_DONE(ctx, done);

    ctx->current_cmd  = 0;
    ctx->timer_active = 0;
    ctx->state        &= (uint8_t)(~(STATE_MOVING | STATE_TIMED));
}

/*===========================================================================
 * 状态管理接口
 *===========================================================================*/

void proto_cmd_set_battery(proto_sys_ctx_t *ctx, uint8_t percent)
{
    if (ctx == NULL) return;
    if (percent > 100) percent = 100;
    ctx->battery = percent;

    /* 电池低压报警 (阈值可调) */
    #define BAT_LOW_THRESHOLD  10
    if (percent < BAT_LOW_THRESHOLD) {
        ctx->error  = ERR_LOW_BATTERY;
        ctx->state |= STATE_FAULT;
    } else if (ctx->error == ERR_LOW_BATTERY) {
        ctx->error  = ERR_NONE;
        ctx->state &= (uint8_t)(~STATE_FAULT);
    }
}

void proto_cmd_set_error(proto_sys_ctx_t *ctx, uint8_t err_code)
{
    if (ctx == NULL) return;
    ctx->error  = err_code;
    ctx->state |= STATE_FAULT;
}

void proto_cmd_clear_error(proto_sys_ctx_t *ctx)
{
    if (ctx == NULL) return;
    ctx->error  = ERR_NONE;
    ctx->state &= (uint8_t)(~STATE_FAULT);
}

void proto_cmd_get_status(const proto_sys_ctx_t *ctx, sta_status_t *status)
{
    if (ctx == NULL || status == NULL) return;
    status->battery  = ctx->battery;
    status->state    = ctx->state;
    status->error    = ctx->error;
    status->cmd_echo = ctx->current_cmd;
}
