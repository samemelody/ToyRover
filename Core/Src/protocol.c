/**
 * @file    protocol.c
 * @brief   ToyRover 协议实现 - CRC8、帧解析状态机、帧构建器、Payload 解析
 */
#include "protocol.h"
#include <string.h>
#include <stdio.h>

/*===========================================================================
 * CRC8 (CRC-8/ROHC)
 * 多项式: 0x07 (x^8 + x^2 + x^1 + 1)
 * 初始值: 0x00
 * 参考: protocol.md v1.0 附录 CRC8 参考实现
 *===========================================================================*/

uint8_t protocol_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/*===========================================================================
 * 帧解析状态机
 *
 * 状态转换:
 *   PARSE_SYNC ──(0xA5)──► PARSE_LEN ──(len<=6)──► PARSE_CMD
 *       ▲                                              │
 *       │                           (len>0)            │
 *       │                         ┌────────────────────┘
 *       │                         ▼
 *       │                    PARSE_PAYLOAD ──(收完)──► PARSE_CRC
 *       │                         ▲                      │
 *       │                         │ (len=0, 跳过)        │
 *       │                         └──────────────────────┘
 *       │                                    │
 *       └───────── (CRC 对 / 不对) ──────────┘
 *
 * 任何时候收到非法值都回退到 PARSE_SYNC，保证自同步。
 *===========================================================================*/

/** 解析状态 */
typedef enum {
    PARSE_SYNC,     /**< 等待 SYNC 字节 0xA5 */
    PARSE_LEN,      /**< 读取 LEN 字节 */
    PARSE_CMD,      /**< 读取 CMD 字节 */
    PARSE_PAYLOAD,  /**< 读取 PAYLOAD */
    PARSE_CRC,      /**< 读取 CRC8 并校验 */
} parse_state_t;

/** 解析器上下文 (模块级静态变量，非线程安全) */
static parse_state_t  g_parse_state = PARSE_SYNC;
static uint8_t        g_frame_buf[PROTO_MAX_FRAME_SIZE];   /**< 帧接收缓冲区 */
static uint8_t        g_frame_pos;                         /**< 当前写入位置 */
static uint8_t        g_expect_len;                        /**< 期望的 PAYLOAD 长度 */
static uint8_t        g_payload_idx;                       /**< 已收到的 PAYLOAD 字节数 */

/*---------------------------------------------------------------------------
 * 内部: 帧校验
 *---------------------------------------------------------------------------*/

/**
 * @brief 校验已接收帧的完整性和 CRC，并填充 proto_cmd_t
 * @param raw       原始帧缓冲区
 * @param total_len 帧总字节数 (含 CRC)
 * @param out_cmd   输出解析后的命令
 * @return true=校验通过, false=非法帧 (CRC 错、长度越界等)
 */
static bool frame_validate(const uint8_t *raw, uint8_t total_len, proto_cmd_t *out_cmd)
{
    /* 最短帧: SYNC(1) + LEN(1) + CMD(1) + CRC(1) = 4 字节 */
    if (total_len < PROTO_HEADER_SIZE + PROTO_CRC_SIZE) {
        return false;
    }

    uint8_t len       = raw[1];
    uint8_t data_len  = (uint8_t)(PROTO_HEADER_SIZE + len);  /* CRC 覆盖的字节数 */

    /* LEN 范围检查 */
    if (len > PROTO_MAX_PAYLOAD) {
        return false;
    }

    /* 长度一致性检查 */
    if (total_len != (uint8_t)(data_len + PROTO_CRC_SIZE)) {
        return false;
    }

    /* CRC8 校验: 从 SYNC 到 PAYLOAD 末字节 */
    uint8_t calc_crc = protocol_crc8(raw, data_len);
    if (calc_crc != raw[data_len]) {
        return false;
    }

    /* 输出 */
    out_cmd->cmd = raw[2];
    out_cmd->payload_len = len;
    if (len > 0) {
        (void)memcpy(out_cmd->payload, &raw[PROTO_HEADER_SIZE], len);
    }

    return true;
}

/*---------------------------------------------------------------------------
 * 公开 API
 *---------------------------------------------------------------------------*/

void protocol_reset(void)
{
    g_parse_state  = PARSE_SYNC;
    g_frame_pos    = 0;
    g_expect_len   = 0;
    g_payload_idx  = 0;
}

void protocol_feed_byte(uint8_t byte, proto_cmd_handler_t handler)
{
    switch (g_parse_state) {

    /*---------------------------------------------------------------------
     * STATE: 等待 SYNC 字节
     *-------------------------------------------------------------------*/
    case PARSE_SYNC:
        if (byte == PROTO_SYNC) {
            g_frame_buf[0] = byte;
            g_frame_pos    = 1;
            g_parse_state  = PARSE_LEN;
        }
        /* 非 SYNC 字节被静默丢弃，实现自同步 */
        break;

    /*---------------------------------------------------------------------
     * STATE: 读取 LEN
     *-------------------------------------------------------------------*/
    case PARSE_LEN:
        if (byte > PROTO_MAX_PAYLOAD) {
            /* 非法长度，回退同步搜索 */
            g_parse_state = PARSE_SYNC;
            break;
        }
        g_frame_buf[1] = byte;
        g_frame_pos    = 2;
        g_expect_len   = byte;
        g_parse_state  = PARSE_CMD;
        break;

    /*---------------------------------------------------------------------
     * STATE: 读取 CMD
     *-------------------------------------------------------------------*/
    case PARSE_CMD:
        g_frame_buf[2] = byte;
        g_frame_pos    = 3;
        if (g_expect_len > 0) {
            g_payload_idx = 0;
            g_parse_state = PARSE_PAYLOAD;
        } else {
            /* 无 PAYLOAD，直接进入 CRC */
            g_parse_state = PARSE_CRC;
        }
        break;

    /*---------------------------------------------------------------------
     * STATE: 读取 PAYLOAD
     *-------------------------------------------------------------------*/
    case PARSE_PAYLOAD:
        g_frame_buf[PROTO_HEADER_SIZE + g_payload_idx] = byte;
        g_payload_idx++;
        if (g_payload_idx >= g_expect_len) {
            g_parse_state = PARSE_CRC;
        }
        break;

    /*---------------------------------------------------------------------
     * STATE: 读取并校验 CRC
     *-------------------------------------------------------------------*/
    case PARSE_CRC: {
        uint8_t total_len = (uint8_t)(PROTO_HEADER_SIZE + g_expect_len + PROTO_CRC_SIZE);
        g_frame_buf[total_len - 1] = byte;

        proto_cmd_t cmd;
        if (frame_validate(g_frame_buf, total_len, &cmd)) {
            /* 有效帧，回调处理 */
            g_parse_state = PARSE_SYNC;   /* 准备接收下一帧 */
            if (handler) {
                handler(&cmd);
            }
        } else {
            /* CRC 错或其它校验失败，丢弃整帧，重新同步 */
            printf("[CRC FAIL] raw:");
            for (uint8_t i = 0; i < total_len; i++) {
                printf(" %02X", g_frame_buf[i]);
            }
            printf(" (got CRC=%02X)\r\n", byte);
            g_parse_state = PARSE_SYNC;
        }
        break;
    }

    default:
        /* 防御: 非法状态回退 */
        protocol_reset();
        break;
    }
}

/*===========================================================================
 * 帧构建器
 *
 * 格式: buf[0]=SYNC, buf[1]=LEN, buf[2]=CMD, buf[3..]=PAYLOAD, buf[N]=CRC
 *===========================================================================*/

/**
 * @brief 内部: 构建完整帧
 * @param cmd     命令/状态码
 * @param payload Payload 数据指针 (可为 NULL 若 len==0)
 * @param len     Payload 长度
 * @param buf     输出缓冲区
 * @param out_len 输出帧总长度
 */
static void frame_build(uint8_t cmd, const uint8_t *payload, uint8_t len,
                        uint8_t *buf, uint8_t *out_len)
{
    buf[0] = PROTO_SYNC;
    buf[1] = len;
    buf[2] = cmd;

    uint8_t data_len = (uint8_t)(PROTO_HEADER_SIZE + len);

    if (len > 0 && payload != NULL) {
        (void)memcpy(&buf[PROTO_HEADER_SIZE], payload, len);
    }

    /* CRC8 覆盖 SYNC ~ PAYLOAD 末字节 */
    buf[data_len] = protocol_crc8(buf, data_len);
    *out_len = (uint8_t)(data_len + PROTO_CRC_SIZE);
}

void protocol_build_status(const sta_status_t *status, uint8_t *buf, uint8_t *out_len)
{
    uint8_t payload[4];
    payload[0] = status->battery;
    payload[1] = status->state;
    payload[2] = status->error;
    payload[3] = status->cmd_echo;
    frame_build(STA_STATUS, payload, PAYLOAD_LEN_STATUS, buf, out_len);
}

void protocol_build_hb_ack(const sta_hb_ack_t *ack, uint8_t *buf, uint8_t *out_len)
{
    uint8_t payload[3];
    payload[0] = ack->seq;
    payload[1] = ack->battery;
    payload[2] = ack->state;
    frame_build(STA_HB_ACK, payload, PAYLOAD_LEN_HB_ACK, buf, out_len);
}

void protocol_build_cmd_done(const sta_cmd_done_t *done, uint8_t *buf, uint8_t *out_len)
{
    uint8_t payload[2];
    payload[0] = done->cmd_echo;
    payload[1] = done->result;
    frame_build(STA_CMD_DONE, payload, PAYLOAD_LEN_CMD_DONE, buf, out_len);
}

/*===========================================================================
 * Payload 解析器 (小端序)
 *===========================================================================*/

bool protocol_parse_move(const proto_cmd_t *cmd, cmd_move_t *out)
{
    if (cmd == NULL || out == NULL || cmd->payload_len < PAYLOAD_LEN_MOVE) {
        return false;
    }
    out->direction   = cmd->payload[0];
    out->speed       = cmd->payload[1];
    out->duration_ms = (uint16_t)(cmd->payload[2] | ((uint16_t)cmd->payload[3] << 8));
    return true;
}

bool protocol_parse_turn(const proto_cmd_t *cmd, cmd_turn_t *out)
{
    if (cmd == NULL || out == NULL || cmd->payload_len < PAYLOAD_LEN_TURN) {
        return false;
    }
    out->direction = cmd->payload[0];
    out->angle     = cmd->payload[1];
    out->speed     = cmd->payload[2];
    return true;
}

bool protocol_parse_arc(const proto_cmd_t *cmd, cmd_arc_t *out)
{
    if (cmd == NULL || out == NULL || cmd->payload_len < PAYLOAD_LEN_ARC) {
        return false;
    }
    out->direction = cmd->payload[0];
    out->turn_dir  = cmd->payload[1];
    out->speed     = cmd->payload[2];
    out->radius_cm = (uint16_t)(cmd->payload[3] | ((uint16_t)cmd->payload[4] << 8));
    return true;
}

bool protocol_parse_heartbeat(const proto_cmd_t *cmd, cmd_heartbeat_t *out)
{
    if (cmd == NULL || out == NULL || cmd->payload_len < PAYLOAD_LEN_HEARTBEAT) {
        return false;
    }
    out->seq = cmd->payload[0];
    return true;
}
