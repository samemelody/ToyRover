/**
 * @file    protocol.h
 * @brief   ToyRover BLE 通信协议 - 帧格式、命令码、数据结构、API 定义
 *
 * 帧格式: SYNC(1B) | LEN(1B) | CMD(1B) | PAYLOAD(0~6B) | CRC8(1B)
 *
 * 参考文档: protocol.md v1.0
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================
 * 帧常量
 *===========================================================================*/
#define PROTO_SYNC              0xA5    /**< 帧同步头 */
#define PROTO_MAX_PAYLOAD       6       /**< 最大 Payload 长度 (字节) */
#define PROTO_HEADER_SIZE       3       /**< SYNC + LEN + CMD */
#define PROTO_CRC_SIZE          1       /**< CRC8 长度 */
#define PROTO_MAX_FRAME_SIZE    10      /**< 最大完整帧: 3+6+1=10 字节 */

/*===========================================================================
 * 下行命令码 (Phone → MCU)
 *===========================================================================*/
#define CMD_MOVE                0x10    /**< 直线运动 */
#define CMD_TURN                0x20    /**< 原地转向 */
#define CMD_ARC                 0x30    /**< 弧线运动 */
#define CMD_STOP                0x40    /**< 立即停止 (最高优先级) */
#define CMD_HEARTBEAT           0x50    /**< 心跳 */

/*===========================================================================
 * 上行状态码 (MCU → Phone)
 *===========================================================================*/
#define STA_STATUS              0x81    /**< 状态上报 */
#define STA_HB_ACK              0x82    /**< 心跳应答 */
#define STA_CMD_DONE            0x83    /**< 指令执行完毕 */

/*===========================================================================
 * 方向常量
 *===========================================================================*/
#define DIR_FORWARD             0x01    /**< 前进 */
#define DIR_BACKWARD            0x02    /**< 后退 */
#define TURN_LEFT               0x01    /**< 左转 */
#define TURN_RIGHT              0x02    /**< 右转 */

/*===========================================================================
 * Payload 长度常量 (按命令码)
 *===========================================================================*/
#define PAYLOAD_LEN_MOVE        4       /**< CMD_MOVE payload 长度 */
#define PAYLOAD_LEN_TURN        3       /**< CMD_TURN payload 长度 */
#define PAYLOAD_LEN_ARC         5       /**< CMD_ARC  payload 长度 */
#define PAYLOAD_LEN_STOP        0       /**< CMD_STOP payload 长度 */
#define PAYLOAD_LEN_HEARTBEAT   1       /**< CMD_HEARTBEAT payload 长度 */
#define PAYLOAD_LEN_STATUS      4       /**< STA_STATUS payload 长度 */
#define PAYLOAD_LEN_HB_ACK      3       /**< STA_HB_ACK payload 长度 */
#define PAYLOAD_LEN_CMD_DONE    2       /**< STA_CMD_DONE payload 长度 */

/*===========================================================================
 * 状态位掩码 (state 字段)
 *===========================================================================*/
#define STATE_MOVING            0x01    /**< bit0: 运动中 */
#define STATE_FAULT             0x02    /**< bit1: 故障 */
#define STATE_TIMED             0x04    /**< bit2: 正在执行定时/定角度指令 */

/*===========================================================================
 * 故障码 (error 字段)
 *===========================================================================*/
#define ERR_NONE                0x00    /**< 正常 */
#define ERR_STALL               0x01    /**< 电机堵转 */
#define ERR_LOW_BATTERY         0x02    /**< 电池低压 */
#define ERR_COMM_TIMEOUT        0x03    /**< 通信超时 (心跳丢失) */

/*===========================================================================
 * 指令完成结果码 (CMD_DONE result 字段)
 *===========================================================================*/
#define CMD_RESULT_SUCCESS      0x00    /**< 成功完成 */
#define CMD_RESULT_STOPPED      0x01    /**< 被 STOP 指令中断 */
#define CMD_RESULT_ABORT        0xFF    /**< 异常中止 */

/*===========================================================================
 * 心跳参数
 *===========================================================================*/
#define HB_INTERVAL_MS          500     /**< 期望心跳间隔 (ms) */
#define HB_TIMEOUT_COUNT        3       /**< 连续丢失次数阈值 */
#define HB_TIMEOUT_MS           (HB_INTERVAL_MS * HB_TIMEOUT_COUNT)  /**< 1500ms */

/*===========================================================================
 * 解析后的命令数据结构
 *===========================================================================*/

/** 通用命令帧 (解析后) */
typedef struct {
    uint8_t cmd;                                        /**< 命令码 / 状态码 */
    uint8_t payload_len;                                /**< Payload 实际长度 */
    uint8_t payload[PROTO_MAX_PAYLOAD];                 /**< Payload 原始字节 */
} proto_cmd_t;

/*===========================================================================
 * 各命令的格式化数据
 *===========================================================================*/

/** CMD_MOVE - 直线运动 */
typedef struct {
    uint8_t  direction;                                 /**< DIR_FORWARD / DIR_BACKWARD */
    uint8_t  speed;                                     /**< 速度百分比 0~100 */
    uint16_t duration_ms;                               /**< 持续时长 (ms), 0=持续运动 */
} cmd_move_t;

/** CMD_TURN - 原地转向 */
typedef struct {
    uint8_t direction;                                  /**< TURN_LEFT / TURN_RIGHT */
    uint8_t angle;                                      /**< 目标角度 °, 1~180 */
    uint8_t speed;                                      /**< 转向速度百分比 1~100 */
} cmd_turn_t;

/** CMD_ARC - 弧线运动 */
typedef struct {
    uint8_t  direction;                                 /**< DIR_FORWARD / DIR_BACKWARD */
    uint8_t  turn_dir;                                  /**< TURN_LEFT / TURN_RIGHT */
    uint8_t  speed;                                     /**< 线速度百分比 1~100 */
    uint16_t radius_cm;                                 /**< 转弯半径 (cm), 1~65535 */
} cmd_arc_t;

/** CMD_HEARTBEAT - 心跳 */
typedef struct {
    uint8_t seq;                                        /**< 递增序号 0~255 */
} cmd_heartbeat_t;

/*===========================================================================
 * 状态数据结构 (MCU → Phone)
 *===========================================================================*/

/** STA_STATUS - 状态上报 */
typedef struct {
    uint8_t battery;                                    /**< 电量百分比 0~100 */
    uint8_t state;                                      /**< 状态位掩码 */
    uint8_t error;                                      /**< 故障码 */
    uint8_t cmd_echo;                                   /**< 当前执行的命令码 */
} sta_status_t;

/** STA_HB_ACK - 心跳应答 */
typedef struct {
    uint8_t seq;                                        /**< 回显心跳序号 */
    uint8_t battery;                                    /**< 电量 */
    uint8_t state;                                      /**< 状态位掩码 */
} sta_hb_ack_t;

/** STA_CMD_DONE - 指令执行完毕 */
typedef struct {
    uint8_t cmd_echo;                                   /**< 完成的命令码 */
    uint8_t result;                                     /**< 结果码 */
} sta_cmd_done_t;

/*===========================================================================
 * 命令回调类型
 *===========================================================================*/

/**
 * @brief 命令处理回调。
 *        当协议解析器收到一条完整、校验正确的命令帧后调用。
 *        MCU 在这里分发处理各命令，并通过 proto_cmd_response_* 发送回复。
 * @param cmd  解析后的命令
 */
typedef void (*proto_cmd_handler_t)(const proto_cmd_t *cmd);

/*===========================================================================
 * API: CRC8 & 帧解析
 *===========================================================================*/

/**
 * @brief CRC-8/ROHC 校验计算
 * @param data  数据缓冲区
 * @param len   数据长度 (字节)
 * @return CRC8 值
 */
uint8_t protocol_crc8(const uint8_t *data, uint8_t len);

/**
 * @brief 向协议解析器喂入一个字节。
 *        对端每收到一个 BLE/UART 字节就调用此函数。
 *        收到完整有效帧时会自动回调 handler。
 * @param byte     接收到的单字节
 * @param handler  命令处理回调 (非 NULL)
 */
void protocol_feed_byte(uint8_t byte, proto_cmd_handler_t handler);

/**
 * @brief 重置解析器状态机 (如 BLE 断连后重连时)
 */
void protocol_reset(void);

/*===========================================================================
 * API: 帧构建 (MCU → Phone)
 *===========================================================================*/

/**
 * @brief 构建 STA_STATUS 帧
 * @param status  状态数据
 * @param buf     输出缓冲区 (需 >= PROTO_MAX_FRAME_SIZE 字节)
 * @param out_len 输出: 实际帧长度
 */
void protocol_build_status(const sta_status_t *status, uint8_t *buf, uint8_t *out_len);

/**
 * @brief 构建 STA_HB_ACK 帧
 * @param ack     心跳应答数据
 * @param buf     输出缓冲区
 * @param out_len 输出: 实际帧长度
 */
void protocol_build_hb_ack(const sta_hb_ack_t *ack, uint8_t *buf, uint8_t *out_len);

/**
 * @brief 构建 STA_CMD_DONE 帧
 * @param done    指令完成数据
 * @param buf     输出缓冲区
 * @param out_len 输出: 实际帧长度
 */
void protocol_build_cmd_done(const sta_cmd_done_t *done, uint8_t *buf, uint8_t *out_len);

/*===========================================================================
 * API: Payload 解析 (将原始字节转换为各命令/状态结构体)
 *===========================================================================*/

/**
 * @brief 从 proto_cmd_t 解析 CMD_MOVE 参数
 * @param cmd  须确保 cmd->cmd == CMD_MOVE
 * @param out  输出 move 参数
 * @return 解析是否成功 (payload_len 检查)
 */
bool protocol_parse_move(const proto_cmd_t *cmd, cmd_move_t *out);

/**
 * @brief 从 proto_cmd_t 解析 CMD_TURN 参数
 */
bool protocol_parse_turn(const proto_cmd_t *cmd, cmd_turn_t *out);

/**
 * @brief 从 proto_cmd_t 解析 CMD_ARC 参数
 */
bool protocol_parse_arc(const proto_cmd_t *cmd, cmd_arc_t *out);

/**
 * @brief 从 proto_cmd_t 解析 CMD_HEARTBEAT 参数
 */
bool protocol_parse_heartbeat(const proto_cmd_t *cmd, cmd_heartbeat_t *out);

/**
 * @brief 判断是否为下行命令 (Phone → MCU)
 */
static inline bool protocol_is_downstream(uint8_t cmd)
{
    return (cmd >= 0x10 && cmd <= 0x50);
}

/**
 * @brief 判断是否为上行状态 (MCU → Phone)
 */
static inline bool protocol_is_upstream(uint8_t cmd)
{
    return (cmd >= 0x80);
}

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
