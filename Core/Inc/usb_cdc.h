/**
 * @file    usb_cdc.h
 * @brief   USB CDC ACM device driver for STM32F103 (no ST USB Device Library required)
 *
 * Uses HAL PCD directly. Presents a virtual COM port to the USB host (Android phone).
 *
 * API:
 *   usb_cdc_init()        — initialize USB CDC, connect to host
 *   usb_cdc_send()        — send data to host (non-blocking, copies data)
 *   usb_cdc_set_rx_cb()   — register callback for received bytes
 */
#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum packet size for Bulk endpoints */
#define CDC_DATA_MAX_PACKET_SIZE  64

/**
 * @brief Receive callback type: called for each byte received from host.
 * @param byte  received byte
 */
typedef void (*usb_cdc_rx_cb_t)(uint8_t byte);

/**
 * @brief Initialize USB CDC ACM device.
 *
 * Must be called after HAL_Init(), SystemClock_Config(), and MX_USB_PCD_Init().
 * This configures PMA buffers, opens endpoints, and connects to the USB bus.
 */
void usb_cdc_init(void);

/**
 * @brief Send data to USB host (non-blocking).
 *
 * Copies data to an internal buffer and starts a Bulk IN transfer.
 * If a transfer is already in progress, the new data is queued (overwrites
 * pending data).
 *
 * @param data  data to send
 * @param len   length (0 ~ CDC_DATA_MAX_PACKET_SIZE)
 */
void usb_cdc_send(const uint8_t *data, uint8_t len);

/**
 * @brief Register a callback for every byte received from the host.
 * @param cb   callback (NULL to disable)
 */
void usb_cdc_set_rx_cb(usb_cdc_rx_cb_t cb);

/**
 * @brief Check if USB is configured and ready for data transfer.
 */
bool usb_cdc_is_ready(void);

/**
 * @brief Print USB debug state to SWO (printf).
 *        Shows: configured state, reset/setup/dataout counters.
 *        Call this from the maintenance task to monitor USB activity.
 */
void usb_cdc_print_debug(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_H */
